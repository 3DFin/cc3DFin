// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright 2023-2025 Carlos Cabo <carloscabo@uniovi.es>
#include "ground.hpp"

// local
#include "connected_components.hpp"
#include "types.hpp"
#include "voxel.hpp"

// spdlog
#include <spdlog/spdlog.h>

// CSF
#include <CSF.h>
#include <PointCloud.h>

// nanoflann
#include <nanoflann.hpp>

// Taskflow
#include <taskflow/algorithm/for_each.hpp>
#include <taskflow/taskflow.hpp>

// System
#include <cstddef>

// #define HN3DFIN_BARYCENTRIC_INTERPOLATION

namespace lib3dfin
{

	HeightNormalization::HeightNormalization(const PointCloud3& point_cloud,
	                                         Parameters         params,
	                                         tf::Executor&      executor)
	    : point_cloud_(point_cloud)
	    , params_(params)
	    , executor_(executor)
	{
	}

	// Main public function
	Eigen::VectorXd HeightNormalization::normalize()
	{
		spdlog::info("[HeighNorm] Computing normalization with CSF algorithm...");
		if (params_.denoise_point_cloud)
		{
			const PointCloud3 denoised_point_cloud_ = denoiseCloud();
			generateDTM(denoised_point_cloud_);
		}
		else
		{
			generateDTM(point_cloud_);
		}

		if (params_.clean_dtm)
		{

#ifdef HN3DFIN_BARYCENTRIC_INTERPOLATION
			smoothDTMmedian();

#else
			// cleanDTMmad(); old behavior... can't use exported mesh
			smoothDTMmedian();
#endif
		}

		constexpr size_t N_NEIGHBORS = 3;
		const size_t     n_points    = point_cloud_.rows();

		if (n_points < N_NEIGHBORS)
			throw std::runtime_error("Input DTM too small (less than 3 points).");

		Eigen::VectorXd normalized_heights(n_points);
		using kd_tree_t         = nanoflann::KDTreeEigenMatrixAdaptor<PointCloud2, 2, nanoflann::metric_L2_Simple>;
		const PointCloud2& dtm2 = dtm_.leftCols<2>();
		kd_tree_t          kd_tree(2, dtm2, 10);
		tf::Executor       executor;
		tf::Taskflow       taskflow;

		const size_t num_workers = executor.num_workers();

		// Pre-allocate worker local storage to avoid allocations in loop
		std::vector<Eigen::Index> neighbors_buffer(N_NEIGHBORS * num_workers);
		std::vector<double>       dists_buffer(N_NEIGHBORS * num_workers);
		std::vector<double>       heights_buffer(N_NEIGHBORS * num_workers);
#ifdef HN3DFIN_BARYCENTRIC_INTERPOLATION
		spdlog::info("[HeighNorm] Normalization using barycentric interpolation")
#else
		spdlog::info("[HeighNorm] Normalization using IDW interpolation");
#endif

		    taskflow.for_each_index(
		        size_t(0), n_points, size_t(1), [&](size_t i)
		        {
			const int    worker_id = executor.this_worker_id();
			const size_t offset    = worker_id * N_NEIGHBORS;

			// Use the dedicated storage of the worker
			Eigen::Index* indices = neighbors_buffer.data() + offset;
			double*       dists   = dists_buffer.data() + offset;
			double*       weights = heights_buffer.data() + offset;

			kd_tree.index_->knnSearch(point_cloud_.row(i).data(), N_NEIGHBORS, indices, dists);

#ifdef HN3DFIN_BARYCENTRIC_INTERPOLATION

            // Barycentric Interpolation
            // Assuming the DTM grid is regular and uniform, its triangulation is Delaunay.
            // In this case, finding the 3 nearest neighbors (NN) of a query point in 2D
            // is equivalent to identifying the 3 vertices of the Delaunay triangle
            // that encloses the query point (due to the circumcircle property of Delaunay triangulation).
            // Barycentric interpolation can then be computed using these 3 points.
          	const Eigen::Vector2d& A = dtm2.row(indices[0]);
           	const Eigen::Vector2d& B = dtm2.row(indices[1]);
           	const Eigen::Vector2d& C = dtm2.row(indices[2]);
           	const Eigen::Vector2d& P = point_cloud_.row(i).head<2>();

           	Eigen::Matrix2d linear_system;
           	linear_system << B(0) - A(0), C(0) - A(0),
           	B(1) - A(1), C(1) - A(1);
           	const Eigen::Vector2d b = P - A;
           	const Eigen::Vector2d uv = linear_system.inverse() * b;
           	double u = 1.0 - uv(0) - uv(1);
           	double v = uv(0);
           	double w = uv(1);
           	double weighted_z = u * dtm_(indices[0], 2) + v * dtm_(indices[1], 2)  + w * dtm_(indices[2], 2);
#else // 3DFIN_IDW_INTERPOLATION

            // Convert squared distances to actual distances and compute weights
		        double sum_weights = 0.0;
			    for (size_t j = 0; j < N_NEIGHBORS; ++j)
			    {
				    weights[j] = std::sqrt(dists[j]); // nanoflann dist are squared
				    sum_weights += weights[j];
			    }

			    // Normalize weights and compute weighted average Z
			    double weighted_z = 0.0;
			    const double inv_sum_weights = 1.0 / sum_weights;
			    for (size_t j = 0; j < N_NEIGHBORS; ++j)
			    {
				    const double normalized_weight = weights[j] * inv_sum_weights;
				    weighted_z += normalized_weight * dtm_(indices[j], 2);
			    }
#endif

			    normalized_heights(i) = point_cloud_(i, 2) - weighted_z; },
		        tf::StaticPartitioner()); // worker ID
		executor.run(taskflow).get();
		spdlog::info("[HeighNorm] End CSF computation...");
		return normalized_heights;
	}

	PointCloud3 HeightNormalization::denoiseCloud()
	{
		const auto [voxel_cloud, cloud_to_vox] = voxelize(point_cloud_, params_.denoise_resolution, params_.denoise_resolution, executor_, true);

		const auto cluster_labels = connected_components(voxel_cloud, params_.denoise_resolution * std::sqrt(3.0) + 1e-6, params_.denoise_minimum_points, executor_);

		// Count occurrences of each cluster label
		std::unordered_map<int32_t, uint32_t> label_counts;
		for (const auto id_vox : cloud_to_vox)
		{
			++label_counts[cluster_labels(id_vox)];
		}

		if (label_counts.size() == 1 && label_counts.count(-1))
		{
			throw std::runtime_error("No valid clusters found.");
		}

		// Identify large clusters (label ≠ -1 and count > min_points)
		// uint32_t because we do not insert -1
		std::set<uint32_t> large_clusters;
		for (const auto& [label, count] : label_counts)
		{
			if (label > -1 && count > params_.denoise_minimum_points)
			{
				large_clusters.insert(label);
			}
		}

		std::vector<Eigen::Index> valid_indices;

		// hint to avoid too small allocation
		// TODO: maybe prefer a mask (more efficient - less allocations - but uses more memory...)
		valid_indices.reserve(large_clusters.size() * params_.denoise_minimum_points);
		for (Eigen::Index point_id = 0; point_id < point_cloud_.rows(); ++point_id)
		{
			const auto& voxel_id = cloud_to_vox(point_id);
			if (large_clusters.count(cluster_labels[voxel_id]))
			{
				valid_indices.push_back(point_id);
			}
		}

		// Build filtered cloud
		PointCloud3 clust_cloud(valid_indices.size(), 3);
		for (size_t i = 0; i < valid_indices.size(); ++i)
		{
			clust_cloud.row(i) = point_cloud_.row(valid_indices[i]);
		}

		return clust_cloud;
	}

	void HeightNormalization::generateDTM(const PointCloud3& dtm_point_cloud_)
	{
		CSF csf;

		csf.params.smooth_slope     = true;
		csf.params.cloth_resolution = static_cast<double>(params_.cloth_resolution);
		csf.params.verbose          = true;

		auto& csf_pc = csf.getPointCloud();
		csf_pc.clear();
		csf_pc.resize(dtm_point_cloud_.rows());

		for (Eigen::Index point_id = 0; point_id < dtm_point_cloud_.rows(); ++point_id)
		{
			csf_pc[point_id] = {dtm_point_cloud_(point_id, 0), -dtm_point_cloud_(point_id, 2), dtm_point_cloud_(point_id, 1)};
		}

		const auto  cloth     = csf.runClothSimulation();
		const auto& particles = cloth.getParticles();

		auto [width, height] = cloth.getGridSize();
		width_               = width;
		height_              = height;
		spdlog::info("[HeighNorm] CSF grid size {}x{}", width_, height_);

		dtm_ = PointCloud3(particles.size(), 3);
		for (size_t particle_id = 0; particle_id < particles.size(); ++particle_id)
		{
			const auto& particle = particles[particle_id];
			dtm_(particle_id, 0) = particle.initial_pos.f[0];
			dtm_(particle_id, 1) = particle.initial_pos.f[2];
			dtm_(particle_id, 2) = -particles[particle_id].height;
		}
	}

	void HeightNormalization::smoothDTMmedian()
	{
		spdlog::info("[HeighNorm] smooth DTM using 3x3 Median filter");
		// Apply 3x3 median filter to inner cells
		std::vector<double>                                                  window(9, 0.0);
		Eigen::Map<Eigen::MatrixXd, Eigen::Unaligned, Eigen::InnerStride<3>> depth_map(dtm_.data() + 2, height_, width_);
		for (int y = 1; y < height_ - 1; ++y)
		{
			for (int x = 1; x < width_ - 1; ++x)
			{

				size_t window_id = 0;
				for (int dy = -1; dy <= 1; ++dy)
				{
					for (int dx = -1; dx <= 1; ++dx)
					{
						window[window_id++] = depth_map(y + dy, x + dx);
					}
				}
				std::nth_element(window.begin(), window.begin() + 4, window.end());
				depth_map(y, x) = window[4]; // median of 9 values
			}
		}
	}

	void HeightNormalization::cleanDTMmad()
	{
		spdlog::info("[HeighNorm] clean DTM using 2*MADs filter");
		constexpr size_t N_NEIGHBORS      = 15;
		constexpr size_t HALF_N_NEIGHBORS = N_NEIGHBORS / 2;
		constexpr double MAD_FACTOR       = 2.0;

		const size_t n_points = dtm_.rows();

		if (n_points < N_NEIGHBORS)
			// TODO catch this in the GUI
			throw std::runtime_error("Input DTM too small (less than 15 points).");

		if (n_points == N_NEIGHBORS)
			spdlog::warn("[HeighNorm] Input DTM has exactly 15 points.");

		const size_t half_n_points = n_points / 2;

		using kd_tree_t         = nanoflann::KDTreeEigenMatrixAdaptor<PointCloud2, 2, nanoflann::metric_L2_Simple>;
		const PointCloud2& dtm2 = dtm_.leftCols<2>();
		kd_tree_t          kd_tree(2, dtm2, 10);

		tf::Taskflow taskflow;

		Eigen::VectorXd abs_devs(n_points);
		// Get number of workers and pre-allocate vectors for each worker
		const size_t              num_workers = executor_.num_workers();
		std::vector<Eigen::Index> neighbors_buffer(N_NEIGHBORS * num_workers);
		std::vector<double>       dists_buffer(N_NEIGHBORS * num_workers);
		std::vector<double>       heights_buffer(N_NEIGHBORS * num_workers);

		taskflow.for_each_index(
		    size_t(0), n_points, size_t(1), [&](size_t i)
		    {
			    // Get current worker ID and calculate offset into pre-allocated vectors
			    const int worker_id = executor_.this_worker_id();
			    const size_t offset = worker_id * N_NEIGHBORS;

				// Thread local storage
			    Eigen::Index* neighbors = neighbors_buffer.data() + offset;
			    double* dists = dists_buffer.data() + offset;
			    double* heights = heights_buffer.data() + offset;

			    kd_tree.index_->knnSearch(dtm2.row(i).data(), N_NEIGHBORS, neighbors, dists);

			    for (size_t j = 0; j < N_NEIGHBORS; ++j)
			    {
				    heights[j] = dtm_(neighbors[j], 2);
			    }
				// N_Neighbors is always odd
			    std::nth_element(heights, heights + HALF_N_NEIGHBORS, heights + N_NEIGHBORS);

			    const double median_z = heights[HALF_N_NEIGHBORS];
			    abs_devs(i) = std::abs(dtm_(i, 2) - median_z); },
		    tf::StaticPartitioner()); // StaticPartitioner is important to have consistent worker's ID

		executor_.run(taskflow).get();

		// Compute MAD (median of absolute deviations)
		Eigen::VectorXd abs_devs_copy = abs_devs;

		double mad = 0.0;
		if (n_points % 2 != 0)
		{
			std::nth_element(std::begin(abs_devs_copy), std::begin(abs_devs_copy) + half_n_points, std::end(abs_devs_copy));
			mad = abs_devs_copy[half_n_points];
		}
		else
		{
			std::partial_sort(std::begin(abs_devs_copy), std::begin(abs_devs_copy) + half_n_points, std::end(abs_devs_copy));
			mad = (abs_devs_copy[half_n_points - 1] + abs_devs_copy[half_n_points]) / 2.0;
		}

		// Filter points
		const double mad_threshold    = MAD_FACTOR * mad;
		const auto   clean_indicator  = abs_devs.array() < mad_threshold;
		const auto   num_valid_points = clean_indicator.count();

		PointCloud3  clean_points(num_valid_points, 3);
		Eigen::Index clean_id = 0;
		for (Eigen::Index point_id = 0; point_id < dtm_.rows(); ++point_id)
		{
			if (clean_indicator(point_id))
				clean_points.row(clean_id++) = dtm_.row(point_id);
		}

		dtm_ = std::move(clean_points);
	}

	std::pair<std::vector<size_t>, PointCloud3> HeightNormalization::exportDTM()
	{
		std::vector<size_t> tri_indices;
		const size_t        num_triangles = (width_ - 1) * (height_ - 1) * 2;
		tri_indices.resize(num_triangles * 3, 0);

		// mesh export code taken from CC.
		// A---D
		// | / |
		// B---C
		for (size_t x = 0; x < width_ - 1; ++x)
		{
			for (size_t y = 0; y < height_ - 1; ++y)
			{
				size_t A = y * width_ + x;
				size_t B = A + 1;
				size_t D = A + width_;
				size_t C = D + 1;

				size_t base_id           = 6 * (x + y * (width_ - 1));
				tri_indices[base_id]     = A;
				tri_indices[base_id + 1] = B;
				tri_indices[base_id + 2] = D;
				tri_indices[base_id + 3] = D;
				tri_indices[base_id + 4] = B;
				tri_indices[base_id + 5] = C;
			}
		}

		return {std::move(tri_indices), dtm_};
	}

	std::pair<bool, double> HeightNormalization::checkHeightNormDiscrepancy(const PointCloud3& point_cloud, const Eigen::VectorXd& z0, double original_area, tf::Executor& executor, double res_xy, double z_min, double z_max, double threshold)
	{
		assert(z_min < z_max);
		assert(original_area > 0);
		assert(threshold > 0 && threshold < 1);

		// Compute the z resolution as a function of z_max - z_min
		const double res_z = (z_max - z_min) * 1.01;

		const ArrayMask pseudo_ground_mask = ((z0.array() >= z_min) && (z0.array() <= z_max));

		auto        pseudo_ground_point_count = pseudo_ground_mask.count();
		PointCloud3 pseudo_ground_cloud(pseudo_ground_point_count, 3);

		Eigen::Index filtered_point_id = 0;
		for (Eigen::Index point_id = 0; point_id < z0.size(); ++point_id)
		{
			if (pseudo_ground_mask(point_id))
			{
				pseudo_ground_cloud.row(filtered_point_id).head<2>() = point_cloud.row(point_id).head<2>();
				pseudo_ground_cloud.row(filtered_point_id++)(2)      = z0(point_id);
			}
		}

		const auto [voxel_cloud, _] = voxelize(pseudo_ground_cloud, res_xy, res_z, executor, false);

		//   # Area of the voxelated ground slice (n of voxels * area of voxel base)
		double slice_area = voxel_cloud.rows() * res_xy * res_xy;

		double threshold_difference = threshold * original_area;

		double area_difference = std::abs(original_area - slice_area);

		//  TODO: In very rare occasions, the slice area could be larger than the original
		//  area. The function should account for that, and return a different kind of
		//  warning for those situations (and its threshold could be different).
		//  For instance, if the original area has been computed through a grid of voxels
		//  (as this function does to compute slice_area) using a smaller voxel size,
		//  this could happen. We haven't tested it yet as we do not have access
		//  to any point clouds where this situation happens.

		//  Check if the difference is greater than 10 % of the first number
		return {area_difference >= threshold_difference, (area_difference * 100 / original_area)};
	}

} // namespace lib3dfin
