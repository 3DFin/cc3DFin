// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright 2023-2025 Carlos Cabo <carloscabo@uniovi.es>
#include "ground.hpp"

// local
#include "connected_components.hpp"
#include "types.hpp"
#include "voxel.hpp"

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

namespace lib3dfin
{

	HeightNormalization::HeightNormalization(const PointCloud3& point_cloud,
	                                         Parameters         params)
	    : point_cloud_(point_cloud)
	    , params_(std::move(params))
	{
	}

	// Main public function
	Eigen::VectorX<double> HeightNormalization::normalize()
	{
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
			cleanDTM();
		}

		constexpr size_t N_NEIGHBORS = 3;
		const size_t     n_points    = point_cloud_.rows();

		if (n_points < N_NEIGHBORS)
			throw std::runtime_error("Input DTM too small (less than 3 points).");

		Eigen::VectorX<double> normalized_heights(n_points);
		using kd_tree_t         = nanoflann::KDTreeEigenMatrixAdaptor<PointCloud2, 2, nanoflann::metric_L2_Simple>;
		const PointCloud2& dtm2 = dtm_.template leftCols<2>();
		kd_tree_t          kd_tree(2, dtm2, 10);
		tf::Executor       executor;
		tf::Taskflow       taskflow;

		const size_t num_workers = executor.num_workers();

		// Pre-allocate worker local storage to avoid allocations in loop
		std::vector<Eigen::Index> neighbors_buffer(N_NEIGHBORS * num_workers);
		std::vector<double>       dists_buffer(N_NEIGHBORS * num_workers);
		std::vector<double>       heights_buffer(N_NEIGHBORS * num_workers);

		taskflow.for_each_index(
		    size_t(0), n_points, size_t(1), [&](size_t i)
		    {
			    const int worker_id = executor.this_worker_id();
				const size_t offset = worker_id * N_NEIGHBORS;

			    // Use this worker's dedicated storage
			    Eigen::Index* indices = neighbors_buffer.data() + offset;
			    double* dists = dists_buffer.data() + offset;
			    double* weights = heights_buffer.data() + offset;

			    kd_tree.index_->knnSearch(point_cloud_.row(i).data(), N_NEIGHBORS, indices, dists);

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

			    normalized_heights(i) = point_cloud_(i, 2) - weighted_z; },
		    tf::StaticPartitioner()); // worker ID
		executor.run(taskflow).get();
		return normalized_heights;
	}

	PointCloud3 HeightNormalization::denoiseCloud()
	{
		const auto [voxel_cloud, cloud_to_vox] = voxelize(point_cloud_, params_.denoise_resolution, params_.denoise_resolution, true);

		const auto cluster_labels = connected_components(voxel_cloud, params_.denoise_resolution * std::sqrt(3.0) + 1e-6, params_.denoise_minimum_points);

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
		// uint32_t because we
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
			csf_pc[point_id] = {static_cast<double>(dtm_point_cloud_(point_id, 0)), static_cast<double>(-dtm_point_cloud_(point_id, 2)), static_cast<double>(dtm_point_cloud_(point_id, 1))};
		}

		const auto  cloth     = csf.runClothSimulation();
		const auto& particles = cloth.getParticles();
		dtm_                  = PointCloud3(particles.size(), 3);
		for (size_t particle_id = 0; particle_id < particles.size(); ++particle_id)
		{
			const auto& particle = particles[particle_id];
			dtm_(particle_id, 0) = particle.initial_pos.f[0];
			dtm_(particle_id, 1) = particle.initial_pos.f[2];
			dtm_(particle_id, 2) = -particles[particle_id].height;
		}
	}

	void HeightNormalization::cleanDTM()
	{
		constexpr size_t N_NEIGHBORS      = 15;
		constexpr size_t HALF_N_NEIGHBORS = N_NEIGHBORS / 2;
		constexpr double MAD_FACTOR       = 2.0;

		const size_t n_points = dtm_.rows();

		if (n_points < N_NEIGHBORS)
			// TODO catch this in the GUI
			throw std::runtime_error("Input DTM too small (less than 15 points).");

		if (n_points == N_NEIGHBORS)
			std::cerr << "Warning: Input DTM has exactly 15 points.\n";

		const size_t half_n_points = n_points / 2;

		using kd_tree_t         = nanoflann::KDTreeEigenMatrixAdaptor<PointCloud2, 2, nanoflann::metric_L2_Simple>;
		const PointCloud2& dtm2 = dtm_.template leftCols<2>();
		kd_tree_t          kd_tree(2, dtm2, 10);

		tf::Executor executor;
		tf::Taskflow taskflow;

		std::vector<double> abs_devs(n_points);
		// Get number of workers and pre-allocate vectors for each worker
		const size_t              num_workers = executor.num_workers();
		std::vector<Eigen::Index> neighbors_buffer(N_NEIGHBORS * num_workers);
		std::vector<double>       dists_buffer(N_NEIGHBORS * num_workers);
		std::vector<double>       heights_buffer(N_NEIGHBORS * num_workers);

		taskflow.for_each_index(
		    size_t(0), n_points, size_t(1), [&](size_t i)
		    {
			    // Get current worker ID and calculate offset into pre-allocated vectors
			    const int worker_id = executor.this_worker_id();
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
			    abs_devs[i] = std::abs(dtm_(i, 2) - median_z); },
		    tf::StaticPartitioner()); // StaticPartitioner is important to have consistent worker's ID

		executor.run(taskflow).get();

		// Compute MAD (median of absolute deviations)
		std::vector<double> abs_devs_copy = abs_devs;

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
		// TODO parallelize, and does not allocate valid_indices?
		std::vector<Eigen::Index> valid_indices;
		valid_indices.reserve(n_points); // this should not be too far...
		for (Eigen::Index i = 0; i < n_points; ++i)
		{
			if (abs_devs[i] < MAD_FACTOR * mad)
			{
				valid_indices.push_back(i);
			}
		}

		PointCloud3 clean_points(valid_indices.size(), 3);
		for (size_t i = 0; i < valid_indices.size(); ++i)
		{
			clean_points.row(i) = dtm_.row(valid_indices[i]);
		}

		dtm_ = std::move(clean_points);
	}

} // namespace lib3dfin
