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

// System
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <set>

namespace lib3dfin
{

	template <typename real_t>
	HeightNormalization<real_t>::HeightNormalization(const RefPointCloud<real_t>& point_cloud,
	                                                 Parameters                   params)
	    : point_cloud_(point_cloud)
	    , params_(std::move(params))
	{
	}

	// Main public function
	template <typename real_t>
	Eigen::VectorX<real_t> HeightNormalization<real_t>::normalize()
	{
		if (params_.denoise_point_cloud)
		{
			PointCloud3<real_t> denoised_point_cloud_ = denoiseCloud();
			generateDTM(RefPointCloud<real_t>(denoised_point_cloud_));
		}
		else
		{
			generateDTM(point_cloud_);
		}

		if (params_.clean_dtm)
		{
			cleanDTM();
		}

		const size_t n_points    = point_cloud_.rows();
		const size_t n_neighbors = 3;

		if (n_points < n_neighbors)
			throw std::runtime_error("Input DTM too small (less than 3 points).");

		Eigen::VectorX<real_t> normalized_heights(n_points);
		using kd_tree_t                 = nanoflann::KDTreeEigenMatrixAdaptor<PointCloud2<real_t>, 2, nanoflann::metric_L2_Simple>;
		const PointCloud2<real_t>& dtm2 = dtm_.template leftCols<2>();
		kd_tree_t                  kd_tree(2, dtm2, 10);

		std::vector<Eigen::Index> indices(n_neighbors);
		std::vector<real_t>       dists(n_neighbors);

		for (size_t i = 0; i < n_points; ++i)
		{
			nanoflann::KNNResultSet<real_t, Eigen::Index> result(n_neighbors);
			result.init(indices.data(), dists.data());
			const real_t* query_pt = point_cloud_.row(i).data();
			kd_tree.index_->findNeighbors(result, query_pt);

			// Convert squared distances to actual distances
			std::vector<real_t> weights(n_neighbors);
			real_t              sum_weights = 0.0;
			// TODO, here we use distances to compute weights like the original code
			// but it should be inverse distances...
			for (size_t j = 0; j < n_neighbors; ++j)
			{
				weights[j] = std::sqrt(dists[j]); // nanoflann dist are squared
				sum_weights += weights[j];
			}

			// Normalize weights
			for (real_t& w : weights)
				w /= sum_weights;

			// Compute weighted average Z from DTM
			real_t weighted_z = 0.0;
			for (size_t j = 0; j < n_neighbors; ++j)
			{
				weighted_z += weights[j] * dtm_(indices[j], 2);
			}

			normalized_heights(i) = point_cloud_(i, 2) - weighted_z;
		}

		return normalized_heights;
	}

	template <typename real_t>
	PointCloud3<real_t> HeightNormalization<real_t>::denoiseCloud()
	{
		const auto [voxel_cloud, cloud_to_vox] = voxelize(point_cloud_, params_.denoise_resolution, params_.denoise_resolution, true);

		const auto cluster_labels = connected_components(RefPointCloud<real_t>(voxel_cloud), real_t(params_.denoise_resolution * std::sqrt(real_t(3)) + 1e-6), params_.denoise_minimum_points);

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
		valid_indices.reserve(large_clusters.size());
		for (Eigen::Index point_id = 0; point_id < point_cloud_.rows(); ++point_id)
		{
			const auto& voxel_id = cloud_to_vox(point_id);
			if (large_clusters.count(cluster_labels[voxel_id]))
			{
				valid_indices.push_back(point_id);
			}
		}

		// Build filtered cloud
		PointCloud3<real_t> clust_cloud(valid_indices.size(), 3);
		for (size_t i = 0; i < valid_indices.size(); ++i)
		{
			clust_cloud.row(i) = point_cloud_.row(valid_indices[i]);
		}

		return clust_cloud;
	}

	template <typename real_t>
	void HeightNormalization<real_t>::generateDTM(const RefPointCloud<real_t>& dtm_point_cloud_)
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
		dtm_                  = PointCloud3<real_t>(particles.size(), 3);
		for (size_t particle_id = 0; particle_id < particles.size(); ++particle_id)
		{
			const auto& particle = particles[particle_id];
			dtm_(particle_id, 0) = static_cast<real_t>(particle.initial_pos.f[0]);
			dtm_(particle_id, 1) = static_cast<real_t>(particle.initial_pos.f[2]);
			dtm_(particle_id, 2) = static_cast<real_t>(-particles[particle_id].height);
		}
	}

	template <typename real_t>
	void HeightNormalization<real_t>::cleanDTM()
	{
		const size_t n_points    = dtm_.rows();
		const size_t n_neighbors = 15;

		if (n_points < n_neighbors)
			// TODO catch this in the GUI
			throw std::runtime_error("Input DTM too small (less than 15 points).");

		if (n_points == n_neighbors)
			std::cerr << "Warning: Input DTM has exactly 15 points.\n";

		const size_t half_n_points    = n_points / 2;
		const size_t half_n_neighbors = n_neighbors / 2;
		const real_t mad_factor       = 2.0;

		using kd_tree_t                 = nanoflann::KDTreeEigenMatrixAdaptor<PointCloud2<real_t>, 2, nanoflann::metric_L2_Simple>;
		const PointCloud2<real_t>& dtm2 = dtm_.template leftCols<2>();
		kd_tree_t                  kd_tree(2, dtm2, 10);

		std::vector<Eigen::Index> neighbors(n_neighbors);
		std::vector<real_t>       dists(n_neighbors);
		std::vector<real_t>       abs_devs(n_points);
		std::vector<real_t>       heights(n_neighbors);

		for (size_t i = 0; i < n_points; ++i)
		{
			nanoflann::KNNResultSet<real_t, Eigen::Index> result(n_neighbors);
			result.init(neighbors.data(), dists.data());
			kd_tree.index_->findNeighbors(result, dtm2.row(i).data());

			for (size_t j = 0; j < n_neighbors; ++j)
			{
				heights[j] = dtm_(neighbors[j], 2);
			}
			std::nth_element(std::begin(heights), std::begin(heights) + half_n_neighbors, std::end(heights));

			const real_t median_z = heights[half_n_neighbors];
			abs_devs[i]           = std::abs(dtm_(i, 2) - median_z);
		}

		// Compute MAD (median of absolute deviations)
		std::vector<real_t> abs_devs_copy = abs_devs;
		std::nth_element(std::begin(abs_devs_copy), std::begin(abs_devs_copy) + half_n_points, std::end(abs_devs_copy));
		const real_t mad = abs_devs_copy[half_n_points];

		// Filter points
		// TODO parallelize, and does not allocate valid_indices?
		std::vector<Eigen::Index> valid_indices;
		valid_indices.reserve(n_points); // this should not be too far...
		for (Eigen::Index i = 0; i < n_points; ++i)
		{
			if (abs_devs[i] < mad_factor * mad)
			{
				valid_indices.push_back(i);
			}
		}

		PointCloud3<real_t> clean_points(valid_indices.size(), 3);
		for (size_t i = 0; i < valid_indices.size(); ++i)
		{
			clean_points.row(i) = dtm_.row(valid_indices[i]);
		}

		dtm_ = clean_points;
	}

	// Required: Explicit instantiations if using in separate translation units
	template class HeightNormalization<float>;
	template class HeightNormalization<double>;

} // namespace lib3dfin
