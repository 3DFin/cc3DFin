// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright 2023-2025 Carlos Cabo <carloscabo@uniovi.es>

#include "PointCloud.h"
#include "connected_components.hpp"
#include "types.hpp"
#include "voxel.hpp"

// CSF
#include <CSF.h>

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
	PointCloud3<real_t> clean_ground(const PointCloud3<real_t>& point_cloud, const real_t resolution, const real_t minimum_points)
	{
		auto [voxel_cloud, cloud_to_vox] = voxelize(point_cloud, resolution, resolution, true);
		// eps = res_ground * math.sqrt(3) + 1e-6
		auto cluster_labels = connected_components(voxel_cloud, resolution * std::sqrt(3) + 1e-6, minimum_points);

		// Count occurrences of each cluster label
		std::unordered_map<int32_t, uint32_t> label_counts;
		for (const auto id_vox : cloud_to_vox)
		{
			++label_counts[cluster_labels[id_vox]];
		}

		// Identify large clusters (label ≠ -1 and count > min_points)
		// uint32_t because we
		std::set<uint32_t> large_clusters;
		for (const auto& [label, count] : label_counts)
		{
			if (label > -1 && count > minimum_points)
			{
				large_clusters.insert(label);
			}
		}

		std::vector<Eigen::Index> valid_indices;
		// hint to avoid to small allocation
		valid_indices.reserve(large_clusters.size());
		for (Eigen::Index point_id = 0; point_id < point_cloud.rows(); ++point_id)
		{
			if (large_clusters.count(cluster_labels[cloud_to_vox(point_id)]))
			{
				valid_indices.push_back(point_id);
			}
		}

		// Build filtered cloud
		PointCloud3<real_t> clust_cloud(valid_indices.size(), 3);
		for (size_t i = 0; i < valid_indices.size(); ++i)
		{
			clust_cloud.row(i) = point_cloud.row(valid_indices[i]);
		}

		return clust_cloud;
	}

	template <typename real_t>
	PointCloud3<real_t> generate_dtm(const PointCloud3<real_t>& point_cloud, const real_t resolution)
	{
		Params csf_params;
		csf_params.smooth_slope     = true;
		csf_params.cloth_resolution = resolution;
		csf_params.verbose          = true;

		CSF   csf;
		auto& csf_pc = csf.getPointCloud();
		csf_pc.clear();
		csf_pc.resize(point_cloud.rows());
		for (Eigen::Index point_id; point_id < point_cloud.rows(); ++point_id)
		{
			csf_pc[point_id] = {static_cast<double>(point_cloud(point_id, 0)), static_cast<double>(-point_cloud(point_id, 2)), static_cast<double>(point_cloud(point_id, 1))};
		}

		const auto          cloth     = csf.runClothSimulation();
		const auto&         particles = cloth.getParticles();
		PointCloud3<real_t> dtm(particles.size(), 3);
		for (size_t particle_id = 0; particle_id < particles.size(); ++particle_id)
		{
			const auto& particle = particles[particle_id];
			dtm(particle_id, 0)  = static_cast<real_t>(particle.initial_pos.f[0]);
			dtm(particle_id, 1)  = static_cast<real_t>(particle.initial_pos.f[2]);
			dtm(particle_id, 2)  = static_cast<real_t>(-particles[particle_id].height);
		}
		return dtm;
	}

	// TODO: prefer grid NN
	// TODO: prefer "pure" median filter
	template <typename real_t>
	PointCloud3<real_t> clean_cloth(PointCloud3<real_t>& cloth)
	{
		const size_t n_points = cloth.rows();
		if (n_points < 15)
			throw std::runtime_error("Input DTM too small (less than 15 points).");

		if (n_points == 15)
			std::cerr << "Warning: Input DTM has exactly 15 points, minimum accepted.\n";

		const size_t half_n_points = n_points / 2;
		const size_t n_neighbors   = 15;
		const real_t mad_factor    = 2.0;

		using kd_tree_t                = nanoflann::KDTreeEigenMatrixAdaptor<PointCloud2<real_t>, 2, nanoflann::metric_L2_Simple>;
		const PointCloud2<real_t> dtm2 = cloth.template leftCols<2>();
		kd_tree_t                 kd_tree(2, dtm2, 10, 0);
		kd_tree.buildIndex();

		std::vector<size_t> neighbors(n_neighbors);
		std::vector<real_t> dists(n_neighbors);
		std::vector<real_t> abs_devs(n_points);

		for (size_t i = 0; i < n_points; ++i)
		{
			nanoflann::KNNResultSet<real_t> result(n_neighbors);
			result.init(neighbors.data(), dists.data());
			kd_tree.findNeighbors(result, dtm2.row(i), nanoflann::SearchParameters(0, true)); // explicitly tell that we want sorted values

			// values are already sorted
			const real_t median_z = cloth(neighbors[half_n_points], 2);
			abs_devs[i]           = std::abs(cloth(i, 2) - median_z);
		}

		// Compute MAD (median of absolute deviations)
		std::vector<real_t> abs_devs_copy = abs_devs;
		std::nth_element(std::begin(abs_devs_copy), std::begin(abs_devs_copy) + half_n_points, std::end(abs_devs_copy));
		const real_t mad = abs_devs_copy[half_n_points];

		// Filter points
		// TODO parallelize, and do not allocate valid_indices?
		std::vector<Eigen::Index> valid_indices;
		valid_indices.reserve(n_points); // this should not be too far...
		for (Eigen::Index i = 0; i < n_points; ++i)
		{
			if (abs_devs[i] <= mad_factor * mad)
			{
				valid_indices.push_back(i);
			}
		}

		PointCloud3<real_t> clean_points(valid_indices.size(), 3);
		for (size_t i = 0; i < valid_indices.size(); ++i)
		{
			clean_points.row(i) = cloth.row(valid_indices[i]);
		}

		return clean_points;
	}

	// TODO share kdtree_index
	template <typename real_t>
	Eigen::VectorX<real_t> normalize_height(const PointCloud3<real_t>& point_cloud, const PointCloud3<real_t>& dtm)
	{

		const size_t n_points = dtm.rows();

		if (n_points < 3)
			throw std::runtime_error("Input DTM too small (less than 3 points).");

		const size_t n_neighbors = 3;

		Eigen::VectorX<real_t> normalized_heights(point_cloud.rows());
		using kd_tree_t                = nanoflann::KDTreeEigenMatrixAdaptor<PointCloud2<real_t>, 2, nanoflann::metric_L2_Simple>;
		const PointCloud2<real_t> dtm2 = dtm.template leftCols<2>();
		kd_tree_t                 kd_tree(2, dtm, 10, 0);
		kd_tree.buildIndex();

		std::vector<size_t> indices(n_neighbors);
		std::vector<double> dists(n_neighbors);

		for (size_t i = 0; i < n_neighbors; ++i)
		{
			const auto                      query_pt = point_cloud.row(i).template head<2>().data();
			nanoflann::KNNResultSet<real_t> result(n_neighbors);
			result.init(indices.data(), dists.data());
			kd_tree.findNeighbors(result, query_pt, nanoflann::SearchParameters());

			// Convert squared distances to actual distances
			std::vector<double> weights(n_neighbors);
			double              sum_weights = 0.0;
			for (size_t j = 0; j < n_neighbors; ++j)
			{
				weights[j] = std::sqrt(dists[j]) + 1e-8; // epsilon to avoid division by zero
				sum_weights += weights[j];
			}

			// Normalize weights
			for (double& w : weights)
				w /= sum_weights;

			// Compute weighted average Z from DTM
			double weighted_z = 0.0;
			for (size_t j = 0; j < n_neighbors; ++j)
			{
				weighted_z += weights[j] * dtm(indices[j], 2);
			}

			normalized_heights(i) = point_cloud(i, 2) - weighted_z;
		}

		return normalized_heights;
	}

} // namespace lib3dfin
