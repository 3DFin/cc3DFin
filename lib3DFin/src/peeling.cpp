// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright 2023-2025 Carlos Cabo <carloscabo@uniovi.es>

#include "peeling.hpp"

#include "connected_components.hpp"
#include "verticality.hpp"
#include "voxel.hpp"

// spdlog
#include <spdlog/spdlog.h>

// nanoflann
#include <nanoflann.hpp>

// taskflow
#include <taskflow/algorithm/for_each.hpp>
#include <taskflow/taskflow.hpp>

// stdlib
#include <cstdint>

namespace lib3dfin
{

	TreePeeler::TreePeeler(
	    const PointCloud3&     point_cloud,
	    TreePeeler::Parameters params)
	    : point_cloud_(point_cloud)
	    , num_points_(point_cloud.rows())
	    , params_(std::move(params))
	{
	}

	void TreePeeler::peel(ArrayClusterIndicator& stripe_indicator)
	{
		spdlog::info("[TreePeeler] Starting peeling process...");
		// reset total time
		total_time_ = 0.0;

		// Perform verticality clustering
		for (uint32_t iter = 0; iter < params_.num_iterations; ++iter)
		{
			stripe_indicator = verticalityClustering(stripe_indicator);
		}

		spdlog::info("[TreePeeler] Total time: {0:.2f}", total_time_);
	}

	ArrayClusterIndicator TreePeeler::filterInitialStripe(const Eigen::VectorXd& z0, double stripe_lower_limit, double stripe_upper_limit)
	{
		ArrayClusterIndicator stripe_cluster_indicator(z0.size());
		stripe_cluster_indicator.setConstant(NO_CLUSTER_ID);
		stripe_cluster_indicator = (z0.array() > stripe_lower_limit && z0.array() < stripe_upper_limit).select(0, stripe_cluster_indicator);
		return stripe_cluster_indicator;
	}

	ArrayClusterIndicator TreePeeler::filterInitialStripe(const Eigen::VectorXd& z0, const Eigen::VectorXd& axis_distance, double max_distance, double stripe_lower_limit, double stripe_upper_limit)
	{
		assert(z0.size() == axis_distance.size());
		ArrayClusterIndicator stripe_cluster_indicator(z0.size());
		for (Eigen::Index point_id = 0; point_id < z0.size(); ++point_id)
		{
			if (z0(point_id) > stripe_lower_limit && z0(point_id) < stripe_upper_limit && axis_distance(point_id) < max_distance)
			{
				stripe_cluster_indicator(point_id) = 0;
			}
			else
			{
				stripe_cluster_indicator(point_id) = NO_CLUSTER_ID;
			}
		}
		return stripe_cluster_indicator;
	}

	PointCloud3 TreePeeler::extractStripe(const PointCloud3& point_cloud, const ArrayClusterIndicator& stripe_indicator)
	{
		const auto  num_points_stripe = (stripe_indicator != NO_CLUSTER_ID).count();
		PointCloud3 stripe_cloud(num_points_stripe, 3);

		Eigen::Index stripe_id = 0;
		for (Eigen::Index point_id = 0; point_id < point_cloud.rows(); ++point_id)
		{
			if (stripe_indicator(point_id) != NO_CLUSTER_ID)
			{
				stripe_cloud.row(stripe_id) = point_cloud.row(point_id);
				stripe_id++;
			}
		}
		return stripe_cloud;
	}

	ArrayClusterIndicator TreePeeler::verticalityClustering(const ArrayClusterIndicator& stripe_indicator)
	{
		auto t_start = std::chrono::high_resolution_clock::now();
		spdlog::info("[Peeling] Computing verticality...");

		// filter stripe by cluster indicator
		const auto stripe_cloud     = extractStripe(point_cloud_, stripe_indicator);
		const auto num_point_stripe = stripe_cloud.rows();

		// Voxelate stripe cloud
		const auto [voxelated_stripe, stripe_cloud_to_vox] = voxelize(PointCloud3(stripe_cloud), params_.resolution_xy, params_.resolution_z, true);
		const auto num_voxels                              = voxelated_stripe.rows();

		// Compute verticality feature
		Eigen::VectorXd vert_values      = compute_verticality_feature(voxelated_stripe, params_.verticality_nn_scale);
		ArrayMask       valid_vox_mask   = vert_values.array() > params_.verticality_threshold;
		auto            num_valid_voxels = valid_vox_mask.count();

		if (!num_valid_voxels)
		{
			throw std::runtime_error("No vertical clusters found. Try to decrease threshold or voxel size.");
			// TODO catch this in the GUI
		}

		PointCloud3        vox_filtered_stripe(num_valid_voxels, 3);
		VecIndex<uint32_t> vox_to_filtered_vox(num_voxels);
		vox_to_filtered_vox.setConstant(0); // beware invalid vox will remains 0, we have to check against valid_vox_mask to desanbiguate between id == 0 and 0 == INVALID

		Eigen::Index filtered_voxel_id = 0;
		for (Eigen::Index voxel_id = 0; voxel_id < voxelated_stripe.rows(); ++voxel_id)
		{
			if (valid_vox_mask(voxel_id))
			{
				vox_to_filtered_vox(voxel_id)                = filtered_voxel_id;
				vox_filtered_stripe.row(filtered_voxel_id++) = voxelated_stripe.row(voxel_id);
			}
		}

		spdlog::info("[Peeling] Number of valid voxels (pass verticality test): {}", num_valid_voxels);

		auto t_mid = std::chrono::high_resolution_clock::now();
		spdlog::info("[Peeling] Verticality done in {0:.2f} s", std::chrono::duration<double>(t_mid - t_start).count());

		spdlog::info("[Peeling] -Clustering...");

		// TODO : this does not handle anisotropy in the voxelization...
		// this is already the case in the original implementation...
		const double      eps            = params_.resolution_xy * std::sqrt(3.0) + 1e-6;
		VecIndex<int32_t> cluster_labels = connected_components(vox_filtered_stripe, eps, 2);

		// Count clusters
		std::unordered_map<int32_t, uint32_t> label_counts;
		for (Eigen::Index filtered_voxel_id = 0; filtered_voxel_id < num_valid_voxels; ++filtered_voxel_id)
		{
			++label_counts[cluster_labels(filtered_voxel_id)];
		}

		if (label_counts.size() == 1 && label_counts.count(NO_CLUSTER_ID))
		{
			throw std::runtime_error("No valid clusters found.");
		}

		auto start_post = std::chrono::high_resolution_clock::now();
		spdlog::info("[Peeling] Clutesring done in {0:.2f} s", std::chrono::duration<double>(start_post - t_mid).count());
		spdlog::info("[Peeling] -Extracting 'candidate' stems...");

		// Find large clusters
		std::set<uint32_t> large_clusters;
		for (const auto& [label, count] : label_counts)
		{
			if (label != NO_CLUSTER_ID && count > params_.num_voxels_threshold)
			{
				large_clusters.insert(label);
			}
		}

		if (large_clusters.empty())
		{
			throw std::runtime_error("Clusters found, but all are too small to be considered stems.");
			// TODO catch this in the GUI
		}

		// Create a new_cluster_indicator and extract it.
		ArrayClusterIndicator new_stripe_indicator(num_points_);
		new_stripe_indicator.setConstant(NO_CLUSTER_ID);
		Eigen::Index stripe_id = 0;
		for (Eigen::Index base_id = 0; base_id < num_points_; ++base_id)
		{
			if (stripe_indicator(base_id) == NO_CLUSTER_ID)
				continue;

			auto voxel_id = stripe_cloud_to_vox(stripe_id++);
			if (!valid_vox_mask(voxel_id))
				continue;

			auto filtered_voxel_id = vox_to_filtered_vox(voxel_id);
			auto cluster_id        = cluster_labels(filtered_voxel_id);

			if (large_clusters.count(cluster_id))
				new_stripe_indicator(base_id) = cluster_id;
		}

		auto   t_end          = std::chrono::high_resolution_clock::now();
		double iteration_time = std::chrono::duration<double>(t_end - t_start).count();

		spdlog::info("[Peeling] {} clusters", large_clusters.size());
		spdlog::info("[Peeling] Full iteration took {0:.2f} s", iteration_time);
		total_time_ += iteration_time;
		return new_stripe_indicator;
	}

} // namespace lib3dfin
