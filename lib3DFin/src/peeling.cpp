// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright 2023-2026 Carlos Cabo <carloscabo@uniovi.es>

#include "peeling.hpp"

#include "connected_components.hpp"
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

	inline double adhoc_verticality(const PointCloud3& cloud)
	{
		// Compute the (3, 3) covariance matrix
		const PointCloud3     centered_cloud = cloud.rowwise() - cloud.colwise().mean();
		const Eigen::Matrix3d cov            = (centered_cloud.transpose() * centered_cloud) / cloud.rows();

		// Compute the eigenvalues and eigenvectors of the covariance
		Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es(cov);

		// eigenvalues are sorted by increasing order so
		// first eigen vector is the normal vector. Its third component is the z component.
		const double normal_z_component = es.eigenvectors()(2, 0);
		return 1.0 - std::abs(normal_z_component);
	}

	Eigen::VectorXd compute_verticality_feature(const PointCloud3& stripe, double scale, tf::Executor& executor)
	{
		using kd_tree_t            = nanoflann::KDTreeEigenMatrixAdaptor<const PointCloud3, 3, nanoflann::metric_L2_Simple>;
		const size_t       max_knn = 50000;
		kd_tree_t          kd_tree(3, stripe, 10, 0);
		const Eigen::Index n_points         = stripe.rows();
		const double       sq_search_radius = scale * scale;

		Eigen::VectorXd verticality(n_points);
		verticality.setZero();

		tf::Taskflow taskflow;

		taskflow.for_each_index(
		    Eigen::Index(0), n_points, Eigen::Index(1), [&](Eigen::Index point_id)
		    {
              std::vector<nanoflann::ResultItem<Eigen::Index, double>> result_set;

              nanoflann::RadiusResultSet<double, Eigen::Index> radius_result_set(sq_search_radius, result_set);
              const auto                                       num_found =
                  kd_tree.index_->radiusSearchCustomCallback(stripe.row(point_id).data(), radius_result_set);

              // not enough point, no feature computation
              if (num_found < 3) return;

              // partial sort for max_knn
              if (num_found > max_knn)
              {
                  std::partial_sort(
                      std::begin(result_set), std::begin(result_set) + max_knn, std::end(result_set), nanoflann::IndexDist_Sorter());
              }

              const size_t num_nn = std::min(num_found, max_knn);

              PointCloud3 cloud(num_nn, 3);
              for (size_t id = 0; id < num_nn; ++id) { cloud.row(static_cast<Eigen::Index>(id)) = stripe.row(result_set[id].first); }
              verticality(point_id) = adhoc_verticality(cloud); });
		executor.run(taskflow).get();

		return verticality;
	}

	TreePeeler::TreePeeler(
	    const PointCloud3&               point_cloud,
	    const Eigen::VectorXd&           z0,
	    std::unique_ptr<FilterPredicate> initial_state,
	    const StripePeelingParams&       params,
	    tf::Executor&                    executor)
	    : point_cloud_(point_cloud)
	    , z0_(z0)
	    , num_points_(point_cloud.rows())
	    , params_(params)
	    , executor_(executor)
	{
		if (initial_state == nullptr)
		{
			initial_state_.reset(new FilterPredicate);
		}
		else
		{
			initial_state_ = std::move(initial_state);
		}
	}

	ArrayClusterIndicator TreePeeler::peel()
	{
		spdlog::info("[TreePeeler] Starting peeling process...");
		// reset total time
		total_time_ = 0.0;

		ArrayClusterIndicator stripe_indicator = initial_state_->filter(point_cloud_, z0_);

		// Perform verticality clustering
		for (uint32_t iter = 0; iter < params_.num_iterations; ++iter)
		{
			stripe_indicator = verticalityClustering(stripe_indicator);
		}

		spdlog::info("[TreePeeler] Total time: {0:.2f} s", total_time_);
		return stripe_indicator;
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
		const auto [voxelated_stripe, stripe_cloud_to_vox] = voxelize(stripe_cloud, params_.resolution_xy, params_.resolution_z, executor_, true);
		const auto num_voxels                              = voxelated_stripe.rows();

		// Compute verticality feature
		const Eigen::VectorXd vert_values      = compute_verticality_feature(voxelated_stripe, params_.verticality_nn_scale, executor_);
		const ArrayMask       valid_vox_mask   = vert_values.array() > params_.verticality_threshold;
		auto                  num_valid_voxels = valid_vox_mask.count();

		if (!num_valid_voxels)
		{
			throw std::runtime_error("No vertical clusters found. Try to decrease threshold or voxel size.");
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

		spdlog::info("[Peeling] Clustering...");

		// TODO(RJ): this does not handle anisotropy in the voxelization...
		// this is already the case in the original implementation...
		const double eps            = params_.resolution_xy * std::sqrt(3.0) + 1e-6;
		const auto   cluster_labels = connected_components(vox_filtered_stripe, eps, 2, executor_);

		// Count clusters
		std::unordered_map<int32_t, uint32_t> label_counts;
		for (Eigen::Index filtered_voxel_id = 0; filtered_voxel_id < num_valid_voxels; ++filtered_voxel_id)
		{
			++label_counts[cluster_labels[filtered_voxel_id]];
		}

		if (label_counts.size() == 1 && label_counts.count(NO_CLUSTER_ID))
		{
			throw std::runtime_error("No valid clusters found.");
		}

		auto start_post = std::chrono::high_resolution_clock::now();
		spdlog::info("[Peeling] Clustering done in {0:.2f} s", std::chrono::duration<double>(start_post - t_mid).count());
		spdlog::info("[Peeling] Extracting 'candidate' stems...");

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
			auto cluster_id        = cluster_labels[filtered_voxel_id];

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
