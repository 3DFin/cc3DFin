// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright 2023-2025 Carlos Cabo <carloscabo@uniovi.es>

#include "peeling.hpp"

#include "connected_components.hpp"
#include "types.hpp"
#include "voxel.hpp"

// nanoflann
#include <Eigen/src/Core/Matrix.h>
#include <Eigen/src/Core/util/Meta.h>
#include <nanoflann.hpp>

// taskflow
#include <taskflow/algorithm/for_each.hpp>
#include <taskflow/taskflow.hpp>

// std lib
#include <cstdint>
#include <iostream>

namespace lib3dfin
{

	template <typename real_t>
	TreePeeler<real_t>::TreePeeler(
	    const RefPointCloud<real_t>&  point_cloud,
	    const Eigen::VectorX<real_t>& z0_in,
	    TreePeeler::Parameters        params_in)
	    : point_cloud_(point_cloud)
	    , num_points_(point_cloud.rows())
	    , z0(z0_in)
	    , params_(std::move(params_in))
	{
	}

	template <typename real_t>
	PointCloud3<real_t> TreePeeler<real_t>::peel()
	{
		std::cout << "[TreePeeler] Starting peeling process..." << std::endl;
		// reset total time
		total_time_ = 0.0;

		// Get the Initial stripe
		auto stripe_indicator = filterStripe();

		// Perform verticality clustering
		for (uint32_t iter = 0; iter < params_.num_iterations; ++iter)
		{
			stripe_indicator = verticalityClustering(stripe_indicator);
		}
		std::cout << "[TreePeeler] total time: " << total_time_ << std::endl;

		// filter stripe by cluster indicator
		auto stripe_cloud = extractStripe(stripe_indicator);

		return stripe_cloud;
	}

	template <typename real_t>
	ArrayClusterIndicator TreePeeler<real_t>::filterStripe()
	{
		ArrayClusterIndicator stripe_cluster_indicator(num_points_);
		stripe_cluster_indicator.setConstant(NO_CLUSTER_ID);
		stripe_cluster_indicator = (z0.array() > params_.stripe_lower_limit && z0.array() < params_.stripe_upper_limit).select(0, stripe_cluster_indicator);
		return stripe_cluster_indicator;
	}

	template <typename real_t>
	real_t adhoc_verticality(const PointCloud3<real_t>& cloud)
	{
		// Compute the (3, 3) covariance matrix
		const PointCloud3<real_t>    centered_cloud = cloud.rowwise() - cloud.colwise().mean();
		const Eigen::Matrix3<real_t> cov            = (centered_cloud.transpose() * centered_cloud) / real_t(cloud.rows());

		// Compute the eigenvalues and eigenvectors of the covariance
		Eigen::SelfAdjointEigenSolver<Eigen::Matrix3<real_t>> es(cov);

		// eigenvalues are sorted by increasing order so
		// first eigen vector is the normal vector. its third component is the z component
		real_t normal_z_component = es.eigenvectors().col(0)(2);
		return real_t(1.0) - std::abs(normal_z_component);
	}

	template <typename real_t>
	Eigen::VectorX<real_t> compute_verticality_feature(const PointCloud3<real_t>& stripe, real_t scale)
	{
		using kd_tree_t            = nanoflann::KDTreeEigenMatrixAdaptor<const PointCloud3<real_t>, 3, nanoflann::metric_L2_Simple>;
		const size_t       max_knn = 50000;
		kd_tree_t          kd_tree(3, stripe, 10, 0);
		const Eigen::Index n_points         = stripe.rows();
		const real_t       sq_search_radius = scale * scale;

		Eigen::VectorX<real_t> verticality(n_points);

		tf::Executor executor;
		tf::Taskflow taskflow;

		taskflow.for_each_index(
		    Eigen::Index(0), n_points, Eigen::Index(1), [&](Eigen::Index point_id)
		    {
            std::vector<nanoflann::ResultItem<Eigen::Index, real_t>> result_set;

            nanoflann::RadiusResultSet<real_t, Eigen::Index> radius_result_set(sq_search_radius, result_set);
            const auto                                       num_found =
                kd_tree.index_->radiusSearchCustomCallback(stripe.row(point_id).data(), radius_result_set);

            // not enough point, no feature computation
            if (num_found < 2) return;

            // partial sort for max_knn
            if (num_found > max_knn)
            {
                std::partial_sort(
                    result_set.begin(), result_set.begin() + max_knn, result_set.end(), nanoflann::IndexDist_Sorter());
            }

            const size_t num_nn = std::min(num_found, max_knn);

            PointCloud3<real_t> cloud(num_nn, 3);
            for (size_t id = 0; id < num_nn; ++id) { cloud.row(id) = stripe.row(result_set[id].first); }
            verticality(point_id) = adhoc_verticality(cloud); });
		executor.run(taskflow).get();
		return verticality;
	}

	template <typename real_t>
	PointCloud3<real_t> TreePeeler<real_t>::extractStripe(const ArrayClusterIndicator& stripe_indicator)
	{
		const auto          num_points_stripe = (stripe_indicator != NO_CLUSTER_ID).count();
		PointCloud3<real_t> stripe_cloud(num_points_stripe, 3);

		Eigen::Index stripe_id = 0;
		for (Eigen::Index point_id = 0; point_id < num_points_; ++point_id)
		{
			if (stripe_indicator(point_id) != NO_CLUSTER_ID)
			{
				stripe_cloud.row(stripe_id) = point_cloud_.row(point_id);
				stripe_id++;
			}
		}
		return stripe_cloud;
	}

	template <typename real_t>
	ArrayClusterIndicator TreePeeler<real_t>::verticalityClustering(const ArrayClusterIndicator& stripe_indicator)
	{
		auto t_start = std::chrono::high_resolution_clock::now();
		std::cout << " -Computing verticality..." << std::endl;

		// filter stripe by cluster indicator
		const auto stripe_cloud     = extractStripe(stripe_indicator);
		const auto num_point_stripe = stripe_cloud.rows();

		// Voxelate stripe cloud
		const auto [voxelated_stripe, cloud_to_vox] = voxelize(RefPointCloud<real_t>(stripe_cloud), params_.resolution_xy, params_.resolution_z, true);
		const auto num_voxels                       = voxelated_stripe.rows();

		// Compute verticality feature
		Eigen::VectorX<real_t> vert_values      = compute_verticality_feature(voxelated_stripe, params_.verticality_nn_scale);
		ArrayMask              valid_vox_mask   = vert_values.array() > params_.verticality_threshold;
		auto                   num_valid_voxels = valid_vox_mask.count();

		if (!num_valid_voxels)
		{
			throw std::runtime_error("No vertical clusters found. Try to decrease threshold or voxel size.");
			// TODO catch this in the GUI
		}

		PointCloud3<real_t> vox_filtered_stripe(num_valid_voxels, 3);
		VecIndex<uint32_t>  vox_to_filtered_vox(num_voxels);
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

		std::cout << "number of filtered voxels: " << num_valid_voxels << std::endl;

		auto t_mid = std::chrono::high_resolution_clock::now();
		std::cout << "   " << std::chrono::duration<double>(t_mid - t_start).count() << " s" << std::endl;

		std::cout << " -Clustering..." << std::endl;

		// TODO : this does not handle anisotropy in the voxelization...
		// this is already the case in the original implementation...
		const real_t      eps            = params_.resolution_xy * std::sqrt(3.0) + 1e-6;
		VecIndex<int32_t> cluster_labels = connected_components(RefPointCloud<real_t>(vox_filtered_stripe), eps, 2);

		// Count clusters
		std::unordered_map<int32_t, uint32_t> label_counts;
		for (size_t filtered_voxel_id = 0; filtered_voxel_id < num_valid_voxels; ++filtered_voxel_id)
		{
			++label_counts[cluster_labels(filtered_voxel_id)];
		}

		if (label_counts.size() == 1 && label_counts.count(NO_CLUSTER_ID))
		{
			throw std::runtime_error("No valid clusters found.");
		}

		auto start_post = std::chrono::high_resolution_clock::now();
		std::cout << "   " << std::chrono::duration<double>(start_post - t_mid).count() << " s" << std::endl;
		std::cout << " -Extracting 'candidate' stems..." << std::endl;

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

			auto voxel_id = cloud_to_vox(stripe_id++);
			if (!valid_vox_mask(voxel_id))
				continue;

			auto filtered_voxel_id = vox_to_filtered_vox(voxel_id);
			auto cluster_id        = cluster_labels[filtered_voxel_id];

			if (large_clusters.count(cluster_id))
				new_stripe_indicator(base_id) = cluster_id;
		}

		auto   t_end          = std::chrono::high_resolution_clock::now();
		double iteration_time = std::chrono::duration<double>(t_end - t_start).count();

		std::cout << "   " << large_clusters.size() << " clusters" << std::endl;
		std::cout << "   iteration took " << iteration_time << std::endl;
		total_time_ += iteration_time;
		return new_stripe_indicator;
	}

	template class TreePeeler<float>;
	template class TreePeeler<double>;

} // namespace lib3dfin
