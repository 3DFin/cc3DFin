// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright 2023-2025 Carlos Cabo <carloscabo@uniovi.es>

#include "stripe.hpp"

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
	PointCloud3<real_t> filter_stripe(const RefPointCloud<real_t>& point_cloud, const Eigen::VectorX<real_t>& z0, real_t stripe_lower_limit, real_t stripe_upper_limit)
	{
		Eigen::Array<bool, Eigen::Dynamic, 1> height_mask = z0.array() > stripe_lower_limit && z0.array() < stripe_upper_limit;
		Eigen::Index                          mask_count  = height_mask.count();
		PointCloud3<real_t>                   stripe(mask_count, 3);

		Eigen::Index stripe_id = 0;
		for (Eigen::Index point_id = 0; point_id < point_cloud.rows(); ++point_id)
		{
			if (height_mask(point_id))
			{
				stripe.row(stripe_id++) = point_cloud.row(point_id);
			}
		}
		return stripe;
	}

	template <typename real_t>
	real_t adhoc_verticality(const PointCloud3<real_t>& cloud)
	{
		// Compute the (3, 3) covariance matrix
		const PointCloud3<real_t>         centered_cloud = cloud.rowwise() - cloud.colwise().mean();
		const Eigen::Matrix<real_t, 3, 3> cov            = (centered_cloud.transpose() * centered_cloud) / real_t(cloud.rows());

		// Compute the eigenvalues and eigenvectors of the covariance
		Eigen::SelfAdjointEigenSolver<Eigen::Matrix<real_t, 3, 3>> es(cov);

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
		real_t             sq_search_radius = scale * scale;

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
	PointCloud3<real_t> one_iter_vert_clustering(const PointCloud3<real_t>& stripe, real_t scale, real_t vert_threshold, uint32_t n_points, real_t resolution_xy, real_t resolution_z)
	{
		using namespace std::chrono;
		auto t_start = high_resolution_clock::now();
		std::cout << " -Computing verticality..." << std::endl;

		// Voxelate
		const auto [voxelated_stripe, cloud_to_vox] = voxelize(RefPointCloud<real_t>(stripe), resolution_xy, resolution_z, true);

		auto num_voxels = voxelated_stripe.rows();

		// Compute verticality feature
		Eigen::VectorX<real_t>                vert_values      = compute_verticality_feature(voxelated_stripe, scale);
		Eigen::Array<bool, Eigen::Dynamic, 1> valid_vox_mask   = vert_values.array() > vert_threshold;
		auto                                  num_valid_voxels = valid_vox_mask.count();

		if (!num_valid_voxels)
		{
			throw std::runtime_error("No vertical clusters found. Try to decrease threshold or voxel size.");
			// TODO catch this in the GUI
		}

		PointCloud3<real_t> vox_filtered_stripe(num_valid_voxels, 3);
		VecIndex<uint32_t>  vox_to_filtered_vox(num_voxels);
		vox_to_filtered_vox.setConstant(0); // invalid will remains 0

		Eigen::Index filtered_voxel_id = 0;
		for (Eigen::Index voxel_id = 0; voxel_id < voxelated_stripe.rows(); ++voxel_id)
		{
			if (valid_vox_mask(voxel_id))
			{
				vox_to_filtered_vox(voxel_id)                = filtered_voxel_id;
				vox_filtered_stripe.row(filtered_voxel_id++) = voxelated_stripe.row(voxel_id);
			}
		}

		std::cout << "filtered voxel number : " << num_valid_voxels << std::endl;

		auto t_mid = high_resolution_clock::now();
		std::cout << "   " << duration<double>(t_mid - t_start).count() << " s" << std::endl;

		std::cout << " -Clustering..." << std::endl;

		// TODO, this do not handle anisotropy in the voxelization...
		//  this already the case in the original implementation..
		real_t            eps            = resolution_xy * std::sqrt(3.0) + 1e-6;
		VecIndex<int32_t> cluster_labels = connected_components(RefPointCloud<real_t>(vox_filtered_stripe), eps, 2);

		// TODO factorise this with the ground.hpp equivalent
		//  Count clusters
		std::unordered_map<int32_t, uint32_t> label_counts;
		for (size_t filtered_voxel_id = 0; filtered_voxel_id < num_valid_voxels; ++filtered_voxel_id)
		{
			++label_counts[cluster_labels(filtered_voxel_id)];
		}

		if (label_counts.size() == 1 && label_counts.count(-1))
		{
			throw std::runtime_error("No valid clusters found.");
		}

		auto start_post = high_resolution_clock::now();
		std::cout << "   " << duration<double>(start_post - t_mid).count() << " s" << std::endl;
		std::cout << " -Extracting 'candidate' stems..." << std::endl;

		// Find large clusters
		std::set<uint32_t> large_clusters;
		for (const auto& [label, count] : label_counts)
		{
			if (label > -1 && count > n_points)
			{
				large_clusters.insert(label);
			}
		}

		if (large_clusters.empty())
		{
			throw std::runtime_error("Clusters found, but all are too small to be considered stems.");
		}

		// Filter cloud by valid clusters
		std::vector<Eigen::Index> valid_indices;
		// hint to avoid too small allocation
		// TODO: maybe prefer a mask (more efficient - less allocations - but uses more memory...)
		valid_indices.reserve(large_clusters.size());
		for (Eigen::Index point_id = 0; point_id < stripe.rows(); ++point_id)
		{
			auto voxel_id = cloud_to_vox(point_id);

			if (!valid_vox_mask(voxel_id))
				continue;

			auto filtered_voxel_id = vox_to_filtered_vox(voxel_id);
			auto cluster_id        = cluster_labels[filtered_voxel_id];
			if (cluster_id > -1 && large_clusters.count(cluster_id))
			{
				valid_indices.push_back(point_id);
			}
		}

		PointCloud3<real_t> clust_stripe_cloud(valid_indices.size(), 3);
		for (size_t i = 0; i < valid_indices.size(); ++i)
		{
			clust_stripe_cloud.row(i) = stripe.row(valid_indices[i]);
		}

		auto   t_end      = high_resolution_clock::now();
		double total_time = duration<double>(t_end - t_start).count();

		std::cout << "   " << large_clusters.size() << " clusters" << std::endl;
		std::cout << "   " << clust_stripe_cloud.rows() << " points" << std::endl;

		return clust_stripe_cloud;
	}

	template <typename real_t>
	void verticality_clustering(const PointCloud3<real_t>& stripe, real_t scale, real_t vert_threshold, uint32_t n_points, real_t resolution_xy, real_t resolution_z, uint32_t n_iter)
	{
		PointCloud3<real_t> ref_stripe = stripe; // TODO avoid copy
		for (uint32_t iter = 0; iter < n_iter; ++iter)
		{
			ref_stripe = one_iter_vert_clustering(ref_stripe, scale, vert_threshold, n_points, resolution_xy, resolution_z);
		}
	}

	template PointCloud3<float>  filter_stripe(const RefPointCloud<float>& point_cloud, const Eigen::VectorX<float>& z0, float stripe_lower_limit, float stripe_upper_limit);
	template PointCloud3<double> filter_stripe(const RefPointCloud<double>& point_cloud, const Eigen::VectorX<double>& z0, double stripe_lower_limit, double stripe_upper_limit);
	template void                verticality_clustering<float>(const PointCloud3<float>& stripe, float scale, float vert_threshold, uint32_t n_points, float resolution_xy, float resolution_z, uint32_t n_iter);
	template void                verticality_clustering<double>(const PointCloud3<double>& stripe, double scale, double vert_threshold, uint32_t n_points, double resolution_xy, double resolution_z, uint32_t n_iter);

} // namespace lib3dfin
