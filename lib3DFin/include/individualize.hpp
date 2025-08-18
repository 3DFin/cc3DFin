#pragma once

// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright 2023-2025 Carlos Cabo <carloscabo@uniovi.es>

#include "types.hpp"
#include "voxel.hpp"

// nanoflann
#include <nanoflann.hpp>

// taskflow
#include <taskflow/algorithm/for_each.hpp>
#include <taskflow/taskflow.hpp>

// stdlib
#include <cstdint>
#include <vector>

namespace lib3dfin
{

	template <typename real_t>
	struct TreeDescriptor
	{
		size_t       tree_id = 0;
		Vec3<real_t> pca; // most significant eigen vector?
		Vec3<real_t> centroid_coordinates;
		real_t       height_difference; // z - z0
		real_t       vertical_deviation;
	};

	template <typename real_t>
	struct AxesData
	{
		std::vector<TreeDescriptor<real_t>> tree_descriptors;
		Eigen::VectorX<real_t>              axis_distance;
		ArrayClusterIndicator               axis_cluster_indicator;
	};

	template <typename real_t>
	std::optional<Vec3<real_t>> vector_plane_intersection(const Vec3<real_t>& axis_pos, const Vec3<real_t>& axis_dir, const Eigen::Hyperplane<real_t, 3>& plane)
	{
		const real_t denom = plane.normal().dot(axis_dir);
		if (std::abs(denom) < 1e-8)
		{
			return std::nullopt;
		}
		const real_t t = -(plane.normal().dot(axis_pos) + plane.offset()) / denom;
		return axis_pos + t * axis_dir;
	}

	template <typename real_t>
	std::optional<std::pair<Vec3<real_t>, Vec3<real_t>>> axis_bb_intersection(const Vec3<real_t>& axis_pos, const Vec3<real_t>& axis_dir, const Vec3<real_t>& bottom_pos, const Vec3<real_t> top_pos)
	{
		Eigen::Hyperplane<real_t, 3> bottom_plane(Vec3<real_t>(0, 0, 1), bottom_pos);
		const auto                   bottom_inter = vector_plane_intersection(axis_pos, axis_dir, bottom_plane);
		if (bottom_inter == std::nullopt)
			return std::nullopt;

		Eigen::Hyperplane<real_t, 3> top_plane(Vec3<real_t>(0, 0, -1), top_pos);
		const auto                   top_inter = vector_plane_intersection(axis_pos, axis_dir, top_plane);
		if (top_inter == std::nullopt)
			return std::nullopt;
		return std::make_pair(bottom_inter.value(), top_inter.value());
	}

	template <typename real_t>
	AxesData<real_t> compute_axes_approximate(
	    const RefPointCloud<real_t>&  point_cloud,
	    const PointCloud3<real_t>&    voxelated_cloud,
	    const real_t                  voxel_resolution, // voxelization descriptor
	    const ArrayClusterIndicator&  clust_stripe_indicator,
	    const real_t                  stripe_lower_limit, // stripe descriptor
	    const real_t                  stripe_upper_limit, // stripe descriptor
	    const Eigen::VectorX<real_t>& z0,
	    // parameters
	    real_t   h_range,
	    uint32_t min_points,
	    real_t   d_max)
	{

		// TODO chrono and progress bar...
		// Space between samples along the axes
		const real_t SAMPLE_STEP = voxel_resolution;
		const auto   num_voxels  = voxelated_cloud.rows();

		// unique and count
		std::unordered_map<int32_t, uint32_t> counts;
		for (Eigen::Index point_id = 0; point_id < clust_stripe_indicator.size(); ++point_id)
		{
			const auto cluster_id = clust_stripe_indicator(point_id);
			if (cluster_id != NO_CLUSTER_ID)
			{
				counts[cluster_id]++;
			}
		}

		// filter clusters whithout enough points
		// TODO: it's in the original algorithm but it seems to be a weaker filter
		// than the minimal number of voxels in the peeling process
		std::vector<int32_t> valid_cluster_ids;
		valid_cluster_ids.reserve(counts.size());
		for (const auto& [cluster_id, count] : counts)
		{
			if (count > min_points)
			{
				valid_cluster_ids.push_back(cluster_id);
			}
		}

		// initialize result set
		AxesData<real_t> result;
		result.tree_descriptors.reserve(valid_cluster_ids.size());
		result.axis_cluster_indicator = ArrayClusterIndicator(num_voxels);
		result.axis_distance          = Eigen::VectorX<real_t>(num_voxels);

		// Height range (actual value, not the %) that points should extend throughout
		const real_t h_range_value = (stripe_upper_limit - stripe_lower_limit) * h_range;

		// Compute bounding box of the voxelated PC
		// TODO AdHoc bb?
		const Vec3<real_t> bb_min = voxelated_cloud.colwise().minCoeff().transpose();
		const Vec3<real_t> bb_max = voxelated_cloud.colwise().maxCoeff().transpose();

		// TODO taskflow // when validated
		std::vector<PointCloud3<real_t>> vec_axis_point_clouds;

		Eigen::Index total_axis_point = 0;

		for (const auto stem_id : valid_cluster_ids)
		{
			const auto          num_points      = clust_stripe_indicator.size();
			const auto          stem_num_points = counts[stem_id];
			PointCloud3<real_t> stem_cloud(stem_num_points, 3);
			// accumulate with max precision
			double z0_accumulator = 0.0;
			double z_accumulator  = 0.0;

			Eigen::Index stem_point_id = 0;
			for (Eigen::Index point_id = 0; point_id < num_points; ++point_id)
			{
				if (clust_stripe_indicator(point_id) == stem_id)
				{
					const Vec3<real_t>& stem_point = point_cloud.row(point_id);
					z_accumulator += static_cast<double>(stem_point(2));
					z0_accumulator += static_cast<double>(z0(point_id));
					stem_cloud.row(stem_point_id) = stem_point;
					stem_point_id++;
				}
			}
			// get min diff in scalar type unused in 3DFin
			const real_t diff_z_z0 = static_cast<real_t>((z_accumulator / stem_num_points) - (z0_accumulator / stem_num_points));

			// TODO not sure this check should be done on Z values... maybe it's better on Z0.
			// TODO could be more efficiently computed  in the loop above ?
			const auto peak_to_peak = stem_cloud.col(2).maxCoeff() - stem_cloud.col(2).minCoeff();
			if (peak_to_peak > h_range_value)
			{
				// TODO add a PCA helper somewhere to factorize PCA computation
				//  compute the PCA.
				//  Compute the (3, 3) covariance matrix
				const PointCloud3<real_t>    centered_cloud = stem_cloud.rowwise() - stem_cloud.colwise().mean();
				const Eigen::Matrix3<real_t> cov            = (centered_cloud.transpose() * centered_cloud) / real_t(stem_cloud.rows());

				// Compute the eigenvalues and eigenvectors of the covariance
				Eigen::SelfAdjointEigenSolver<Eigen::Matrix3<real_t>> es(cov);

				// Eigen values are sorted in increasing order, we looks for the more significant (components / axis)
				const Vec3<real_t> principal_axis = es.eigenvectors().col(2);

				// TODO could be more efficiently computed in the loop above
				const Vec3<real_t> centroid = stem_cloud.colwise().mean();

				const auto maybe_range = axis_bb_intersection(centroid, principal_axis, bb_min, bb_max);

				// TODO numbering could be non contiguous.
				if (!maybe_range)
				{
					std::cout << "invalid axis, tree skipped" << std::endl;
					continue;
				}

				std::cout << centroid(0) << " " << centroid(1) << " " << centroid(2) << std::endl;
				std::cout << principal_axis(0) << " " << principal_axis(1) << " " << principal_axis(2) << std::endl;

				const auto bottom_point   = maybe_range.value().first;
				const auto top_point      = maybe_range.value().second;
				const auto range_distance = (top_point - bottom_point).norm();

				const auto          num_sample = static_cast<size_t>(std::ceil(range_distance / SAMPLE_STEP));
				PointCloud3<real_t> axis_point_cloud(num_sample, 3);
				// get the upward pointing vector
				const auto axis_sample_axis = principal_axis(2) < 0 ? -principal_axis : principal_axis;
				for (Eigen::Index point_id = 0; point_id < num_sample; ++point_id)
				{
					axis_point_cloud.row(point_id) = bottom_point + axis_sample_axis * (static_cast<real_t>(point_id) * SAMPLE_STEP);
				}
				vec_axis_point_clouds.push_back(std::move(axis_point_cloud));
				total_axis_point += num_sample;
			}
		}

		// Concat axis cloud
		ArrayClusterIndicator axis_indicator(total_axis_point);
		PointCloud3<real_t>   concat_axis_point_cloud(total_axis_point, 3);
		Eigen::Index          padding = 0;
		Eigen::Index          axis_id = 0;
		for (const auto& axis_pointcloud : vec_axis_point_clouds)
		{
			const auto axis_num_points                                    = axis_pointcloud.rows();
			concat_axis_point_cloud.block(padding, 0, axis_num_points, 3) = std::move(axis_pointcloud);
			axis_indicator.segment(padding, axis_num_points).setConstant(axis_id);
			padding += axis_pointcloud.rows();
			++axis_id;
		}
		vec_axis_point_clouds.clear();
		vec_axis_point_clouds.shrink_to_fit();

		// KD-tree search
		using kd_tree_t = nanoflann::KDTreeEigenMatrixAdaptor<const PointCloud3<real_t>, 3, nanoflann::metric_L2_Simple>;
		kd_tree_t kd_tree(3, concat_axis_point_cloud, 10, 0);

		tf::Executor executor;
		tf::Taskflow taskflow;
		// for point in voxelated-cloud, query
		taskflow.for_each_index(
		    Eigen::Index(0), num_voxels, Eigen::Index(1), [&](Eigen::Index point_id)
		    {
			    Eigen::Index                                            index;
			    real_t * distance = result.axis_distance.data() + point_id;

				kd_tree.index_->knnSearch(voxelated_cloud.row(point_id).data(), 1, &index, distance);

				if(*distance > d_max)
				{
				    result.axis_cluster_indicator(point_id) = NO_CLUSTER_ID;
					*distance = d_max;
				} else {
				    result.axis_cluster_indicator(point_id) = index;
				} });
		executor.run(taskflow).get();

		return result;
	}

	template <typename real_t>
	void individualize_trees(
	    const RefPointCloud<real_t>&  point_cloud,
	    const ArrayClusterIndicator&  clust_stripe_indicator,
	    const real_t                  stripe_lower_limit, // stripe descriptor
	    const real_t                  stripe_upper_limit, // stripe descriptor
	    const Eigen::VectorX<real_t>& z0,
	    // params
	    const real_t   resolution_z,
	    const real_t   resolution_xy,
	    const real_t   h_range,
	    const real_t   d_max,
	    const uint32_t min_points,
	    const real_t   d,
	    const real_t   max_dev,
	    const real_t   resolution_heights)
	{

		const auto [voxelated_cloud, cloud_to_vox] = voxelize(point_cloud, resolution_xy, resolution_z, true);
		auto                          t0           = std::chrono::high_resolution_clock::now();
		const auto                    axes         = compute_axes_approximate(point_cloud, voxelated_cloud, resolution_xy, clust_stripe_indicator, stripe_lower_limit, stripe_upper_limit, z0, h_range, min_points, d_max);
		auto                          t1           = std::chrono::high_resolution_clock::now();
		std::chrono::duration<double> elapsed      = t1 - t0;
		std::cout << "compute_axes_approximate: "
		          << elapsed.count() << " seconds\n";
	}

} // namespace lib3dfin
