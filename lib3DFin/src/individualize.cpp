// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright 2023-2025 Carlos Cabo <carloscabo@uniovi.es>

// local
#include "individualize.hpp"

#include "connected_components.hpp"
#include "voxel.hpp"

// nanoflann
#include <nanoflann.hpp>

// taskflow
#include <taskflow/algorithm/for_each.hpp>
#include <taskflow/taskflow.hpp>

namespace lib3dfin
{

	TreeData TreeIndividualizer::individualize()
	{
		const auto [voxelated_cloud, cloud_to_vox]        = voxelize(point_cloud_, params_.resolution_xy, params_.resolution_z, true);
		auto                          t0                  = std::chrono::high_resolution_clock::now();
		auto                          voxelated_axes_data = computeAxesApproximate(voxelated_cloud);
		auto                          t1                  = std::chrono::high_resolution_clock::now();
		std::chrono::duration<double> elapsed             = t1 - t0;

		std::cout << "[Individualize] compute_axes_approximate: "
		          << elapsed.count() << " seconds\n";
		t0 = std::chrono::high_resolution_clock::now();
		compute_heights(voxelated_cloud, voxelated_axes_data);
		t1      = std::chrono::high_resolution_clock::now();
		elapsed = t1 - t0;
		std::cout << "[Individualize] compute_height: "
		          << elapsed.count() << " seconds\n";

		TreeData axes_data;
		axes_data.tree_descriptors = std::move(voxelated_axes_data.tree_descriptors);
		axes_data.cluster_indicator.resize(point_cloud_.rows());
		axes_data.axis_distance.resize(point_cloud_.rows());
		for (Eigen::Index point_id = 0; point_id < point_cloud_.rows(); ++point_id)
		{
			axes_data.cluster_indicator(point_id) = voxelated_axes_data.cluster_indicator(cloud_to_vox(point_id));
			axes_data.axis_distance(point_id)     = voxelated_axes_data.axis_distance(cloud_to_vox(point_id));
		}
		return axes_data;
	}

	TreeData TreeIndividualizer::computeAxesApproximate(
	    const PointCloud3& voxelated_cloud)
	{
		// TODO chrono and progress bar...
		// Space between samples along the axes
		const double sample_step = params_.resolution_xy;
		const auto   num_voxels  = voxelated_cloud.rows();
		const auto   num_points  = point_cloud_.rows();

		// unique and count
		std::unordered_map<int32_t, uint32_t> counts;
		for (Eigen::Index point_id = 0; point_id < num_points; ++point_id)
		{
			const auto cluster_id = stripe_.cluster_indicator(point_id);
			if (cluster_id != NO_CLUSTER_ID)
			{
				counts[cluster_id]++;
			}
		}

		// Filter clusters without enough points
		// TODO: This filter exists in the original algorithm but appears to weak because it is
		// less restrictive than the minimum voxel threshold used in the peeling process
		std::vector<int32_t> valid_cluster_ids;
		valid_cluster_ids.reserve(counts.size());
		for (const auto& [cluster_id, count] : counts)
		{
			if (count > params_.minimum_points_stem)
			{
				valid_cluster_ids.push_back(cluster_id);
			}
		}

		// initialize result set
		TreeData result;
		result.tree_descriptors.reserve(valid_cluster_ids.size());
		result.cluster_indicator = ArrayClusterIndicator(num_voxels);
		result.axis_distance     = Eigen::VectorXd(num_voxels);

		// Height range (actual value, not the %) that points should extend throughout
		const double h_range_value = (stripe_.upper_limit - stripe_.lower_limit) * params_.height_range;
		// Compute bounding box of the voxelated point cloud
		const Vec3 bb_min = voxelated_cloud.colwise().minCoeff().transpose();
		const Vec3 bb_max = voxelated_cloud.colwise().maxCoeff().transpose();

		for (const auto stem_id : valid_cluster_ids)
		{
			const auto  stem_num_points = counts[stem_id];
			PointCloud3 stem_cloud(stem_num_points, 3);
			// accumulate with max precision
			double z0_accumulator = 0.0;
			Vec3   coord_accumulator(0.0, 0.0, 0.0);

			Eigen::Index stem_point_id = 0;
			for (Eigen::Index point_id = 0; point_id < num_points; ++point_id)
			{
				if (stripe_.cluster_indicator(point_id) == stem_id)
				{
					const Vec3& stem_point = point_cloud_.row(point_id);
					coord_accumulator += stem_point;
					z0_accumulator += z0_(point_id);
					stem_cloud.row(stem_point_id++) = stem_point;
				}
			}
			const auto stem_height_range = stem_cloud.col(2).maxCoeff() - stem_cloud.col(2).minCoeff();
			if (stem_height_range > h_range_value)
			{
				TreeDescriptor tree_descriptor(stem_id);
				// get min diff in scalar type unused in 3DFin
				tree_descriptor.centroid_coordinates = coord_accumulator / stem_num_points;
				tree_descriptor.height_difference    = static_cast<double>(tree_descriptor.centroid_coordinates.z() - (z0_accumulator / stem_num_points));

				// compute the (3, 3) covariance matrix
				const PointCloud3     centered_cloud = stem_cloud.rowwise() - tree_descriptor.centroid_coordinates.transpose();
				const Eigen::Matrix3d covariance     = (centered_cloud.transpose() * centered_cloud) / double(stem_num_points);

				// Eigen decomposition of the covariance
				Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es(covariance);

				// Eigen values are sorted in increasing order, we looks for the more significant component / axis
				tree_descriptor.setAxis(es.eigenvectors().col(2), params_.axis_maximum_vertical_deviation);

				// safe guard
				if (tree_descriptor.axis_vertical_deviation > 88.0)
				{
					std::cout << "[Individualize] invalid axis, tree skipped" << std::endl;
					continue;
				}

				result.tree_descriptors.push_back(tree_descriptor);
			}
		}

		// Generate axis clouds
		Eigen::Index             total_axis_point = 0;
		std::vector<PointCloud3> vec_axis_point_clouds;
		for (auto& tree_descriptor : result.tree_descriptors)
		{
			auto axis_point_cloud = tree_descriptor.computeAxisSampling(bb_min, bb_max, sample_step);
			total_axis_point += axis_point_cloud.rows();
			vec_axis_point_clouds.push_back(std::move(axis_point_cloud));
		}

		std::cout << "[Individualize] num valid trees: " << result.tree_descriptors.size() << std::endl;

		// Concat axis clouds
		ArrayClusterIndicator axis_indicator(total_axis_point);
		PointCloud3           concat_axis_point_cloud(total_axis_point, 3);
		Eigen::Index          padding = 0;
		for (size_t valid_tree_number = 0; valid_tree_number < result.tree_descriptors.size(); ++valid_tree_number)
		{
			Eigen::Index       axis_id                                    = result.tree_descriptors[valid_tree_number].tree_id;
			const PointCloud3& axis_pointcloud                            = vec_axis_point_clouds[valid_tree_number];
			const auto         axis_num_points                            = axis_pointcloud.rows();
			concat_axis_point_cloud.block(padding, 0, axis_num_points, 3) = std::move(axis_pointcloud);
			axis_indicator.segment(padding, axis_num_points).setConstant(axis_id);
			padding += axis_num_points;
		}
		vec_axis_point_clouds.clear();
		vec_axis_point_clouds.shrink_to_fit();

		// KD-tree search
		using kd_tree_t = nanoflann::KDTreeEigenMatrixAdaptor<const PointCloud3, 3, nanoflann::metric_L2_Simple>;
		kd_tree_t kd_tree(3, concat_axis_point_cloud, 10, 0);

		tf::Executor executor;
		tf::Taskflow taskflow;
		// for point in voxelated-cloud, query
		const double sq_dmax = params_.maximum_dist_axis * params_.maximum_dist_axis;
		taskflow.for_each_index(
		    Eigen::Index(0), num_voxels, Eigen::Index(1), [&](Eigen::Index voxel_id)
		    {
				    Eigen::Index                                         index;
				    double sq_distance = 0.0;
					kd_tree.index_->knnSearch(voxelated_cloud.row(voxel_id).data(), 1, &index, &sq_distance);

					if(sq_distance > sq_dmax)
					{
					    result.cluster_indicator(voxel_id) = NO_CLUSTER_ID;
						result.axis_distance(voxel_id) = params_.maximum_dist_axis;
					} else {
					    result.cluster_indicator(voxel_id) =  axis_indicator(index);
						result.axis_distance(voxel_id) = std::sqrt(sq_distance);
					} });
		executor.run(taskflow).get();

		return result;
	}

	void TreeIndividualizer::compute_heights(
	    const PointCloud3& voxelated_cloud,
	    TreeData&          axis_data)
	{
		// large voxel to avoid underpopulated cells
		PointCloud3        large_voxels_cloud;
		VecIndex<uint32_t> vox_to_large_vox;
		std::tie(large_voxels_cloud, vox_to_large_vox) = voxelize(voxelated_cloud, params_.resolution_height, params_.resolution_height, true);
		const double      eps                          = params_.resolution_height * std::sqrt(3) + 1e-6;
		VecIndex<int32_t> cluster_labels               = connected_components(large_voxels_cloud, eps, 2);

		// Count clusters
		std::unordered_map<int32_t, uint32_t> label_counts;
		for (size_t voxel_id = 0; voxel_id < cluster_labels.size(); ++voxel_id)
		{
			++label_counts[cluster_labels(voxel_id)];
		}

		// Find large clusters
		std::set<uint32_t> valid_clusters;
		for (const auto& [label, count] : label_counts)
		{
			if (label != NO_CLUSTER_ID && count > 3)
			{
				valid_clusters.insert(label);
			}
		}

		// Eliminating all points that belong to clusters with less than 4 points (large voxels), and which dist_axis < d and in not valid_tree_id set
		tf::Executor executor;
		tf::Taskflow taskflow;
		taskflow.for_each(std::begin(axis_data.tree_descriptors), std::end(axis_data.tree_descriptors), [&](TreeDescriptor& tree_descriptor)
		                  {
			double       max_z    = std::numeric_limits<double>::lowest();
			Eigen::Index max_z_id = 0;
			const auto   tree_id  = tree_descriptor.tree_id;
			for (size_t voxel_id = 0; voxel_id < voxelated_cloud.rows(); ++voxel_id)
			{
				const auto point_tree_id = axis_data.cluster_indicator[voxel_id];
				if (point_tree_id != tree_id)
					continue;
				const auto large_vox_id = vox_to_large_vox(voxel_id);
				if (!valid_clusters.count(cluster_labels[large_vox_id]))
					continue;
				if (axis_data.axis_distance[voxel_id] > params_.height_distance_from_axis)
					continue;

				if (voxelated_cloud(voxel_id, 2) > max_z)
				{
					max_z_id = voxel_id;
					max_z    = voxelated_cloud(voxel_id, 2);
				}
			}
			tree_descriptor.setHeighestPoint(voxelated_cloud.row(max_z_id)); });

		executor.run(taskflow).get();
	}

} // namespace lib3dfin
