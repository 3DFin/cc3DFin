#pragma once

// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright 2023-2025 Carlos Cabo <carloscabo@uniovi.es>

#include "config.hpp"
#include "types.hpp"

// stdlib
#include <cstdint>

namespace lib3dfin
{

	class TreeIndividualizer
	{
	  public: // struct
		struct Parameters
		{
			double   resolution_xy{0.035};
			double   resolution_z{0.035};
			double   height_range{0.7};
			double   maximum_dist_axis{15.0};
			uint32_t minimum_points_stem{20};
			double   axis_maximum_vertical_deviation{25.0};
			double   height_distance_from_axis{1.5};
			double   resolution_height{0.3};

			static Parameters FromGlobalConfig(const Params& params)
			{
				return Parameters{
				    params.tree_resolution_xy,
				    params.tree_resolution_z,
				    params.tree_height_range,
				    params.tree_dist_axis_threshold,
				    params.tree_minimum_points_stem,
				    params.tree_axis_max_vertical_deviation,
				    params.tree_height_distance_from_axis,
				    params.tree_resolution_height};
			};
		};

	  public: // methods
		explicit TreeIndividualizer(const PointCloud3& point_cloud, const Stripe& stripe, const Eigen::VectorXd& z0, const Parameters params)
		    : point_cloud_(point_cloud)
		    , stripe_(stripe)
		    , z0_(z0)
		    , params_(params)
		{
		}

		TreeData individualize();

	  private: // methods
		TreeData computeAxesApproximate(
		    const PointCloud3& voxelated_cloud);

		void compute_heights(
		    const PointCloud3& voxelated_cloud,
		    TreeData&          axis_data) const;

	  private: // variables
		const PointCloud3&     point_cloud_;
		const Stripe&          stripe_;
		const Eigen::VectorXd& z0_;
		const Parameters       params_;
	};

} // namespace lib3dfin
