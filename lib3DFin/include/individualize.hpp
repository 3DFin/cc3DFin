#pragma once

// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright 2023-2025 Carlos Cabo <carloscabo@uniovi.es>

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
			double   maximum_dist_axis{15.0};               // maximum_
			uint32_t minimum_points_stem{20};               // minimum_points this is the minimum number of voxels required to consider stripe cluster to be a valid as a stem
			double   axis_maximum_vertical_deviation{25.0}; // maximum_dev
			double   height_distance_from_axis{1.5};
			double   resolution_height{0.3};
		};

	  public: // methods
		explicit TreeIndividualizer(
		    const PointCloud3&     point_cloud,
		    const Stripe&          stripe,
		    const Eigen::VectorXd& z0,
		    const Parameters       params)
		    : point_cloud_(point_cloud)
		    , stripe_(stripe)
		    , z0_(z0)
		    , params_(std::move(params))
		{
		}

		TreeData individualize();

	  private: // methods
		TreeData computeAxesApproximate(
		    const PointCloud3& voxelated_cloud);

		void compute_heights(
		    const PointCloud3& voxelated_cloud,
		    TreeData&          axis_data);

	  private: // variables
		const PointCloud3&     point_cloud_;
		const Stripe&          stripe_;
		const Eigen::VectorXd& z0_;
		const Parameters       params_;
	};

} // namespace lib3dfin
