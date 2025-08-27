#pragma once

// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright 2023-2025 Carlos Cabo <carloscabo@uniovi.es>

#include "types.hpp"

// stdlib
#include <cstdint>

namespace lib3dfin
{

	template <typename real_t>
	class TreeIndividualizer
	{
	  public: // struct
		struct Parameters
		{
			real_t   resolution_xy{0.035};
			real_t   resolution_z{0.035};
			real_t   height_range{0.7};
			real_t   maximum_dist_axis{15.0};               // maximum_
			uint32_t minimum_points_stem{20};               // minimum_points this is the minimum number of voxels required to consider stripe cluster to be a valid as a stem
			real_t   axis_maximum_vertical_deviation{25.0}; // maximum_dev
			real_t   height_distance_from_axis{1.5};
			real_t   resolution_height{0.3};
		};

	  public: // methods
		explicit TreeIndividualizer(
		    const RefPointCloud<real_t>&  point_cloud,
		    const Stripe<real_t>&         stripe,
		    const Eigen::VectorX<real_t>& z0,
		    const Parameters              params = Parameters())
		    : point_cloud_(point_cloud)
		    , stripe_(stripe)
		    , z0_(z0)
		    , params_(std::move(params))
		{
		}

		AxesData<real_t> individualize();

	  private: // methods
		AxesData<real_t> computeAxesApproximate(
		    const PointCloud3<real_t>& voxelated_cloud);

		void compute_heights(
		    const PointCloud3<real_t>& voxelated_cloud,
		    AxesData<real_t>&          axis_data);

	  private: // variables
		const RefPointCloud<real_t>&  point_cloud_;
		const Stripe<real_t>&         stripe_;
		const Eigen::VectorX<real_t>& z0_;
		const Parameters              params_;
	};

} // namespace lib3dfin
