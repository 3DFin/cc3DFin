#pragma once

// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright 2023-2025 Carlos Cabo <carloscabo@uniovi.es>

#include "types.hpp"

#include <cstdint>

namespace lib3dfin
{
	template <typename real_t>
	class TreePeeler
	{
	  public: // struct
		struct Parameters
		{
			real_t   stripe_lower_limit    = 0.7;  // lower_limit
			real_t   stripe_upper_limit    = 3.5;  // lower_limit
			real_t   verticality_nn_scale  = 0.1;  // verticality_scale_stripe
			real_t   verticality_threshold = 0.7;  // / verticality_thresh_stripe
			uint32_t num_voxels_threshold  = 1000; // number_of_points
			real_t   resolution_xy         = 0.02;
			real_t   resolution_z          = 0.02;
			uint32_t num_iterations        = 2; // number_of_iterations
		};

	  public:
		explicit TreePeeler(const RefPointCloud<real_t>& point_cloud, const Eigen::VectorX<real_t>& z0, TreePeeler::Parameters params = TreePeeler::Parameters());
		PointCloud3<real_t> peel();

	  private: // methods
		PointCloud3<real_t> filter_stripe();
		PointCloud3<real_t> verticality_clustering(const PointCloud3<real_t>& stripe);

	  private: // members
		const RefPointCloud<real_t>&  point_cloud_;
		const Eigen::VectorX<real_t>& z0;
		const Parameters              params_;
		double total_time_ = 0.0;
	};

} // namespace lib3dfin
