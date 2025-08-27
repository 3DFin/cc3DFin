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
			real_t   verticality_nn_scale{0.1};  // verticality_scale_stripe
			real_t   verticality_threshold{0.7}; // / verticality_thresh_stripe
			uint32_t num_voxels_threshold{1000}; // number_of_points
			real_t   resolution_xy{0.02};
			real_t   resolution_z{0.02};
			uint32_t num_iterations{2}; // number_of_iterations
		};

	public: // static
		static ArrayClusterIndicator filterInitialStripe(const Eigen::VectorX<real_t>& z0, real_t stripe_lower_limit, real_t stripe_upper_limit);
		static ArrayClusterIndicator filterInitialStripe(const Eigen::VectorX<real_t>& z0, const Eigen::VectorX<real_t>& axis_distance, real_t max_distance, real_t stripe_lower_limit, real_t stripe_upper_limit);
		static PointCloud3<real_t>   extractStripe(const RefPointCloud<real_t>& point_cloud, const ArrayClusterIndicator& stripe_indicator);

	public:
		explicit TreePeeler(const RefPointCloud<real_t>& point_cloud, TreePeeler::Parameters params = TreePeeler::Parameters());
		void peel(ArrayClusterIndicator& stripe_indicator);

	  private: // methods
		ArrayClusterIndicator        verticalityClustering(const ArrayClusterIndicator& stripe_indicator);

	  private: // members
		const RefPointCloud<real_t>&  point_cloud_;
		const Eigen::Index            num_points_;
		const Parameters              params_;
		double                        total_time_ = 0.0;
	};

} // namespace lib3dfin
