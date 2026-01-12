#pragma once

// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright 2023-2025 Carlos Cabo <carloscabo@uniovi.es>

#include "config.hpp"
#include "types.hpp"

// StdLib
#include <cstdint>

namespace lib3dfin
{
	class TreePeeler
	{
	  public: // struct
		struct Parameters
		{
			double   verticality_nn_scale{0.1};
			double   verticality_threshold{0.7};
			uint32_t num_voxels_threshold{1000};
			double   resolution_xy{0.02};
			double   resolution_z{0.02};
			uint32_t num_iterations{2};

			static Parameters StripeFromGlobalConfig(const Params& params)
			{
				return Parameters{
				    params.verticality_radius_stripe,
				    params.verticality_threshold_stripe,
				    params.stripe_peeling_voxels_threshold,
				    params.stripe_peeling_resolution_xy,
				    params.stripe_peeling_resolution_z,
				    params.stripe_peeling_num_iterations};
			}

			static Parameters StemFromGlobalConfig(const Params& params)
			{
				return Parameters{
				    params.verticality_radius_stem,
				    params.verticality_threshold_stem,
				    params.stripe_peeling_voxels_threshold,
				    params.stripe_peeling_resolution_xy,
				    params.stripe_peeling_resolution_z,
				    params.stripe_peeling_num_iterations};
			}
		};

	  public: // static
		static ArrayClusterIndicator filterInitialStripe(const Eigen::VectorXd& z0, double stripe_lower_limit, double stripe_upper_limit);
		static ArrayClusterIndicator filterInitialStripe(const Eigen::VectorXd& z0, const Eigen::VectorXd& axis_distance, double max_distance, double stripe_lower_limit, double stripe_upper_limit);
		static PointCloud3           extractStripe(const PointCloud3& point_cloud, const ArrayClusterIndicator& stripe_indicator);

	  public:
		explicit TreePeeler(const PointCloud3& point_cloud, TreePeeler::Parameters params);
		void peel(ArrayClusterIndicator& stripe_indicator);

	  private: // methods
		ArrayClusterIndicator verticalityClustering(const ArrayClusterIndicator& stripe_indicator);

	  private: // members
		const PointCloud3& point_cloud_;
		const Eigen::Index num_points_;
		const Parameters   params_;
		double             total_time_ = 0.0;
	};

} // namespace lib3dfin
