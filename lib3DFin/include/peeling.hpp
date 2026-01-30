#pragma once

// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright 2023-2025 Carlos Cabo <carloscabo@uniovi.es>

#include "config.hpp"
#include "types.hpp"

// StdLib
#include <cstdint>

namespace lib3dfin
{
	class FilterPredicate
	{
	  public: // methods
		FilterPredicate()          = default;
		virtual ~FilterPredicate() = default;

		//! Identity filtering
		virtual ArrayClusterIndicator filter(const PointCloud3& point_cloud, const Eigen::VectorXd& z0)
		{
			return ArrayClusterIndicator::Zero(z0.size());
		}
	};

	class StripeFilterPredicate : public FilterPredicate
	{
	  public: // methods
		StripeFilterPredicate(double stripe_lower_limit, double stripe_upper_limit)
		    : stripe_lower_limit_(stripe_lower_limit)
		    , stripe_upper_limit_(stripe_upper_limit)
		{
		}

		virtual ~StripeFilterPredicate() = default;
		virtual ArrayClusterIndicator filter(const PointCloud3& point_cloud, const Eigen::VectorXd& z0)
		{
			ArrayClusterIndicator stripe_cluster_indicator(z0.size());
			stripe_cluster_indicator.setConstant(NO_CLUSTER_ID);
			stripe_cluster_indicator = (z0.array() > stripe_lower_limit_ && z0.array() < stripe_upper_limit_).select(0, stripe_cluster_indicator);
			return stripe_cluster_indicator;
		}

	  private: // members
		double stripe_lower_limit_{0.};
		double stripe_upper_limit_{0.};
	};

	class StemFilterPredicate : public FilterPredicate
	{
	  public: // methods
		StemFilterPredicate(double stripe_lower_limit, double stripe_upper_limit, double max_distance, const Eigen::VectorXd& axis_distance)
		    : stripe_lower_limit_(stripe_lower_limit)
		    , stripe_upper_limit_(stripe_upper_limit)
		    , max_distance_(max_distance)
		    , axis_distance_(axis_distance)
		{
		}

		virtual ~StemFilterPredicate() = default;
		virtual ArrayClusterIndicator filter(const PointCloud3& point_cloud, const Eigen::VectorXd& z0)
		{
			assert(z0.size() == axis_distance_.size());
			ArrayClusterIndicator stripe_cluster_indicator(z0.size());
			for (Eigen::Index point_id = 0; point_id < z0.size(); ++point_id)
			{
				if (z0(point_id) > stripe_lower_limit_ && z0(point_id) < stripe_upper_limit_ && axis_distance_(point_id) < max_distance_)
				{
					stripe_cluster_indicator(point_id) = 0;
				}
				else
				{
					stripe_cluster_indicator(point_id) = NO_CLUSTER_ID;
				}
			}
			return stripe_cluster_indicator;
		}

	  private: // members
		double                 max_distance_{0.};
		double                 stripe_lower_limit_{0.};
		double                 stripe_upper_limit_{0.};
		const Eigen::VectorXd& axis_distance_;
	};

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
		static PointCloud3 extractStripe(const PointCloud3& point_cloud, const ArrayClusterIndicator& stripe_indicator);

	  public: // methods
		explicit TreePeeler(const PointCloud3& point_cloud, const Eigen::VectorXd& z0, std::unique_ptr<FilterPredicate> unitial_state, const TreePeeler::Parameters params);
		ArrayClusterIndicator peel();

	  private: // methods
		ArrayClusterIndicator verticalityClustering(const ArrayClusterIndicator& stripe_indicator);

	  private: // members
		const PointCloud3&               point_cloud_;
		const Eigen::VectorXd&           z0_;
		const Eigen::Index               num_points_;
		const Parameters                 params_;
		std::unique_ptr<FilterPredicate> initial_state_;
		double                           total_time_ = 0.0;
	};

} // namespace lib3dfin
