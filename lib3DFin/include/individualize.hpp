#pragma once

// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright 2023-2026 Carlos Cabo <carloscabo@uniovi.es>

#include "config.hpp"
#include "types.hpp"

// taskflow
#include <taskflow/taskflow.hpp>

// stdlib
#include <cstdint>

namespace lib3dfin
{

	class TreeIndividualizer
	{

	  public: // methods
		explicit TreeIndividualizer(const PointCloud3& point_cloud, const Stripe& stripe, const Eigen::VectorXd& z0, const TreeIndividualizerParams& params, tf::Executor& executor)
		    : point_cloud_(point_cloud)
		    , stripe_(stripe)
		    , z0_(z0)
		    , params_(params)
		    , executor_(executor)
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
		const PointCloud3&           point_cloud_;
		const Stripe&                stripe_;
		const Eigen::VectorXd&       z0_;
		const TreeIndividualizerParams params_;
		tf::Executor&                executor_;
	};

} // namespace lib3dfin
