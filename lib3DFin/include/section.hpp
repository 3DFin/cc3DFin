#pragma once

// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright 2023-2026 Carlos Cabo <carloscabo@uniovi.es>

// local
#include "config.hpp"
#include "types.hpp"

// taskflow
#include <taskflow/taskflow.hpp>

namespace lib3dfin
{
	class SectionExtractor
	{

	  public: // methods
		explicit SectionExtractor(const PointCloud3& point_cloud, const ArrayClusterIndicator& sections_indicator, const Eigen::VectorXd& z0, TreeData& trees, const StemSectionParams& params, tf::Executor& executor);
		void extract();

	  private: // methods
		void     fitCircle(const PointCloud2& section_cloud, CircleData& circle_data);
		uint32_t innerCircle(const PointCloud2& circle_cloud, const Circle& circle_params);
		uint32_t sectorOccupancy(const PointCloud2& circle_cloud, const Circle& circle_params);
		// tilt dection for all sections of a given stem
		void tiltDetection(CircleSections& circles,
		                   const double    abs_weight_factor = 3.0,
		                   const double    rel_weight_factor = 1.0);

	  private: // members
		const PointCloud3&           point_cloud_;
		const ArrayClusterIndicator& section_indicator_;
		const Eigen::VectorXd&       z0_;
		TreeData&                    trees_;
		const Eigen::Index           num_points_{0};
		const StemSectionParams      params_;
		const Eigen::Index           num_sections_{0};
		size_t                       dbh_section_id_{0};
		tf::Executor&                executor_;
	};

} // namespace lib3dfin
