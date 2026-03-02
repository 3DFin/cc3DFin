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

	  public: // struct
		struct Parameters
		{
			double   stem_minimum_height{0.3};
			double   stem_maximum_height{25.0};
			double   stem_section_interval{0.2};
			double   stem_section_thickness{0.05};
			double   stem_section_circle_width{0.02};
			uint32_t stem_section_inner_point_threshold{5};
			double   stem_section_diameter_proportion{0.5};
			double   stem_section_minimum_diameter{0.09};
			double   stem_section_maximum_diameter{1.0};
			double   stem_section_clustering_distance{0.02};
			uint32_t stem_section_min_points{80};
			uint32_t stem_section_sector_count{16};
			uint32_t stem_section_min_occupied_sectors{9};
			double   outlier_probability_threshold{0.3}; // TODO: new, not mapped to in the GUI
			double   DBH{1.3};                           // TODO: new, not mapped to in the GUI

			static Parameters FromGlobalConfig(const Params& params)
			{
				return Parameters{
				    params.stem_minimum_height,
				    params.stem_maximum_height,
				    params.stem_section_interval,
				    params.stem_section_thickness,
				    params.stem_section_circle_width,
				    params.stem_section_inner_point_threshold,
				    params.stem_section_diameter_proportion,
				    params.stem_section_minimum_diameter,
				    params.stem_section_maximum_diameter,
				    params.stem_section_clustering_distance,
				    params.stem_section_min_points,
				    params.stem_section_sector_count,
				    params.stem_section_min_occupied_sectors};
				// config.outlier_probability_threshold,
				// config.DBH
			}
		};

	  public: // methods
		explicit SectionExtractor(const PointCloud3& point_cloud, const ArrayClusterIndicator& sections_indicator, const Eigen::VectorXd& z0, TreeData& trees, const Parameters params, tf::Executor& executor);
		void extract();

	  private: // methods
		void     fitCircle(const PointCloud2& section_cloud, CircleData& circle_data);
		uint32_t innerCircle(const PointCloud2& circle_cloud, const Circle& circle_params);
		uint32_t sectorOccupancy(const PointCloud2& circle_cloud, const Circle& circle_params);
		// tilt dection for all sections of a given stem
		void tiltDetection(CircleSections& circles,
		                   const double    abs_weight_factor = 3.0,
		                   const double    rel_weight_factor = 1.0);

	  private: // tree locations // methods
		void                               computeDBHSectionID();
		std::tuple<size_t, size_t, size_t> getDBHRange() const;
		std::pair<size_t, size_t>          countValidSections(const CircleSections& circles, size_t lower, size_t upper) const;
		bool                               checkRadiiConsistency(const CircleSections& circles, size_t lower, size_t upper) const;
		bool                               checkTwoRadiiConsistency(const CircleSections& circles, size_t idx1, size_t idx2, double factor) const;
		TreeLocatorResult                  axisLocation(const TreeDescriptor& tree_descriptor) const;
		TreeLocatorResult                  dbhLocation(const TreeDescriptor& tree_descriptor, size_t section_index, const CircleSections& circles) const;
		TreeLocatorResult                  treeLocator(const TreeDescriptor& tree_descriptor) const;

	  private: // members
		const PointCloud3&           point_cloud_;
		const ArrayClusterIndicator& section_indicator_;
		const Eigen::VectorXd&       z0_;
		TreeData&                    trees_;
		const Eigen::Index           num_points_;
		const Parameters             params_;
		const Eigen::Index           num_sections_;
		size_t                       dbh_section_id_{0};
		tf::Executor&                executor_;
	};

} // namespace lib3dfin
