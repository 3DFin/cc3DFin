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
	class LocalizationExtractor
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
		explicit LocalizationExtractor(TreeData& trees, const Parameters params, tf::Executor& executor);
		void extract();

	  private: // static methods
		static bool                      checkRadiiConsistency(const CircleSections& circles, size_t lower, size_t upper, double factor);
		static bool                      checkRadiiConsistencyMADS(const CircleSections& circles, size_t lower, size_t upper);
		static bool                      checkTwoRadiiConsistency(const CircleSections& circles, size_t lower, size_t upper, double factor);
		static std::pair<size_t, size_t> countValidSections(const CircleSections& circles, size_t lower, size_t upper);

	  private: // methods
		void              computeDBHSectionID();
		void              computeDBHRangeIDs();
		bool              checkRadiiConsistencyProgressive(const CircleSections& circles, size_t lower, size_t upper);
		TreeLocatorResult axisLocation(const TreeDescriptor& tree_descriptor) const;
		TreeLocatorResult dbhLocation(const TreeDescriptor& tree_descriptor, size_t section_index, const CircleSections& circles) const;
		TreeLocatorResult treeLocator(const TreeDescriptor& tree_descriptor);

	  private: // members
		TreeData&        trees_;
		const Parameters params_;
		Eigen::Index     num_sections_{0};
		size_t           dbh_section_id_{0};
		size_t           lower_d_section_{0};
		size_t           upper_d_section_{0};
		size_t           total_sections_{0};
		uint32_t         num_pass_test_{0};
		uint32_t         total_pass_test_{0};
		tf::Executor&    executor_;
	};
} // namespace lib3dfin
