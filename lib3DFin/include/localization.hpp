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
	  public: // methods
		explicit LocalizationExtractor(TreeData& trees, const StemSectionParams& params, tf::Executor& executor);
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
		TreeLocatorResult dbhLocation(const TreeDescriptor& tree_descriptor, size_t section_index, const CircleSections& circles, const DBHSource& dbh_source) const;
		TreeLocatorResult treeLocator(const TreeDescriptor& tree_descriptor);

	  private: // members
		TreeData&               trees_;
		const StemSectionParams params_;
		Eigen::Index            num_sections_{0};
		size_t                  bh_section_id_{0};
		size_t                  lower_d_section_{0};
		size_t                  upper_d_section_{0};
		size_t                  total_sections_{0};
		uint32_t                num_pass_test_{0};
		uint32_t                total_pass_test_{0};
		tf::Executor&           executor_;
	};
} // namespace lib3dfin
