#pragma once

// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright 2023-2026 Carlos Cabo <carloscabo@uniovi.es>

// local
#include "config.hpp"
#include "types.hpp"

// std
#include <array>

// taskflow
#include <taskflow/taskflow.hpp>

namespace lib3dfin
{
	class LocalizationExtractor
	{
	  public: // methods
		explicit LocalizationExtractor(const StemSectionParams& params, tf::Executor& executor);
		void extract(TreeData& trees);

	  private:

		//static members
		static constexpr size_t maxNumSections = 5;

		// static methods
		static std::array<int, maxNumSections> getNeighborhood(const TreeDescriptor& tree_descriptor, size_t dbh_id);


		// methods
		void              computeDBHSectionID();
		void              computeDBHRangeIDs();
		[[nodiscard]] TreeLocatorResult axisLocation(const TreeDescriptor& tree_descriptor) const;
		[[nodiscard]] TreeLocatorResult dbhLocation(const TreeDescriptor& tree_descriptor, size_t section_index, const CircleSections& circles, const DBHSource& dbh_source) const;
		TreeLocatorResult treeLocator(const TreeDescriptor& tree_descriptor);

		const StemSectionParams params_;
		size_t                  num_sections_{0};
		size_t                  bh_section_id_{0};
		size_t                  lower_d_section_{0};
		size_t                  upper_d_section_{0};
		size_t                  total_sections_{0};
		tf::Executor&           executor_;
	};
} // namespace lib3dfin
