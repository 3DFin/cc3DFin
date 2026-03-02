// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright 2023-2026 Carlos Cabo <carloscabo@uniovi.es>

// local
#include "localization.hpp"

// spdlog
#include <spdlog/spdlog.h>

namespace lib3dfin
{

	LocalizationExtractor::LocalizationExtractor(TreeData& trees, const Parameters params, tf::Executor& executor)
	    : trees_(trees)
	    , params_(params)
	    , executor_(executor)
	{
		assert(!trees_.tree_descriptors.empty());
		num_sections_ = trees_.tree_descriptors[0].circle_data.size();
		computeDBHSectionID();
	}

	void LocalizationExtractor::computeDBHSectionID()
	{
		double min_diff = std::abs(params_.stem_minimum_height - params_.DBH);

		for (size_t section_id = 1; section_id < static_cast<size_t>(num_sections_); ++section_id)
		{
			const double section_height = params_.stem_minimum_height + section_id * params_.stem_section_interval;
			const double diff           = std::abs(section_height - params_.DBH);
			if (diff < min_diff)
			{
				min_diff        = diff;
				dbh_section_id_ = static_cast<size_t>(section_id);
			}
		}
	}

	void LocalizationExtractor::extract()
	{
		std::chrono::steady_clock::time_point start = std::chrono::steady_clock::now();
		spdlog::info("[LocalizationExtractor] Computing tree DBH and localization...");
		for (auto& tree : trees_.tree_descriptors)
		{
			// run tree localization on the fitted sections
			const auto tree_localization = treeLocator(tree);
			tree.setLocation(tree_localization);
		}
		std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
		spdlog::info("[LocalizationExtractor] Computing tree DBH and localization in {} us", std::chrono::duration_cast<std::chrono::microseconds>(end - start).count());
	}

	std::tuple<size_t, size_t, size_t> LocalizationExtractor::getDBHRange() const
	{
		const size_t lower_d_section = std::max(dbh_section_id_ - size_t(1), size_t(0));
		const size_t upper_d_section = std::min(static_cast<size_t>(num_sections_), dbh_section_id_ + 2);
		const size_t total_sections  = upper_d_section - lower_d_section;
		return {lower_d_section, upper_d_section, total_sections};
	}

	std::pair<size_t, size_t> LocalizationExtractor::countValidSections(const CircleSections& circles, size_t lower, size_t upper) const
	{
		size_t valid_circles          = 0;
		size_t enough_sector_coverage = 0;

		for (size_t j = lower; j < upper; ++j)
		{
			if (circles[j].status == CircleData::Status::SUCCESS)
				valid_circles++;
			if (circles[j].sector_percentage > 0.3)
				enough_sector_coverage++;
		}
		return {valid_circles, enough_sector_coverage};
	}

	bool LocalizationExtractor::checkRadiiConsistency(const CircleSections& circles, size_t lower, size_t upper) const
	{
		assert(upper - lower == 3);
		std::array<double, 3> valid_radius;
		size_t                i = 0;
		for (size_t j = lower; j < upper; ++j, ++i)
		{
			valid_radius[i] = circles[j].circle.radius;
		}

		// Compute median
		std::array<double, 3> sorted_radius = valid_radius;
		std::sort(std::begin(sorted_radius), std::end(sorted_radius));
		const double median_radius = sorted_radius[1];

		// Compute median absolute deviation for the two extremas
		std::array<double, 2> abs_deviations = {
		    std::abs(sorted_radius[0] - median_radius),
		    std::abs(sorted_radius[2] - median_radius)};

		// 3 MADS
		return std::max(abs_deviations[0], abs_deviations[1]) < 3 * std::min(abs_deviations[0], abs_deviations[1]);
	}

	bool LocalizationExtractor::checkTwoRadiiConsistency(const CircleSections& circles, size_t idx1, size_t idx2, double factor) const
	{
		const double radius1    = circles[idx1].circle.radius;
		const double radius2    = circles[idx2].circle.radius;
		const double max_radius = std::max(radius1, radius2);
		return std::abs(radius1 - radius2) < max_radius * factor;
	}

	TreeLocatorResult LocalizationExtractor::axisLocation(const TreeDescriptor& tree_descriptor) const
	{
		TreeLocatorResult result;
		result.dbh = 0.0; // No evaluation of the DBH

		const double cos_deviation = std::cos(tree_descriptor.axis_vertical_deviation * DEG_TO_RAD);
		// axis_verical_deviation is filtered a priori, so there is no chance of division by zero.
		assert(abs(cos_deviation) > 1e-8);

		const double diff_height       = params_.DBH - tree_descriptor.centroid_coordinates.z() + tree_descriptor.height_difference;
		const double dist_centroid_dbh = diff_height / cos_deviation;
		result.location                = tree_descriptor.axis * dist_centroid_dbh + tree_descriptor.centroid_coordinates;
		return result;
	}

	TreeLocatorResult LocalizationExtractor::dbhLocation(const TreeDescriptor& tree_descriptor, size_t section_index, const CircleSections& circles) const
	{
		TreeLocatorResult result;
		result.dbh = circles[section_index].circle.radius * 2.0;
		result.location << circles[section_index].circle.center, tree_descriptor.height_difference + params_.DBH;
		return result;
	}

	TreeLocatorResult LocalizationExtractor::treeLocator(const TreeDescriptor& tree_descriptor) const
	{
		// Early return if DBH is not included in the range of admissible stem sizes
		if (params_.stem_minimum_height >= params_.DBH || params_.stem_maximum_height <= params_.DBH)
			return axisLocation(tree_descriptor); // Find the closest section to the DBH and its neighborhood
		const auto [lower_d_section, upper_d_section, total_sections] = getDBHRange();

		// Check section validity of the DBH neighborhood
		const auto [num_valid_circles, num_enough_sector_coverage] = countValidSections(tree_descriptor.circle_data, lower_d_section, upper_d_section);

		if (num_valid_circles < total_sections || num_valid_circles < 2)
		{
			return axisLocation(tree_descriptor);
		}

		const bool all_sections_valids = (num_valid_circles == total_sections) && (num_enough_sector_coverage == total_sections);

		// Handle edge cases first
		// all sections are valid but there is only two valid circles dbh_section_id == 0 or dbh_section_id == total_sections - 1
		if (num_valid_circles == 2 && all_sections_valids)
		{

			// case that arise if dbh_section_id == 0
			if (lower_d_section == 0)
			{
				// First section case - check coherence between two sections
				if (checkTwoRadiiConsistency(tree_descriptor.circle_data, lower_d_section, lower_d_section + 1, 0.1))
					return dbhLocation(tree_descriptor, dbh_section_id_, tree_descriptor.circle_data);
				else
					return axisLocation(tree_descriptor);
			}

			// case that arise if dbh_section_id == total_sections - 1
			if (upper_d_section == static_cast<size_t>(num_sections_))
			{
				// Last section case
				if (checkTwoRadiiConsistency(tree_descriptor.circle_data, upper_d_section - 2, upper_d_section - 1, 0.15))
					return dbhLocation(tree_descriptor, dbh_section_id_, tree_descriptor.circle_data);
				else
					return axisLocation(tree_descriptor);
			}
		}

		// General case with 3 sections
		// if there are at least 3 valid cicles but some of them have a low coverage
		assert(num_valid_circles == 3);
		if (num_enough_sector_coverage < 3)
		{
			return axisLocation(tree_descriptor);
		}

		// Else we can take the average of the sections estimations and check coherence
		// if the coherence test fails, we fall back to axis estimation
		if (checkRadiiConsistency(tree_descriptor.circle_data, lower_d_section, upper_d_section))
			return dbhLocation(tree_descriptor, dbh_section_id_, tree_descriptor.circle_data);
		else
		{
			return axisLocation(tree_descriptor);
		}
	}

} // namespace lib3dfin
