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
		computeDBHRangeIDs();
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
		spdlog::info("num_pass_test_: {} / total_pass_test_: {}", num_pass_test_, total_pass_test_);
		spdlog::info("[LocalizationExtractor] Computing tree DBH and localization in {} us", std::chrono::duration_cast<std::chrono::microseconds>(end - start).count());
	}

	void LocalizationExtractor::computeDBHRangeIDs()
	{
		lower_d_section_ = std::max(dbh_section_id_ - size_t(1), size_t(0));
		upper_d_section_ = std::min(static_cast<size_t>(num_sections_ - 1), dbh_section_id_ + size_t(2));
		total_sections_  = upper_d_section_ - lower_d_section_;
	}

	std::pair<size_t, size_t> LocalizationExtractor::countValidSections(const CircleSections& circles, size_t lower, size_t upper)
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

	bool LocalizationExtractor::checkRadiiConsistency(const CircleSections& circles, size_t lower, size_t upper, double factor)
	{
		assert(upper - lower == 3);
		double rmax = circles[lower].circle.radius;
		double rmin = circles[lower].circle.radius;

		for (size_t j = lower + 1; j < upper; j++)
		{
			double cur_radius = circles[j].circle.radius;
			if (cur_radius > rmax)
				rmax = cur_radius;
			if (cur_radius < rmin)
				rmin = cur_radius;
		}

		double min_max = rmax / rmin;
		return min_max <= factor;
	}

	bool LocalizationExtractor::checkRadiiConsistencyProgressive(const CircleSections& circles, size_t lower, size_t upper)
	{
		assert(upper - lower == 3);
		double rmax = circles[lower].circle.radius;
		double rmin = circles[lower].circle.radius;

		for (size_t j = lower + 1; j < upper; j++)
		{
			double cur_radius = circles[j].circle.radius;
			if (cur_radius > rmax)
				rmax = cur_radius;
			if (cur_radius < rmin)
				rmin = cur_radius;
		}

		double min_max = rmax / rmin;
		if (min_max <= 1.05)
			return true;
		if (min_max > 1.10)
			return true;

		// expension
		lower = std::min(size_t(0), lower - 1);
		upper = std::max(upper + 1, static_cast<size_t>(num_sections_ - 1));

		double cur_radius = circles[lower].circle.radius;
		if (cur_radius > rmax)
			rmax = cur_radius;
		if (cur_radius < rmin)
			rmin = cur_radius;

		cur_radius = circles[upper].circle.radius;
		if (cur_radius > rmax)
			rmax = cur_radius;
		if (cur_radius < rmin)
			rmin = cur_radius;
		min_max = rmax / rmin;

		return min_max <= 1.10;
	}

	bool LocalizationExtractor::checkRadiiConsistencyMADS(const CircleSections& circles, size_t lower, size_t upper)
	{
		assert(upper - lower == 3);
		std::array<double, 3> valid_radii;
		size_t                i = 0;
		for (size_t j = lower; j < upper; ++j, ++i)
		{
			valid_radii[i] = circles[j].circle.radius;
		}

		// Compute median
		std::array<double, 3> sorted_radii = valid_radii;
		std::sort(std::begin(sorted_radii), std::end(sorted_radii));

		const double median_radius = sorted_radii[1];
		// Compute median absolute deviation for the two extremas
		// median absolute deviation for the median is... 0
		std::array<double, 2> abs_deviations = {
		    std::abs(sorted_radii[0] - median_radius),
		    std::abs(sorted_radii[2] - median_radius)};

		// 3 MADS
		return std::max(abs_deviations[0], abs_deviations[1]) < 3 * std::min(abs_deviations[0], abs_deviations[1]);
	}

	bool LocalizationExtractor::checkTwoRadiiConsistency(const CircleSections& circles, size_t lower, size_t upper, double factor)
	{
		const double radius1 = circles[lower].circle.radius;
		const double radius2 = circles[upper].circle.radius;
		const double rmin    = std::min(radius1, radius2);
		const double rmax    = std::max(radius1, radius2);

		return rmax / rmin <= factor;
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

	TreeLocatorResult LocalizationExtractor::treeLocator(const TreeDescriptor& tree_descriptor)
	{
		// Early return if DBH is not included in the range of admissible stem sizes
		if (params_.stem_minimum_height >= params_.DBH || params_.stem_maximum_height <= params_.DBH)
			return axisLocation(tree_descriptor); // Find the closest section to the DBH and its neighborhood

		if (total_sections_ < 2)
			return axisLocation(tree_descriptor);

		// Check section validity of the DBH neighborhood
		const auto [num_valid_circles, num_enough_sector_coverage] = countValidSections(tree_descriptor.circle_data, lower_d_section_, upper_d_section_);

		if (num_valid_circles < 2)
			return axisLocation(tree_descriptor);

		const bool all_sections_valids = (num_valid_circles == total_sections_) && (num_enough_sector_coverage == total_sections_);
		// Handle edge cases first
		// Case where there are only two sections and both are valid
		// dbh_section_id == 0 or dbh_section_id == total_sections - 1

		if (num_valid_circles == 2 && all_sections_valids)
		{
			double cur_threshold{0.0};
			// case that arise if dbh_section_id == 0
			if (lower_d_section_ == 0)
				cur_threshold = 0.1;
			// case that arise if dbh_section_id == num_sections - 1
			else if (upper_d_section_ == static_cast<size_t>(num_sections_ - 1))
				cur_threshold = 0.15;

			// check coherence between two sections
			if (checkTwoRadiiConsistency(tree_descriptor.circle_data, lower_d_section_, upper_d_section_, cur_threshold))
				return dbhLocation(tree_descriptor, dbh_section_id_, tree_descriptor.circle_data);
			else
				return axisLocation(tree_descriptor);
		}

		// if there are at least 3 valid cicles but some of them have a low coverage
		if (num_enough_sector_coverage < 3)
			return axisLocation(tree_descriptor);

		// General case with 3 sections
		assert(total_sections_ == 3);
		// Else we can take the average of the sections estimations and check coherence
		// if the coherence test fails, we fall back to axis estimation
		total_pass_test_++;
		if (checkRadiiConsistency(tree_descriptor.circle_data, lower_d_section_, upper_d_section_, 1.05))
		{
			++num_pass_test_;
			return dbhLocation(tree_descriptor, dbh_section_id_, tree_descriptor.circle_data);
		}
		else
			return axisLocation(tree_descriptor);
	}

} // namespace lib3dfin
