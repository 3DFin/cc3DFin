// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright 2023-2026 Carlos Cabo <carloscabo@uniovi.es>

// local
#include "localization.hpp"

// spdlog
#include <spdlog/spdlog.h>

namespace lib3dfin
{

	LocalizationExtractor::LocalizationExtractor(TreeData& trees, const StemSectionParams& params, tf::Executor& executor)
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
				min_diff       = diff;
				bh_section_id_ = static_cast<size_t>(section_id);
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
		lower_d_section_ = std::max(bh_section_id_ - size_t(2), size_t(0));
		upper_d_section_ = std::min(static_cast<size_t>(num_sections_ - 1), bh_section_id_ + size_t(2));
		total_sections_  = upper_d_section_ - lower_d_section_ + 1;
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

		const double cos_deviation = std::cos(tree_descriptor.axis.vertical_deviation_deg * DEG_TO_RAD);
		// axis_verical_deviation is filtered a priori, so there is no chance of division by zero.
		assert(abs(cos_deviation) > 1e-8);

		const double diff_height       = params_.DBH - tree_descriptor.location.centroid_coordinates.z() + tree_descriptor.dims.height_difference;
		const double dist_centroid_dbh = diff_height / cos_deviation;
		result.location                = tree_descriptor.axis.direction * dist_centroid_dbh + tree_descriptor.location.centroid_coordinates;
		result.dbh_source              = DBHSource::NOT_RELIABLE;
		return result;
	}

	TreeLocatorResult LocalizationExtractor::dbhLocation(const TreeDescriptor& tree_descriptor, size_t section_index, const CircleSections& circles, const DBHSource& dbh_source) const
	{
		TreeLocatorResult result;
		result.dbh = circles[section_index].circle.radius * 2.0;
		result.location << circles[section_index].circle.center, tree_descriptor.dims.height_difference + params_.DBH;
		result.dbh_source = dbh_source;
		return result;
	}

	std::array<int, 5> getNeighborhood(const TreeDescriptor& tree_descriptor, const size_t dbh_id)
	{
		std::array<int, 5> indices{-1, -1, -1, -1, -1};

		// -1 for OOB or status < sucess (i.e 0 diameter)
		for (int offset = -2; offset <= 2; ++offset)
		{
			int id = dbh_id + offset;
			if (id < 0 || id >= tree_descriptor.circle_data.size() || tree_descriptor.circle_data[id].status < CircleData::Status::SUCCESS)
				continue;

			indices[offset + 2] = id;
		}

		return indices;
	}

	TreeLocatorResult LocalizationExtractor::treeLocator(const TreeDescriptor& tree_descriptor)
	{
		// Early return if DBH is not included in the range of admissible stem sizes
		if (params_.stem_minimum_height >= params_.DBH || params_.stem_maximum_height <= params_.DBH)
			return axisLocation(tree_descriptor); // Find the closest section to the DBH and its neighborhood

		const auto& circle_data = tree_descriptor.circle_data;
		// For these sections, only consider diameter values that are non-zero,
		// it's true for status >= SUCCESS
		if (circle_data[bh_section_id_].status == CircleData::Status::SUCCESS)
			return dbhLocation(tree_descriptor, bh_section_id_, circle_data, DBHSource::BHSECTION_OQ_OK);

		auto dbh_nn_info = getNeighborhood(tree_descriptor, bh_section_id_);

		constexpr size_t bh_local_id = 2;
		constexpr double threshold   = 0.08;

		// eq. to tests bh_section has a non-zero diameter
		if (dbh_nn_info[bh_local_id] != -1)
		{
			double bh_radius = circle_data[bh_section_id_].circle.radius;

			for (size_t section_id = 0; section_id < 5; section_id++)
			{
				int section_global_id = dbh_nn_info[section_id];
				if (section_id == bh_local_id || section_global_id < 0)
					continue;

				double curr_radius = circle_data[section_global_id].circle.radius;
				double diff        = std::abs(bh_radius - curr_radius);
				double min_d       = std::min(bh_radius, curr_radius);
				if (diff / min_d < threshold)
					return dbhLocation(tree_descriptor, bh_section_id_, circle_data, DBHSource::BHSECTION_NEIGHBOUR_SUPPORT);
			}
		}

		// else test pairs and get the closest to the DBH
		// putative dbh z0: either we have a non-zero diameter dbh section and we take its z0, either we take the DBH value from parameters
		const double putative_dbh_z0 = dbh_nn_info[bh_local_id] < 0 ? params_.DBH : circle_data[bh_section_id_].z0;

		std::pair<int, int> best_pair{-1, -1};
		double              best_distance = std::numeric_limits<double>::max();
		int                 best_quality  = -1;
		double              best_diff     = std::numeric_limits<double>::max();

		for (size_t i = 0; i < 5; i++)
		{
			int i_global = dbh_nn_info[i];
			if (i == bh_local_id || i_global < 0)
				continue;

			for (size_t j = i + 1; j < 5; j++)
			{
				int j_global = dbh_nn_info[j];
				if (j == bh_local_id || j_global < 0)
					continue;

				double radius_i = circle_data[i_global].circle.radius;
				double radius_j = circle_data[j_global].circle.radius;
				if (std::min(radius_i, radius_j) == 0)
					continue;

				double diff_ratio = std::abs(radius_i - radius_j) / std::min(radius_i, radius_j);
				if (diff_ratio >= threshold)
					continue;

				double distance_i    = std::abs(circle_data[i_global].z0 - putative_dbh_z0);
				double distance_j    = std::abs(circle_data[j_global].z0 - putative_dbh_z0);
				double pair_distance = distance_i + distance_j;
				int    quality       = (circle_data[i_global].status == CircleData::Status::SUCCESS) + (circle_data[j_global].status == CircleData::Status::SUCCESS);

				// Multi criteria comparisons
				bool is_better = false;
				if (pair_distance < best_distance)
				{
					is_better = true;
				}
				else if (pair_distance == best_distance)
				{
					if (quality > best_quality)
					{
						is_better = true;
					}
					else if (quality == best_quality)
					{
						if (diff_ratio < best_diff)
						{
							is_better = true;
						}
					}
				}

				if (is_better || best_pair.first < 0)
				{
					best_pair     = {i_global, j_global};
					best_distance = pair_distance;
					best_quality  = quality;
					best_diff     = diff_ratio;
				}
			}
		}

		if (best_pair.first >= 0)
		{
			int selected = (std::abs(circle_data[best_pair.first].z0 - putative_dbh_z0) < std::abs(circle_data[best_pair.second].z0 - putative_dbh_z0)) ? best_pair.first : best_pair.second;
			return dbhLocation(tree_descriptor, selected, circle_data, DBHSource::NEIGHBOURING_CONSISTENT_PAIR);
		}

		// Last
		// Check for any neighboring section with Overall Quality = 1
		int    best_neighbor       = -1;
		double min_distance_to_dbh = std::numeric_limits<double>::max();

		for (size_t i = 0; i < 5; i++)
		{
			int global_id = dbh_nn_info[i];
			if (i == bh_local_id || global_id < 0)
				continue;

			if (circle_data[global_id].status == CircleData::Status::SUCCESS)
			{
				double dist = std::abs(circle_data[global_id].z0 - putative_dbh_z0);
				if (dist < min_distance_to_dbh)
				{
					min_distance_to_dbh = dist;
					best_neighbor       = global_id;
				}
			}
		}

		if (best_neighbor >= 0)
		{
			return dbhLocation(tree_descriptor, best_neighbor, circle_data, DBHSource::CLOSEST_OQ_OK_NEIGHBOUR);
		}

		return axisLocation(tree_descriptor);
	};

} // namespace lib3dfin
