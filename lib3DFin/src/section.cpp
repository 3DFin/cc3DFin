// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright 2023-2025 Carlos Cabo <carloscabo@uniovi.es>

// local
#include "section.hpp"

#include "circle_fit.hpp"
#include "slink.hpp"
#include "statistics.hpp"
#include "types.hpp"

// spdlog
#include <spdlog/spdlog.h>

// stdlib
#include <cassert>

namespace lib3dfin
{

	SectionExtractor::SectionExtractor(const PointCloud3& point_cloud, const ArrayClusterIndicator& sections_indicator, const Eigen::VectorXd& z0, TreeData& trees, const Parameters params)
	    : point_cloud_(point_cloud)
	    , section_indicator_(sections_indicator)
	    , num_points_(point_cloud.rows())
	    , z0_(z0)
	    , trees_(trees)
	    , num_sections_(static_cast<Eigen::Index>(std::floor((params_.stem_maximum_height - params_.stem_minimum_height) / params_.stem_section_interval)))
	    , params_(params)
	{
		// Iinitialize DBH
		computeDBHSectionID();
	}

	void SectionExtractor::extract()
	{
		spdlog::info("[SectionExtractor] Computing diameters along stems...");

		// iterate over the trees
		for (auto& tree : trees_.tree_descriptors)
		{
			const auto         tree_mask          = (section_indicator_.array() == tree.tree_id);
			const Eigen::Index number_points_tree = tree_mask.count();

			PointCloud3  tree_cloud(number_points_tree, 3);
			Eigen::Index output_id = 0;
			for (Eigen::Index point_id = 0; point_id < num_points_; ++point_id)
			{
				if (tree_mask(point_id))
				{
					tree_cloud.row(output_id) << point_cloud_.row(point_id).head<2>(), z0_(point_id);
					output_id++;
				}
			}

			CircleSections circles(num_sections_);

			// TODO: use taskflow
			for (Eigen::Index section_id = 0; section_id < num_sections_; ++section_id)
			{
				const auto section_start = params_.stem_minimum_height + section_id * params_.stem_section_interval;
				const auto section_end   = section_start + params_.stem_section_thickness;
				auto&      cur_circle    = circles[section_id];
				cur_circle.z0            = section_start;

				const auto         section_mask       = (tree_cloud.col(2).array() >= section_start) && (tree_cloud.col(2).array() < section_end);
				const Eigen::Index num_section_points = section_mask.count();

				if (num_section_points < params_.stem_section_min_points)
				{
					cur_circle.status = CircleData::Status::NOT_ENOUGH_POINTS;
					continue;
				}
				PointCloud2 section_cloud(num_section_points, 2);

				Eigen::Index section_output_id = 0;
				for (Eigen::Index point_id = 0; point_id < tree_cloud.rows(); ++point_id)
				{
					if (section_mask(point_id))
					{
						section_cloud(section_output_id, 0) = tree_cloud(point_id, 0);
						section_cloud(section_output_id, 1) = tree_cloud(point_id, 1);
						section_output_id++;
					}
				}

				// fit_circle
				fitCircle(section_cloud, cur_circle);
				// if the fitting failed, we cluster the cloud with single linkage algorithm and try fitting again
				if (cur_circle.status != CircleData::Status::SUCCESS)
				{
					// Exctract the largest CC.
					// rerun the circle fitting algorithm on this clustered cloud
					const auto max_cc_section = fcluster_naive(section_cloud, params_.stem_section_clustering_distance);

					// no luck with single linkage clustering, we pass this section
					if (max_cc_section.size() < params_.stem_section_min_points)
					{
						cur_circle.status = CircleData::Status::NOT_ENOUGH_POINTS;
						continue;
					}

					fitCircle(max_cc_section, cur_circle);
				}
			}

			// detect tilt outliers on the fitted sections
			tiltDetection(circles);

			tree.circle_data = std::move(circles);
			// run tree localization on the fitted sections
			auto tree_localization = treeLocator(tree);
			tree.setLocation(tree_localization);
		}
		spdlog::info("[SectionExtractor] Computing diameters along stems done");
	}

	void SectionExtractor::fitCircle(const PointCloud2& section_cloud, CircleData& circle_data)
	{
		circle_data.circle          = LMCircleFit(section_cloud);
		const Circle& circle_params = circle_data.circle;

		if (circle_params.radius < params_.stem_section_minimum_diameter / 2.0)
		{
			circle_data.status = CircleData::Status::DIAMETER_TOO_SMALL;
			return;
		}

		if (circle_params.radius > params_.stem_section_maximum_diameter / 2.0)
		{
			circle_data.status = CircleData::Status::DIAMETER_TOO_LARGE;
			return;
		}

		circle_data.number_points_inner = innerCircle(section_cloud, circle_params);

		if (circle_data.number_points_inner > params_.stem_section_inner_point_threshold)
		{
			circle_data.status = CircleData::Status::TOO_MANY_POINTS_INNER;
			return;
		}

		const auto num_occupied_sectors = sectorOccupancy(section_cloud, circle_params);
		circle_data.sector_percentage   = static_cast<double>(num_occupied_sectors) / params_.stem_section_sector_count;

		if (num_occupied_sectors < params_.stem_section_min_occupied_sectors)
		{
			circle_data.status = CircleData::Status::NOT_ENOUGH_SECTOR_COVERAGE;
			return;
		}
		circle_data.status = CircleData::Status::SUCCESS;
	}

	uint32_t SectionExtractor::innerCircle(const PointCloud2& circle_cloud, const Circle& circle_params)
	{
		const double sq_threshold = (circle_params.radius * params_.stem_section_diameter_proportion) * (circle_params.radius * params_.stem_section_diameter_proportion);
		// vectorize
		auto num_valid_points = ((circle_cloud.rowwise() - circle_params.center.transpose()).rowwise().squaredNorm().array() < sq_threshold).count();
		return num_valid_points;
	}

	uint32_t SectionExtractor::sectorOccupancy(const PointCloud2& circle_cloud, const Circle& circle_params)
	{
		const double R_min_sq        = (circle_params.radius - params_.stem_section_circle_width) * (circle_params.radius - params_.stem_section_circle_width);
		const double R_max_sq        = (circle_params.radius + params_.stem_section_circle_width) * (circle_params.radius + params_.stem_section_circle_width);
		const double inv_sector_size = static_cast<double>(params_.stem_section_sector_count) / (2.0 * M_PI);

		const Eigen::Index n_points = circle_cloud.rows();

		std::vector<bool> sector_occupancy_indicator(params_.stem_section_sector_count, false);

		for (Eigen::Index point_id = 0; point_id < n_points; ++point_id)
		{
			const Vec2& point     = circle_cloud.row(point_id);
			const Vec2  red_point = point - circle_params.center;

			// Check radial constraint using squared distances
			const double r_sq = red_point.squaredNorm();
			if (r_sq < R_min_sq || r_sq > R_max_sq)
				continue;

			// Check angular constraint
			double angle = std::atan2(red_point.y(), red_point.x());
			if (angle < 0)
				angle += 2.0 * M_PI;

			const uint32_t sector         = static_cast<uint32_t>(std::floor(angle * inv_sector_size));
			const uint32_t clamped_sector = std::min(sector, params_.stem_section_sector_count - 1); // be sure we don't exceed the maximum sector index //TODO clamp angle instead

			sector_occupancy_indicator[clamped_sector] = true;
		}

		// count the number of occupied sectors
		const uint32_t num_occupied_sectors = std::count(std::cbegin(sector_occupancy_indicator),
		                                                 std::cend(sector_occupancy_indicator),
		                                                 true);
		// percentage of occupied sectors
		return num_occupied_sectors;
	}

	// tilt dection for all sections of a given stem
	void SectionExtractor::tiltDetection(CircleSections& circles,
	                                     const double    abs_weight_factor,
	                                     const double    rel_weight_factor)
	{
		std::vector<size_t> valid_ids;
		valid_ids.reserve(circles.size());

		for (size_t section_id = 0; section_id < circles.size(); ++section_id)
		{
			if (circles[section_id].status == CircleData::Status::SUCCESS)
			{
				valid_ids.push_back(section_id);
			}
		}

		const size_t num_valid_sections = valid_ids.size();

		if (num_valid_sections == 0)
			return;

		// compute outlier weights
		// vs. the original implementation, num_valid_sections - 1  is the correct way to compute outlier weights
		// because the current section (i==j) can't be an outlier to itself.
		const double total_weight  = (num_valid_sections - 1) * rel_weight_factor + abs_weight_factor;
		const double abs_outlier_w = abs_weight_factor / total_weight;
		const double rel_outlier_w = rel_weight_factor / total_weight;

		// tilt matrix = atan(xy / z)
		Eigen::MatrixXd tilt_matrix(num_valid_sections, num_valid_sections);
		tilt_matrix.diagonal().setZero(); // Initialize diagonal to zero

		for (size_t i = 0; i < num_valid_sections; ++i)
		{
			for (size_t j = i + 1; j < num_valid_sections; ++j)
			{
				const double height_difference = std::abs(circles[valid_ids[i]].z0 - circles[valid_ids[j]].z0);
				const double planar_distance   = (circles[valid_ids[i]].circle.center - circles[valid_ids[j]].circle.center).norm();
				// Since we prune i == j,
				// There is no way z_dist could be zero, so the following atan is safe.
				// Original implementation convert tilt in degrees but there is no need IQR computation
				const double tilt_angle = std::atan2(height_difference, planar_distance);
				tilt_matrix(i, j)       = tilt_angle;
				tilt_matrix(j, i)       = tilt_angle;
			}
		}

		// sum tilts per rows
		const Eigen::VectorXd tilt_sum = tilt_matrix.rowwise().sum();

		// absolute outliers by IQR
		const auto abs_outliers_mask = interquartile_range(tilt_sum);

		for (size_t k = 0; k < num_valid_sections; ++k)
		{
			if (abs_outliers_mask(k))
				circles[valid_ids[k]].outlier_probability += abs_outlier_w;
		}

		// relative outliers
		// the original algorithm compute a median to assign q weight to the current section
		// we compute directy the IQR on the OTHER section and keep track of their indices
		// this is clearer, more logical and efficient.
		// We need at least 2 sections to compute the IQR.
		if (num_valid_sections < 2)
			return;

		for (size_t valid_section_id = 0; valid_section_id < num_valid_sections; ++valid_section_id)
		{
			// Create vector of all other sections' tilt values
			Eigen::VectorXd           other_sections(num_valid_sections - 1);
			std::vector<Eigen::Index> tilt_indices;
			tilt_indices.reserve(num_valid_sections - 1);

			Eigen::Index consecutive_id = 0;
			for (size_t other_section_id = 0; other_section_id < num_valid_sections; ++other_section_id)
			{
				if (other_section_id != valid_section_id)
				{
					other_sections(consecutive_id) = tilt_matrix(valid_section_id, other_section_id);
					tilt_indices.push_back(other_section_id);
					consecutive_id++;
				}
			}

			// Detect outliers among other sections
			const auto rel_outliers_mask = interquartile_range(other_sections);

			// Apply weights only to the outlier sections
			for (size_t other_section_id = 0; other_section_id < rel_outliers_mask.size(); ++other_section_id)
			{
				if (rel_outliers_mask(other_section_id))
				{
					const size_t current_section = tilt_indices[other_section_id];
					circles[valid_ids[current_section]].outlier_probability += rel_outlier_w;
				}
			}
		}
		// Mark outliers based on probability threshold
		for (size_t valid_section_id = 0; valid_section_id < num_valid_sections; ++valid_section_id)
		{
			if (circles[valid_ids[valid_section_id]].outlier_probability > params_.outlier_probability_threshold)
			{
				circles[valid_ids[valid_section_id]].status = CircleData::Status::TILT_OUTLIER;
			}
		}
	}

	void SectionExtractor::computeDBHSectionID()
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

	std::tuple<size_t, size_t, size_t> SectionExtractor::getDBHRange() const
	{
		const size_t lower_d_section = std::max(dbh_section_id_ - size_t(1), size_t(0));
		const size_t upper_d_section = std::min(static_cast<size_t>(num_sections_), dbh_section_id_ + 2);
		const size_t total_sections  = upper_d_section - lower_d_section;
		return {lower_d_section, upper_d_section, total_sections};
	}

	std::pair<size_t, size_t> SectionExtractor::countValidSections(const CircleSections& circles, size_t lower, size_t upper) const
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

	bool SectionExtractor::checkRadiiConsistency(const CircleSections& circles, size_t lower, size_t upper) const
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

	bool SectionExtractor::checkTwoRadiiConsistency(const CircleSections& circles, size_t idx1, size_t idx2, double factor) const
	{
		const double radius1    = circles[idx1].circle.radius;
		const double radius2    = circles[idx2].circle.radius;
		const double max_radius = std::max(radius1, radius2);
		return std::abs(radius1 - radius2) < max_radius * factor;
	}

	TreeLocatorResult SectionExtractor::axisLocation(const TreeDescriptor& tree_descriptor) const
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

	TreeLocatorResult SectionExtractor::dbhLocation(const TreeDescriptor& tree_descriptor, size_t section_index, const CircleSections& circles) const
	{
		TreeLocatorResult result;
		result.dbh         = circles[section_index].circle.radius * 2.0;
		result.location(0) = circles[section_index].circle.center(0);
		result.location(1) = circles[section_index].circle.center(1);
		result.location(2) = tree_descriptor.height_difference + params_.DBH;
		return result;
	}

	TreeLocatorResult SectionExtractor::treeLocator(const TreeDescriptor& tree_descriptor) const
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
