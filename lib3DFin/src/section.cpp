// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright 2023-2026 Carlos Cabo <carloscabo@uniovi.es>

// local
#include "section.hpp"

#include "circle_fit.hpp"
#include "slink.hpp"
#include "statistics.hpp"
#include "types.hpp"

// taskflow
#include <taskflow/algorithm/for_each.hpp>
#include <taskflow/taskflow.hpp>

// spdlog
#include <spdlog/spdlog.h>

// stdlib
#include <cassert>

namespace lib3dfin
{

	SectionExtractor::SectionExtractor(const PointCloud3& point_cloud, const ArrayClusterIndicator& sections_indicator, const Eigen::VectorXd& z0, const StemSectionParams& params, tf::Executor& executor)
	    : point_cloud_(point_cloud)
	    , section_indicator_(sections_indicator)
	    , num_points_(point_cloud.rows())
	    , z0_(z0)
	    , num_sections_(static_cast<Eigen::Index>(std::floor((params.stem_maximum_height - params.stem_minimum_height) / params.stem_section_interval)))
	    , params_(params)
	    , executor_(executor)
	{
	}

	void SectionExtractor::extract(TreeData& trees) const
	{

		std::chrono::steady_clock::time_point start = std::chrono::steady_clock::now();
		spdlog::info("[SectionExtractor] Computing diameters along stems...");

		// iterate over the trees
		for (auto& tree : trees.tree_descriptors)
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

			tf::Taskflow taskflow;
			auto         compute_section = taskflow.for_each_index(Eigen::Index(0), num_sections_, Eigen::Index(1), [&](Eigen::Index section_id)
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
					return;
				}

				PointCloud2 section_cloud(num_section_points, 2);

				Eigen::Index section_output_id = 0;
				for (Eigen::Index point_id = 0; point_id < tree_cloud.rows(); ++point_id)
				{
					if (section_mask(point_id))
					{
	                    section_cloud.row(section_output_id++) = tree_cloud.row(point_id).head<2>();
					}
				}

				// fit circle
				fitCircle(section_cloud, cur_circle);
				// if the fitting failed, we cluster the cloud with single linkage algorithm and try fitting again
				if (cur_circle.status != CircleData::Status::SUCCESS)
				{
					// Exctract the largest CC.
					const auto max_cc_section = fcluster_naive(section_cloud, params_.stem_section_clustering_distance);

					// No luck with single linkage clustering, we pass this section
					if (max_cc_section.size() < params_.stem_section_min_points)
					{
						cur_circle.status = CircleData::Status::NOT_ENOUGH_POINTS;
						return;
					}

					// Else  rerun the circle fitting algorithm on the clustered cloud
					fitCircle(max_cc_section, cur_circle);
				} });

			executor_.run(taskflow).wait();
			// detect tilt outliers on the fitted sections
			tiltDetection(circles);
			tree.circle_data = std::move(circles);
		}

		std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
		spdlog::info("[SectionExtractor] Computing diameters along stems done in {} ms", std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count());
	}

	void SectionExtractor::fitCircle(const PointCloud2& section_cloud, CircleData& circle_data) const
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

	uint32_t SectionExtractor::innerCircle(const PointCloud2& circle_cloud, const Circle& circle_params) const
	{
		const double sq_threshold = (circle_params.radius * params_.stem_section_diameter_proportion) * (circle_params.radius * params_.stem_section_diameter_proportion);
		// vectorize
		auto num_valid_points = ((circle_cloud.rowwise() - circle_params.center.transpose()).rowwise().squaredNorm().array() < sq_threshold).count();
		return num_valid_points;
	}

	uint32_t SectionExtractor::sectorOccupancy(const PointCloud2& circle_cloud, const Circle& circle_params) const
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
			const uint32_t clamped_sector = std::min(sector, params_.stem_section_sector_count - 1); // be sure we don't exceed the maximum sector index

			sector_occupancy_indicator[clamped_sector] = true;
		}

		// count the number of occupied sectors
		const uint32_t num_occupied_sectors = std::count(std::cbegin(sector_occupancy_indicator),
		                                                 std::cend(sector_occupancy_indicator),
		                                                 true);
		// number of occupied sectors
		return num_occupied_sectors;
	}

	// tilt detection for all sections of a given stem
	void SectionExtractor::tiltDetection(CircleSections& circles,
	                                     const double    abs_weight_factor,
	                                     const double    rel_weight_factor) const
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
} // namespace lib3dfin
