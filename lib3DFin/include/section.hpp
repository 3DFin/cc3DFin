#pragma once

// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright 2023-2025 Carlos Cabo <carloscabo@uniovi.es>

// local
#include "circle_fit.hpp"
#include "slink.hpp"
#include "types.hpp"

namespace lib3dfin
{
	template <typename real_t>
	class SectionExtractor
	{

		using CircleData     = CircleData<real_t>;
		using CircleSections = CircleSections<real_t>;

	  public: // struct
		struct Params
		{
			real_t       stem_minimum_height{0.3};
			real_t       stem_maximum_height{25.0};
			real_t       section_length{0.2};
			real_t       section_width{0.05};
			uint32_t     inner_circle_point_threshold{5};
			real_t       stem_diameter_proportion{0.5};
			real_t       stem_minimum_diameter{0.09};
			real_t       stem_maximum_diameter{1.0};
			real_t       circle_point_distance{0.02};
			uint32_t     min_num_points_section{80};
			uint32_t     total_number_sectors{16};
			uint32_t     minimum_number_sectors{9};
			real_t       circle_width{0.02};
			real_t       outlier_probability_threshold{0.3}; // this is added vs. the original implementation
			const real_t DBH{1.3};
		};

	  public:
		explicit SectionExtractor(const RefPointCloud<real_t>& point_cloud, const Eigen::VectorX<real_t>& z0, const AxesData<real_t>& trees, const Params params = Params())
		    : point_cloud_(point_cloud)
		    , num_points_(point_cloud.rows())
		    , z0_(z0)
		    , trees_(trees)
		    , num_sections_(static_cast<Eigen::Index>(std::floor((params_.stem_maximum_height - params_.stem_minimum_height) / params_.section_length)))
		    , params_(std::move(params))
		{
			// Iinitialize DBH
			computeDBHSectionID();
		}

		std::vector<CircleSections> extract()
		{
			std::vector<CircleSections> tree_circle_sections;
			// iterate over the trees
			for (const auto& tree : trees_.tree_descriptors)
			{
				const auto& cluster_indicator = trees_.axis_cluster_indicator;

				const auto         tree_mask          = (cluster_indicator.array() == tree.tree_id);
				const Eigen::Index number_points_tree = tree_mask.count();

				PointCloud3<real_t> tree_cloud(number_points_tree, 3);
				Eigen::Index        output_id = 0;
				for (Eigen::Index point_id = 0; point_id < num_points_; ++point_id)
				{
					if (tree_mask(point_id))
					{
						tree_cloud.row(output_id) << point_cloud_.row(point_id).template head<2>(), z0_(point_id);
						output_id++;
					}
				}

				CircleSections circles(num_sections_);

				for (Eigen::Index section_id = 0; section_id < num_sections_; ++section_id)
				{
					const auto section_start = params_.stem_minimum_height + section_id * params_.section_length;
					const auto section_end   = section_start + params_.section_width;
					auto&      cur_circle    = circles[section_id];
					cur_circle.z0            = section_start;

					const auto         section_mask       = (tree_cloud.col(2).array() >= section_start) && (tree_cloud.col(2).array() < section_end);
					const Eigen::Index num_section_points = section_mask.count();

					if (num_section_points < params_.min_num_points_section)
					{
						cur_circle.status = CircleData::Status::NOT_ENOUGH_POINTS;
						continue;
					}

					PointCloud2<real_t> section_cloud(num_section_points, 2);

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
					fit_circle(section_cloud, cur_circle);
					if (cur_circle.status != CircleData::Status::SUCCESS)
					{
						// cluster the cloud with single linkage algorithm
						// rerun the algorithm on the clustered cloud
						const auto max_cc_section = fcluster_slink(section_cloud, params_.circle_width);

						if (max_cc_section.size() < params_.min_num_points_section)
						{
							cur_circle.status = CircleData::Status::NOT_ENOUGH_POINTS;
							continue;
						}

						fit_circle(max_cc_section, cur_circle);
					}
				}
				tilt_detection(circles);
				const auto tree_localization = tree_locator(circles, tree);
				tree_circle_sections.emplace_back(std::move(circles));
			}
			return tree_circle_sections;
		}

	  private: // methods
		void fit_circle(const PointCloud2<real_t>& section_cloud, CircleData& circle_data)
		{
			circle_data.circle                  = LMCircleFit(section_cloud);
			const Circle<real_t>& circle_params = circle_data.circle;

			if (circle_params.radius < params_.stem_minimum_diameter / 2)
			{
				circle_data.status = CircleData::Status::DIAMETER_TOO_SMALL;
				return;
			}

			if (circle_params.radius > params_.stem_maximum_diameter / 2)
			{
				circle_data.status = CircleData::Status::DIAMETER_TOO_LARGE;
				return;
			}

			circle_data.number_points_inner = innerCircle(section_cloud, circle_params);

			if (circle_data.number_points_inner > params_.inner_circle_point_threshold)
			{
				circle_data.status = CircleData::Status::TOO_MANY_POINTS_INNER;
				return;
			}

			const auto num_occupied_sectors = sectorOccupancy(section_cloud, circle_params);
			circle_data.sector_percentage   = static_cast<real_t>(num_occupied_sectors) / params_.total_number_sectors;

			if (num_occupied_sectors < params_.minimum_number_sectors)
			{
				circle_data.status = CircleData::Status::NOT_ENOUGH_SECTOR_COVERAGE;
				return;
			}
			circle_data.status = CircleData::Status::SUCCESS;
		}

		uint32_t innerCircle(const PointCloud2<real_t>& circle_cloud, const Circle<real_t>& circle_params)
		{
			const real_t sq_threshold = (circle_params.radius * params_.stem_diameter_proportion) * (circle_params.radius * params_.stem_diameter_proportion);
			// vectorize
			auto num_valid_points = ((circle_cloud.rowwise() - circle_params.center.transpose()).rowwise().squaredNorm().array() < sq_threshold).count();
			return num_valid_points;
		}

		uint32_t sectorOccupancy(const PointCloud2<real_t>& circle_cloud, const Circle<real_t>& circle_params)
		{
			const real_t R_min_sq        = (circle_params.radius - params_.circle_width) * (circle_params.radius - params_.circle_width);
			const real_t R_max_sq        = (circle_params.radius + params_.circle_width) * (circle_params.radius + params_.circle_width);
			const real_t inv_sector_size = static_cast<real_t>(params_.total_number_sectors) / (2.0 * M_PI);

			const Eigen::Index n_points = circle_cloud.rows();

			std::vector<bool> sector_occupancy_indicator(params_.total_number_sectors, false);

			for (Eigen::Index point_id = 0; point_id < n_points; ++point_id)
			{
				const Vec2<real_t>& point     = circle_cloud.row(point_id);
				const Vec2<real_t>  red_point = point - circle_params.center;

				// Check radial constraint using squared distances
				const real_t r_sq = red_point.squaredNorm();
				if (r_sq < R_min_sq || r_sq > R_max_sq)
					continue;

				// Check angular constraint
				real_t angle = std::atan2(red_point.y(), red_point.x());
				if (angle < 0)
					angle += 2.0 * M_PI;

				const uint32_t sector         = static_cast<uint32_t>(std::floor(angle * inv_sector_size));
				const uint32_t clamped_sector = std::min(sector, params_.total_number_sectors - 1); // be sure we don't exceed the maximum sector index //TODO clamp angle instead

				sector_occupancy_indicator[clamped_sector] = true;
			}

			// count the number of occupied sectors
			const uint32_t num_occupied_sectors = std::count(std::begin(sector_occupancy_indicator),
			                                                 std::end(sector_occupancy_indicator),
			                                                 true);
			// percentage of occupied sectors
			return num_occupied_sectors;
		}

		std::array<real_t, 2> quantiles(const Eigen::VectorX<real_t>& tilt_data, const std::array<real_t, 2>& bounds = {0.25, 0.75})
		{

			if (tilt_data.size() == 0)
			{
				return {0, 0};
			}
			const size_t max_id       = std::ceil(tilt_data.size() * bounds[1]);
			const size_t num_elements = max_id + 1;

			// Create a deep copy to sort the data
			Eigen::VectorX<real_t> partial_tilt_data(num_elements);
			std::partial_sort_copy(std::begin(tilt_data), std::begin(tilt_data) + num_elements, std::begin(partial_tilt_data), std::end(partial_tilt_data));

			std::array<real_t, 2> result;

			// compute with linear interpolation like the default in numpy
			for (size_t id_bound = 0; id_bound < 2; ++id_bound)
			{
				const real_t id_pos   = bounds[id_bound] * (tilt_data.size() - 1);
				const size_t id_left  = static_cast<size_t>(std::floor(id_pos));
				const size_t id_right = static_cast<size_t>(std::ceil(id_pos));

				if (id_left == id_right)
				{
					result[id_bound] = partial_tilt_data(id_left);
					continue;
				}

				const real_t weight = id_pos - id_left;
				result[id_bound]    = partial_tilt_data(id_left) * (1.0 - weight) + partial_tilt_data(id_right) * weight;
			}
			return result;
		}

		Eigen::VectorX<bool> interquartile_range(const Eigen::VectorX<real_t>& data_vector,
		                                         real_t                        lower_q = 0.25,
		                                         real_t                        upper_q = 0.75,
		                                         real_t                        n_range = 1.5)
		{

			const auto quartiles = quantiles(data_vector, {lower_q, upper_q});

			const real_t iqr = quartiles[1] - quartiles[0];

			const real_t lower_bound = quartiles[0] - iqr * n_range;
			const real_t upper_bound = quartiles[1] + iqr * n_range;

			return (data_vector.array() < lower_bound || data_vector.array() > upper_bound);
		}

		// tilt dection for all sections of a given stem
		void tilt_detection(CircleSections& circles,
		                    const real_t    abs_weight_factor = real_t(3.0),
		                    const real_t    rel_weight_factor = real_t(1.0))
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
			const real_t total_weight  = (num_valid_sections - 1) * rel_weight_factor + abs_weight_factor;
			const real_t abs_outlier_w = abs_weight_factor / total_weight;
			const real_t rel_outlier_w = rel_weight_factor / total_weight;

			// tilt matrix = atan(xy / z)
			Eigen::MatrixX<real_t> tilt_matrix(num_valid_sections, num_valid_sections);
			tilt_matrix.diagonal().setZero(); // Initialize diagonal to zero

			for (size_t i = 0; i < num_valid_sections; ++i)
			{
				for (size_t j = i + 1; j < num_valid_sections; ++j)
				{
					const real_t height_difference = std::abs(circles[valid_ids[i]].z0 - circles[valid_ids[j]].z0);
					const real_t planar_distance   = (circles[valid_ids[i]].circle.center - circles[valid_ids[j]].circle.center).norm();
					// Since we prune i == j,
					// Rhere is no way z_dist could be zero, so the following atan is safe.
					// Original implementation convert tilt in degrees but there is no need IQR computation
					const real_t tilt_angle = std::atan2(height_difference, planar_distance);
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
			// the original algorithm compute a median to assign weight to the current section
			// we compute directy the IQR on the OTHER section and keep track of their indices
			// this is clearer, more logical and efficient. So we need at least 2 sections to compute the IQR.
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
			// filter outliers based on probability threshold
			for (size_t valid_section_id = 0; valid_section_id < num_valid_sections; ++valid_section_id)
			{
				if (circles[valid_ids[valid_section_id]].outlier_probability > params_.outlier_probability_threshold)
				{
					circles[valid_ids[valid_section_id]].status = CircleData::Status::TILT_OUTLIER;
				}
			}
		}

	  private:
		void computeDBHSectionID()
		{
			real_t min_diff = std::abs(params_.stem_minimum_height - params_.DBH);

			for (size_t section_id = 1; section_id < static_cast<size_t>(num_sections_); ++section_id)
			{
				const real_t section_height = params_.stem_minimum_height + section_id * params_.section_length;
				const real_t diff           = std::abs(section_height - params_.DBH);
				if (diff < min_diff)
				{
					min_diff        = diff;
					dbh_section_id_ = static_cast<size_t>(section_id);
				}
			}
		}

		std::tuple<size_t, size_t, size_t> getDBHRange() const
		{
			const size_t lower_d_section = std::max(dbh_section_id_ - size_t(1), size_t(0));
			const size_t upper_d_section = std::min(static_cast<size_t>(num_sections_), dbh_section_id_ + 2);
			const size_t total_sections  = upper_d_section - lower_d_section;
			return {lower_d_section, upper_d_section, total_sections};
		}

		std::pair<size_t, size_t> count_valid_sections(const CircleSections& circles, size_t lower, size_t upper) const
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

		bool check_radius_coherence(const CircleSections& circles, size_t lower, size_t upper) const
		{
			assert(upper - lower == 3);
			std::array<real_t, 3> valid_radius;
			size_t                i = 0;
			for (size_t j = lower; j < upper; ++j, ++i)
			{
				valid_radius[i] = circles[j].circle.radius;
			}

			// Compute median
			std::array<real_t, 3> sorted_radius = valid_radius;
			std::sort(std::begin(sorted_radius), std::end(sorted_radius));
			const real_t median_radius = sorted_radius[1];

			// Compute median absolute deviation
			std::array<real_t, 2> abs_deviations = {
			    std::abs(sorted_radius[0] - median_radius),
			    std::abs(sorted_radius[2] - median_radius)};

			return std::max(abs_deviations[0], abs_deviations[1]) < 3 * std::min(abs_deviations[0], abs_deviations[1]);
		}

		bool check_two_radius_coherence(const CircleSections& circles, size_t idx1, size_t idx2, real_t factor) const
		{
			const real_t radius1    = circles[idx1].circle.radius;
			const real_t radius2    = circles[idx2].circle.radius;
			const real_t max_radius = std::max(radius1, radius2);
			return std::abs(radius1 - radius2) < max_radius * factor;
		}

		TreeLocatorResult<real_t> axis_location(const TreeDescriptor<real_t>& tree_descriptor) const
		{
			TreeLocatorResult<real_t> result;
			result.dbh = 0.0; // No evaluation of the DBH

			const real_t cos_deviation = std::cos(tree_descriptor.axis_vertical_deviation * DEG_TO_RAD<real_t>);
			// axis_verical_deviation is filtered a priori, so there is no chance of division by zero.
			assert(abs(cos_deviation) > 1e-8);

			const real_t diff_height       = params_.DBH - tree_descriptor.height_difference;
			const real_t dist_centroid_dbh = diff_height / cos_deviation;
			result.location                = tree_descriptor.axis * dist_centroid_dbh + tree_descriptor.centroid_coordinates;

			return result;
		}

		TreeLocatorResult<real_t> dbh_location(const TreeDescriptor<real_t>& tree_descriptor, size_t section_index, const CircleSections& circles) const
		{

			TreeLocatorResult<real_t> result;
			result.dbh         = circles[section_index].circle.radius * 2.0;
			result.location(0) = circles[section_index].circle.center(0);
			result.location(1) = circles[section_index].circle.center(1);
			result.location(2) = tree_descriptor.height_difference + params_.DBH;
			return result;
		}

		TreeLocatorResult<real_t> tree_locator(const CircleSections& circles, const TreeDescriptor<real_t>& tree_descriptor) const
		{
			// Early exit if minimum section height is above DBH
			if (params_.stem_minimum_height > params_.DBH)
				return axis_location(tree_descriptor);

			// Find section closest to breast height and its neighborhood
			const auto [lower_d_section, upper_d_section, total_sections] = getDBHRange();

			// Check section validity in DBH neighborhood
			const auto [num_valid_circles, num_enough_sector_coverage] = count_valid_sections(circles, lower_d_section, upper_d_section);

			if (num_valid_circles < total_sections || num_valid_circles < 2)
				return axis_location(tree_descriptor);

			const bool all_sections_valids = (num_valid_circles == total_sections) && (num_enough_sector_coverage == total_sections);

			// Handle edge cases first
			// all sections are valid but there is only two valid circles dbh_section_id == 0 or dbh_section_id == total_sections - 1
			if (num_valid_circles == 2 && all_sections_valids)
			{
				// case that arise if dbh_section_id == 0
				if (lower_d_section == 0)
				{
					// First section case - check coherence between two sections
					if (check_two_radius_coherence(circles, lower_d_section, lower_d_section + 1, 0.1))
						return dbh_location(tree_descriptor, dbh_section_id_, circles);
					else
						return axis_location(tree_descriptor);
				}

				// case that arise if dbh_section_id == total_sections - 1
				if (upper_d_section == static_cast<size_t>(num_sections_))
				{
					// Last section case
					if (check_two_radius_coherence(circles, upper_d_section - 2, upper_d_section - 1, 0.15))
						return dbh_location(tree_descriptor, dbh_section_id_, circles);
					else
						return axis_location(tree_descriptor);
				}
			}

			// General case with 3 sections
			// if there are at least 3 valid cicles but some of them have a low coverage
			assert(num_valid_circles == 3);
			if (num_enough_sector_coverage < 3)
			{
				return axis_location(tree_descriptor);
			}

			// else we can take the average of the sections estimations and check coherence
			// if the coherence test fails, we fall back to axis estimation
			if (check_radius_coherence(circles, lower_d_section, upper_d_section))
				return dbh_location(tree_descriptor, dbh_section_id_, circles);
			else
			{
				return axis_location(tree_descriptor);
			}
		}

	  private: // members
		const RefPointCloud<real_t>&  point_cloud_;
		const Eigen::VectorX<real_t>& z0_;
		const AxesData<real_t>&       trees_;
		const Eigen::Index            num_points_;
		const Params                  params_;
		const Eigen::Index            num_sections_;
		size_t                        dbh_section_id_{0};
	};

} // namespace lib3dfin
