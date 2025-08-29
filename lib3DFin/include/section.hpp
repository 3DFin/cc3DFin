#pragma once

// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright 2023-2025 Carlos Cabo <carloscabo@uniovi.es>

// local
#include "circle_fit.hpp"
#include "slink.hpp"
#include "types.hpp"

// std::lib
#include <iostream>

namespace lib3dfin
{
	template <typename real_t>
	class SectionExtractor
	{
	  public: // struct
		struct Params
		{
			real_t   stem_minimum_height{0.3};
			real_t   stem_maximum_height{25.0};
			real_t   section_length{0.2};
			real_t   section_width{0.05};
			uint32_t inner_circle_point_threshold{5};
			real_t   stem_diameter_proportion{0.5};
			real_t   stem_minimum_diameter{0.09};
			real_t   stem_maximum_diameter{1.0};
			real_t   circle_point_distance{0.02};
			uint32_t min_num_points_section{80};
			uint32_t total_number_sectors{16};
			uint32_t minimum_number_sectors{9};
			real_t   circle_width{0.02};
		};

		struct CircleData
		{
			real_t       radius{0.0};
			Vec3<real_t> center{0.0, 0.0, 0.0};
			bool         is_valid{false};    // check
			bool         second_time{false}; // second time //TODO not convinced, eliminate this indicator
			real_t       sector_percentage{0.0};
			uint32_t     number_points_inner{0};
		};

	  public:
		explicit SectionExtractor(const RefPointCloud<real_t>& point_cloud, const Eigen::VectorX<real_t>& z0, const AxesData<real_t>& trees, const Params params = Params())
		    : point_cloud_(point_cloud)
		    , num_points_(point_cloud.rows())
		    , z0_(z0)
		    , trees_(trees)
		    , params_(std::move(params))
		{
		}

		std::optional<Circle<real_t>> fit_circle(const PointCloud2<real_t>& section_cloud)
		{
			const auto circle_params = LMCircleFit(section_cloud);

			if (circle_params.radius < params_.stem_minimum_diameter / 2 || circle_params.radius > params_.stem_maximum_diameter / 2)
			{
				//std::cout << "failed at diameter check " << circle_params.radius << std::endl;
				return std::nullopt;
			}

			const uint32_t num_points_in = innerCircle(section_cloud, circle_params);

			if (num_points_in > params_.inner_circle_point_threshold)
			{
				//std::cout << "failed at inner circle point count check" << std::endl;
				return std::nullopt;
			}

			const real_t sector_percentage = sectorOccupancy(section_cloud, circle_params);

			if (sector_percentage * params_.total_number_sectors < params_.minimum_number_sectors)
			{
				//std::cout << "failed at sector check" << sector_percentage << std::endl;
				return std::nullopt;
			}

			return circle_params;
		}

		uint32_t innerCircle(const PointCloud2<real_t>& circle_cloud, const Circle<real_t>& circle_params)
		{
			uint32_t     num_valid_points = 0;
			const real_t sq_threshold     = (circle_params.radius * params_.stem_diameter_proportion) * (circle_params.radius * params_.stem_diameter_proportion);
			for (Eigen::Index point_id = 0; point_id < circle_cloud.rows(); ++point_id)
			{
				const Vec2<real_t>& point    = circle_cloud.row(point_id);
				const real_t        distance = (point - circle_params.center).squaredNorm();
				if (distance < sq_threshold)
				{
					num_valid_points++;
				}
			}
			return num_valid_points;
		}

		real_t sectorOccupancy(const PointCloud2<real_t>& circle_cloud, const Circle<real_t>& circle_params)
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

				real_t angle = std::atan2(red_point.y(), red_point.x());
				if (angle < 0)
					angle += 2.0 * M_PI; // Normalize to [0, 2π)

				const uint32_t sector         = static_cast<uint32_t>(std::floor(angle * inv_sector_size));
				const uint32_t clamped_sector = std::min(sector, params_.total_number_sectors - 1); // be sure we don't exceed the maximum sector index //TODO clamp angle

				sector_occupancy_indicator[clamped_sector] = true;
			}

			// count the number of occupied sectors
			const uint32_t num_occupied_sectors = std::count(std::begin(sector_occupancy_indicator),
			                                                 std::end(sector_occupancy_indicator),
			                                                 true);
			// percentage of occupied sectors
			real_t percentage_occupied_sectors = static_cast<real_t>(num_occupied_sectors) / params_.total_number_sectors;

			return percentage_occupied_sectors;
		}

		void extract()
		{
			// number of sections
			const Eigen::Index num_sections = static_cast<Eigen::Index>(std::floor((params_.stem_maximum_height - params_.stem_minimum_height) / params_.section_length));
			// iterate over the trees
			for (const auto& tree : trees_.tree_descriptors)
			{
				const auto  tree_id           = tree.tree_id;
				const auto& cluster_indicator = trees_.axis_cluster_indicator;

				const auto   tree_mask          = (cluster_indicator.array() == tree_id);
				Eigen::Index number_points_tree = tree_mask.count();
				std::cout << "Processing tree " << tree.tree_id << " with " << number_points_tree << " points" << std::endl;

				PointCloud3<real_t> tree_cloud(number_points_tree, 3);
				Eigen::Index        output_id = 0;
				for (Eigen::Index point_id = 0; point_id < num_points_; ++point_id)
				{
					if (tree_mask(point_id))
					{
						tree_cloud(output_id, 0) = point_cloud_(point_id, 0);
						tree_cloud(output_id, 1) = point_cloud_(point_id, 1);
						tree_cloud(output_id, 2) = z0_(point_id);
						output_id++;
					}
				}

				for (Eigen::Index section_id = 0; section_id < num_sections; ++section_id)
				{
					const auto section_start = params_.stem_minimum_height + section_id * params_.section_length;
					const auto section_end   = section_start + params_.section_width;

					const auto   section_mask       = (tree_cloud.col(2).array() >= section_start) && (tree_cloud.col(2).array() < section_end);
					Eigen::Index num_section_points = section_mask.count();

					if (num_section_points < params_.min_num_points_section)
						continue;

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
					auto circle_params = fit_circle(section_cloud);
					if (circle_params == std::nullopt)
					{
						// cluster the cloud with single linkage algorithm
						// rerun the algorithm on the clustered cloud
						auto max_cc_section = fcluster_slink(section_cloud, params_.circle_width);
						circle_params       = fit_circle(max_cc_section);
						if (circle_params == std::nullopt)
						{
							continue;
						}
					}
				}
			}
		}

	  private: // members
		const RefPointCloud<real_t>&  point_cloud_;
		const Eigen::VectorX<real_t>& z0_;
		const AxesData<real_t>&       trees_;
		const Eigen::Index            num_points_;
		const Params                  params_;
	};

} // namespace lib3dfin
