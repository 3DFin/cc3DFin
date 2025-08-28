#pragma once

// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright 2023-2025 Carlos Cabo <carloscabo@uniovi.es>

// local
#include "circle_fit.hpp"
#include "types.hpp"

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
			real_t   section_lenght{0.2};
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

		struct Circle
		{
			real_t       radius{0.0};
			Vec3<real_t> center{0.0, 0.0, 0.0};
			bool         is_valid{false};    // check
			bool         second_time{false}; // second time //TODO not convinced, eliminate this indicator
			real_t       sector_percentage{0.0};
			uint32_t     number_points_inner{0};
		};

	  public:
		explicit SectionExtractor(const PointCloud3<real_t>& cloud, const Eigen::VectorX<real_t>& z0, const AxesData<real_t>& trees, const Params& params)
		    : cloud_(cloud)
		    , num_points_(cloud.rows())
		    , trees_(trees)
		    , z0_(z0)
		    , params_(std::move(params))
		{
		}

		void extract()
		{
			// number of sections
			Eigen::Index num_sections = static_cast<Eigen::Index>(std::floor((params_.stem_maximum_height - params_.stem_minimum_height) / params_.section_length));
			// iterate over the trees
			for (const auto& tree : trees_.tree_descriptors)
			{
				const auto  tree_id           = tree.tree_id;
				const auto& cluster_indicator = trees_.axis_cluster_indicator;

				const auto   tree_mask          = (cluster_indicator.array() == tree_id);
				Eigen::Index number_points_tree = tree_mask.count();

				PointCloud3<real_t> tree_cloud(number_points_tree, 3);
				Eigen::Index        output_id = 0;
				for (Eigen::Index point_id = 0; point_id < num_points_; ++point_id)
				{
					if (tree_mask(point_id))
					{
						tree_cloud.row(output_id, 0)   = cloud_.row(point_id, 0);
						tree_cloud.row(output_id, 1)   = cloud_.row(point_id, 1);
						tree_cloud.row(output_id++, 2) = z0_(point_id);
					}
				}

				for (Eigen::Index section_id = 0; section_id < num_sections; ++section_id)
				{
					const auto section_start = params_.stem_minimum_height + section_id * params_.section_length;
					const auto section_end   = section_start + params_.section_width;

					const auto   section_mask       = (tree_cloud.row(2).array() >= section_start) && (tree_cloud.row(2).array() < section_end);
					Eigen::Index num_section_points = section_mask.count();

					if (num_section_points < params_.min_num_points_section)
						continue;

					PointCloud2<real_t> section_cloud(num_section_points, 3);

					Eigen::Index output_id = 0;
					for (Eigen::Index point_id = 0; point_id < num_section_points; ++point_id)
					{
						if (section_mask(point_id))
						{
							section_cloud.row(num_section_points++) = tree_cloud.row(point_id).template head<2>();
						}
					}
					// fit_circle
					const auto circle_params = LMCircleFit(section_cloud);

				}
			}
		}

	  private: // members
		const PointCloud3<real_t>&    cloud_;
		const Eigen::VectorX<real_t>& z0_;
		const AxesData<real_t>&       trees_;
		const Eigen::Index            num_points_;
		const Params&                 params_;
	};

} // namespace lib3dfin
