#pragma once

// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright 2023-2025 Carlos Cabo <carloscabo@uniovi.es>

// local
#include "config.hpp"
#include "types.hpp"
#include "voxel.hpp"

// taskflow
#include <taskflow/taskflow.hpp>

namespace lib3dfin
{
	class HeightNormalization
	{
	  public: // struct
		struct Parameters
		{
			double   cloth_resolution{0.45};
			bool     denoise_point_cloud{false};
			double   denoise_resolution{0.15};
			uint32_t denoise_minimum_points{2};
			bool     clean_dtm{true}; // New, not mapped to any global config

			static Parameters FromGlobalConfig(const Params& params)
			{
				return Parameters{
				    params.cloth_resolution,
				    params.denoise_point_cloud,
				    params.denoise_resolution,
				    params.denoise_minimum_points,
				    true};
			}
		};

	  public:
		explicit HeightNormalization(const PointCloud3& point_cloud, HeightNormalization::Parameters params, tf::Executor& executor);

		Eigen::VectorXd normalize();

		static std::pair<bool, double> checkHeightNormDiscrepancy(const PointCloud3& point_cloud, const Eigen::VectorXd& z0, double original_area, tf::Executor& executor, double res_xy = 1.0, double z_min = -0.1, double z_max = 0.15, double threshold = 0.1)
		{
			assert(z_min < z_max);
			assert(original_area > 0);
			assert(threshold > 0 && threshold < 1);

			// Compute the z resolution as a function of z_max - z_min
			const double res_z = (z_max - z_min) * 1.01;

			const ArrayMask pseudo_ground_mask = ((z0.array() >= z_min) && (z0.array() <= z_max));

			auto        pseudo_ground_point_count = pseudo_ground_mask.count();
			PointCloud3 pseudo_ground_cloud(pseudo_ground_point_count, 3);

			Eigen::Index filtered_point_id = 0;
			for (Eigen::Index point_id = 0; point_id < z0.size(); ++point_id)
			{
				if (pseudo_ground_mask(point_id))
				{
					pseudo_ground_cloud.row(filtered_point_id).head<2>() = point_cloud.row(point_id).head<2>();
					pseudo_ground_cloud.row(filtered_point_id++)(2)      = z0(point_id);
				}
			}

			const auto [voxel_cloud, _] = voxelize(pseudo_ground_cloud, res_xy, res_z, executor, false);

			//   # Area of the voxelated ground slice (n of voxels * area of voxel base)
			double slice_area = voxel_cloud.rows() * res_xy * res_xy;

			double threshold_difference = threshold * original_area;

			double area_difference = std::abs(original_area - slice_area);

			//  TODO: In very rare occasions, the slice area could be larger than the original
			//  area. The function should account for that, and return a different kind of
			//  warning for those situations (and its threshold could be different).
			//  For instance, if the original area has been computed through a grid of voxels
			//  (as this function does to compute slice_area) using a smaller voxel size,
			//  this could happen. We haven't tested it yet as we do not have access
			//  to any point clouds where this situation happens.

			//  Check if the difference is greater than 10 % of the first number
			return {area_difference >= threshold_difference, (area_difference * 100 / original_area)};
		}

		std::pair<std::vector<size_t>, PointCloud3> exportDTM();

	  private: // methods
		PointCloud3 denoiseCloud();
		void        generateDTM(const PointCloud3& dtm_point_cloud);
		void        cleanDTM();

	  private: // members
		const PointCloud3& point_cloud_;
		const Parameters   params_;
		PointCloud3        dtm_;
		size_t             width_{0};
		size_t             height_{0};
		tf::Executor&      executor_;
	};

} // namespace lib3dfin
