#pragma once

// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright 2023-2025 Carlos Cabo <carloscabo@uniovi.es>

// local
#include "config.hpp"
#include "types.hpp"

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
			double   smooth_laplacian_lambda{0.125};
			bool     clean_dtm{true}; // New, not mapped to any global config

			static Parameters FromGlobalConfig(const Params& params)
			{
				return Parameters{
				    params.cloth_resolution,
				    params.denoise_point_cloud,
				    params.denoise_resolution,
				    params.denoise_minimum_points,
				    params.dtm_smooth_laplacian_lambda,
				    true};
			}
		};

	  public:
		explicit HeightNormalization(const PointCloud3& point_cloud, HeightNormalization::Parameters params, tf::Executor& executor);

		Eigen::VectorXd normalize();

		static std::pair<bool, double> checkHeightNormDiscrepancy(const PointCloud3& point_cloud, const Eigen::VectorXd& z0, double original_area, tf::Executor& executor, double res_xy = 1.0, double z_min = -0.1, double z_max = 0.15, double threshold = 0.1);

		const DTMData exportDTM() const;

	  private: // methods
		PointCloud3 denoiseCloud();
		void        generateDTM(const PointCloud3& dtm_point_cloud);
		void        cleanDTMmad();
		void        smoothDTMmedian();
		void        smoothDTMLaplacian();
		void        normalizeGrid();

	  private: // members
		const PointCloud3& point_cloud_;
		const Parameters   params_;
		PointCloud3        dtm_;
		std::vector<bool>  mask_;
		size_t             width_{0};
		size_t             height_{0};
		tf::Executor&      executor_;
	};

} // namespace lib3dfin
