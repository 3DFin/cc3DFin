#pragma once

// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright 2023-2025 Carlos Cabo <carloscabo@uniovi.es>

#include "config.hpp"
#include "types.hpp"

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
		explicit HeightNormalization(const PointCloud3& point_cloud, HeightNormalization::Parameters params);
		Eigen::VectorXd normalize();

	  private: // methods
		PointCloud3 denoiseCloud();
		void        generateDTM(const PointCloud3& dtm_point_cloud);
		void        cleanDTM();

	  private: // members
		const PointCloud3& point_cloud_;
		const Parameters   params_;
		PointCloud3        dtm_;
	};

} // namespace lib3dfin
