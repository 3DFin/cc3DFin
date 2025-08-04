#pragma once

// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright 2023-2025 Carlos Cabo <carloscabo@uniovi.es>

#include "types.hpp"

namespace lib3dfin
{
	template <typename real_t>
	class HeightNormalization
	{
	  public: // struct
		struct Parameters
		{
			real_t   cloth_resolution       = 0.45; // basic / res_cloth // changed from 0.7 to 0.45
			bool     denoise_point_cloud    = false;
			real_t   denoise_resolution     = 0.15; // res_ground
			uint32_t denoise_minimum_points = 2;    // minimum_points_ground
			bool     clean_dtm              = true;
		};

	  public:
		explicit HeightNormalization(const RefPointCloud<real_t>& point_cloud_, HeightNormalization::Parameters params = HeightNormalization::Parameters());
		Eigen::VectorX<real_t> normalize();

	  private: // methods
		PointCloud3<real_t> denoiseCloud();
		void                generateDTM(const RefPointCloud<real_t>& dtm_point_cloud);
		void                cleanDTM();

	  private: // members
		const RefPointCloud<real_t>& point_cloud_;
		Parameters                   params_;
		PointCloud3<real_t>          dtm_;

	};

} // namespace lib3dfin
