#pragma once

// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright 2023-2025 Carlos Cabo <carloscabo@uniovi.es>

#include <cstdint>

namespace lib3dfin
{

	// WIP. comment show original variable names (in dendromatics) if we changed the name
	struct Params
	{
		// WIP. comment show original variable names (in dendromatics) if we changed the name
		// Global
		bool compute_height_normalization{true};
		// Ground Parameters (ground.hpp)
		double   cloth_resolution{0.45};
		bool     denoise_point_cloud{false};
		double   denoise_resolution{0.15};
		uint32_t denoise_minimum_points{2};
		// bool     clean_dtm{true}; Not mapped yet
	};
} // namespace lib3dfin
