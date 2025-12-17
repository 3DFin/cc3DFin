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

		// Stripe Peeling
		double   stripe_lower_limit{0.7};
		double   stripe_upper_limit{3.5};
		double   verticality_radius_stripe{0.1};
		double   verticality_threshold_stripe{0.7};
		uint32_t stripe_peeling_voxels_threshold{1000};
		double   stripe_peeling_resolution_xy{0.02};
		double   stripe_peeling_resolution_z{0.02};
		uint32_t stripe_peeling_num_iterations{2};

		// Tree individualizer
		double   tree_resolution_xy{0.035};
		double   tree_resolution_z{0.035};
		double   tree_height_range{0.7};
		double   tree_dist_axis_threshold{15.0};
		uint32_t tree_minimum_points_stem{20};
		double   tree_axis_max_vertical_deviation{25.0};
		double   tree_height_distance_from_axis{1.5};
		double   tree_resolution_height{0.3};
	};
} // namespace lib3dfin
