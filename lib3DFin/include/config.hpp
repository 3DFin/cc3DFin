#pragma once

// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright 2023-2025 Carlos Cabo <carloscabo@uniovi.es>

#include <cstdint>

namespace lib3dfin
{

	// WIP. comment show original variable names (in dendromatics) if we changed the name
	struct params
	{
		double stripe_upper_limit           = 3.5; // upper_limit
		double stripe_lower_limit           = 0.7; // lower_limit
		double peeling_number_of_iterations = 3.5; // number_of_iterations

		// advanced
		double stem_maximum_diameter = 1.0; // maximum_diameter
		double stem_search_diameter  = 2.0;
		double minimum_height        = 0.3;
		double maximum_height        = 25.0;
		double section_length        = 0.2;  // section_len
		double section_width         = 0.05; // section_wid

		// expert
		double   res_xy_stripe                = 0.02;
		double   res_z_stripe                 = 0.02;
		uint32_t number_of_points_stripe      = 1000; // number_of_points
		double   verticality_scale_stripe     = 0.1;
		double   verticality_threshold_stripe = 0.7; // verticality_thresh_stripe
		double   height_range_stripe          = 0.7; // height_range

		double   res_xy_stem             = 0.035; // res_xy
		double   res_z_stem              = 0.035; // res_z
		uint32_t minimum_points          = 20;    // TODO minimum_points_stem ?
		double   verticality_scale_stem  = 0.1;   // verticality_thresh_stems
		double   verticality_thresh_stem = 0.7;
		double   maximum_distance        = 15.0; // maximum_d
		double   distance_from_axis      = 1.5;  // distance_to_axis

		double res_heights               = 0.3;
		double maximum_deviation_heights = 25.0; // maximum_dev

		uint32_t number_points_section        = 80;
		double   diameter_proportion          = 0.5;
		double   minimum_diameter             = 0.09;
		uint32_t inner_circle_point_threshold = 5;    // point_threshold
		double   circle_point_distance        = 0.02; // point_distance
		uint32_t number_sectors               = 16;
		uint32_t minimum_number_sectors       = 9; // m_number_sectors
		double   circle_width                 = 0.02;

		// Height Normalization
		double   cloth_resolution       = 0.45; // basic / res_cloth // changed from 0.7 to 0.45
		double   denoise_resolution     = 0.15; // res_ground
		uint32_t denoise_minimum_points = 2;    // minimum_points_ground

		// draw
		double draw_circle_sampling     = 200;  // circa;
		double draw_axis_point_interval = 0.01; // p_interval
		double draw_axis_downstep       = 0.5;  // axis_upstep
		double draw_axis_upstep         = 10.0; // axis_downstep

		// miscs
		// z0_name; z0 should be computed externally.
		bool do_normalize = true;
		bool do_denoise     = true;
		bool export_txt   = true;
	};
} // namespace lib3dfin
