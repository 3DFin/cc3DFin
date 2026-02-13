#pragma once

// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright 2023-2025 Carlos Cabo <carloscabo@uniovi.es>

#include <cstdint>

namespace lib3dfin
{

	/// Parameter sctruct.
	/// This is not aligned on purpose
	// NOLINTBEGIN
	struct Params
	{
		// Global
		bool compute_height_normalization{true};

		// Ground Parameters (ground.hpp)
		double   cloth_resolution{0.45};
		bool     denoise_point_cloud{false};
		double   denoise_resolution{0.15};
		uint32_t denoise_minimum_points{2};
		double   dtm_smooth_laplacian_lambda{0.125};
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

		// Section extraction: stem filtering / peeling
		double   stem_search_diameter{2.0};
		double   stem_minimum_height{0.3};
		double   stem_maximum_height{25.0};
		double   verticality_radius_stem{0.1};
		double   verticality_threshold_stem{0.7};
		double   stem_section_thickness{0.05};
		double   stem_section_circle_width{0.02};
		double   stem_section_interval{0.2};
		double   stem_section_clustering_distance{0.02};
		double   stem_section_minimum_diameter{0.09};
		double   stem_section_maximum_diameter{1.0};
		uint32_t stem_section_inner_point_threshold{5};
		uint32_t stem_section_min_points{80};
		double   stem_section_diameter_proportion{0.5};
		uint32_t stem_section_sector_count{16};
		uint32_t stem_section_min_occupied_sectors{9};

		// double   outlier_probability_threshold{0.3}; // TODO: new, not mapped to in the GUI
		// TODO Drawing
	};
	// NOLINTEND
} // namespace lib3dfin
