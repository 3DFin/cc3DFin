#pragma once

// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright 2023-2026 Carlos Cabo <carloscabo@uniovi.es>

#include <cstdint>

namespace lib3dfin
{

	// NOLINTBEGIN
	struct GroundParams
	{
		double   cloth_resolution{0.45};
		bool     denoise_point_cloud{false};
		double   denoise_resolution{0.15};
		uint32_t denoise_minimum_points{2};
		double   dtm_smooth_laplacian_lambda{0.125};
		bool     clean_dtm{true};
	};

	struct StripePeelingParams
	{
		double   stripe_lower_limit{0.7};
		double   stripe_upper_limit{3.5};
		double   verticality_nn_scale{0.1};
		double   verticality_threshold{0.7};
		uint32_t num_voxels_threshold{1000};
		double   resolution_xy{0.02};
		double   resolution_z{0.02};
		uint32_t num_iterations{2};
	};

	struct TreeIndividualizerParams
	{
		double   resolution_xy{0.035};
		double   resolution_z{0.035};
		double   height_range{0.7};
		double   maximum_dist_axis{15.0};
		uint32_t minimum_points_stem{20};
		double   axis_maximum_vertical_deviation{25.0};
		double   height_distance_from_axis{1.5};
		double   resolution_height{0.3};
	};

	struct StemSectionParams
	{
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
		double   outlier_probability_threshold{0.3};
		double   DBH{1.3};
	};

	/// Parameter struct.
	/// This is not aligned on purpose
	struct Params
	{
		// Global
		bool compute_height_normalization{true};

		// Ground Parameters
		GroundParams ground;

		// Stripe Peeling
		StripePeelingParams stripe_peeling;

		// Tree individualizer
		TreeIndividualizerParams tree;

		// Section extraction: stem filtering / peeling
		StemSectionParams stem;
	};
	// NOLINTEND
} // namespace lib3dfin
