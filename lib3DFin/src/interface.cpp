// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright 2023-2025 Carlos Cabo <carloscabo@uniovi.es>

#include "interface.hpp"

#include "ground.hpp"
#include "individualize.hpp"
#include "peeling.hpp"
#include "section.hpp"
#include "types.hpp"

namespace lib3dfin
{

	std::tuple<std::vector<int32_t>, std::vector<double>, TreeData, std::vector<CircleSections>> process(const float* cloud_data, size_t num_points)
	{

		// Convert point cloud to double
		PointCloud3 point_cloud(num_points, 3);
		for (size_t i = 0; i < num_points; ++i)
		{
			point_cloud.row(i) = Vec3(cloud_data[i * 3], cloud_data[i * 3 + 1], cloud_data[i * 3 + 2]);
		}

		// TODO: check height normalization calculation, this doe not seams to give the same
		//  extent as its use in python
		HeightNormalization height_normalizer(point_cloud, HeightNormalization::Parameters());
		const auto          z0 = height_normalizer.normalize();
		TreePeeler          stripe_peeler(point_cloud, TreePeeler::Parameters());
		Stripe              stripe(0.7, 3.5);
		stripe.cluster_indicator = TreePeeler::filterInitialStripe(z0, 0.7, 3.5);

		// side effect on indicator
		stripe_peeler.peel(stripe.cluster_indicator);

		TreeIndividualizer tree_individualizer(point_cloud, stripe, z0, TreeIndividualizer::Parameters());
		auto               tree_data = tree_individualizer.individualize();

		// stem_search_diameter / 2, minimum_height, maximum_height + section_width
		auto stem_indicator = TreePeeler::filterInitialStripe(z0, tree_data.axis_distance, 2.0 / 2, 0.3, 25 + 0.05);
		// TODO: verticality could change at this point
		// double verticality_scale_stem  = 0.1;  // verticality_thresh_stems
		// double verticality_thresh_stem = 0.7;

		//TODO Beware Side effect on indicator
		stripe_peeler.peel(stem_indicator);

		ArrayClusterIndicator sections_indicator = (stem_indicator > -1).select(tree_data.cluster_indicator, -1);
		SectionExtractor      section_extractor(point_cloud, sections_indicator, z0, tree_data, SectionExtractor::Parameters());

		const auto circle_sections = section_extractor.extract();

		std::vector<int32_t> stem_indicator_vector(stripe.cluster_indicator.data(), stripe.cluster_indicator.data() + stripe.cluster_indicator.size());

		std::vector<double> z0_vector(z0.data(), z0.data() + z0.size());

		return std::make_tuple(std::move(stem_indicator_vector), std::move(z0_vector), std::move(tree_data), std::move(circle_sections));
	}

} // namespace lib3dfin
