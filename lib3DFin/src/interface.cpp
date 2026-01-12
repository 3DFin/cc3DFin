// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright 2023-2025 Carlos Cabo <carloscabo@uniovi.es>

#include "interface.hpp"

#include "ground.hpp"
#include "individualize.hpp"
#include "peeling.hpp"
#include "section.hpp"
#include "types.hpp"
#include "voxel.hpp"

#ifdef TDFIN_USES_OPENXLSX
#include "xlsx.hpp"
#endif

// stdlib
#include <chrono>

namespace lib3dfin
{
	using TDFResult = std::tuple<std::vector<int32_t>, std::vector<double>, lib3dfin::TreeData>;

	TDFResult process(const float* cloud_data, const double* z0_sf, size_t num_points, const Params& params, std::shared_ptr<spdlog::logger> logger, const std::optional<fs::path>& output_basepath)
	{

		ProjectMeta project_meta;
		if (logger)
		{
			spdlog::set_default_logger(logger);
		}

		const auto start_total = std::chrono::steady_clock::now();
		spdlog::info("Starting 3DFin computation... ");

		// Convert point cloud to our datastructure (it uses double precision)
		PointCloud3 point_cloud(num_points, 3);
		for (size_t i = 0; i < num_points; ++i)
		{
			point_cloud.row(i) = Vec3(cloud_data[i * 3], cloud_data[i * 3 + 1], cloud_data[i * 3 + 2]);
		}

		Eigen::VectorXd z0;

		// Compute normalization if needed
		if (params.compute_height_normalization || !z0_sf)
		{
			if (!params.compute_height_normalization && !z0_sf)
			{
				spdlog::warn("Input height normalization is null, force computation of height normalization");
			}

			auto [voxelated_ground, _] = voxelize(point_cloud, 1.0, 2000, true);
			project_meta.num_points    = point_cloud.rows();
			project_meta.area_m2       = voxelated_ground.rows();

			spdlog::info("This cloud has {0:.2f} million points, its area is {1:}", project_meta.num_points / 1'000'000.0, project_meta.area_m2);
			HeightNormalization height_normalizer(point_cloud, HeightNormalization::Parameters::FromGlobalConfig(params));
			z0 = height_normalizer.normalize();
		}
		else
		{
			spdlog::info("Using provided height normalization");
			z0 = Eigen::Map<const Eigen::VectorXd>(z0_sf, num_points);

			ArrayMask pseudo_ground_mask = (z0.array() < 0.5);

			auto        pseudo_ground_point_count = pseudo_ground_mask.count();
			PointCloud3 pseudo_ground_cloud(pseudo_ground_point_count, 3);

			Eigen::Index filtered_point_id = 0;
			for (Eigen::Index point_id = 0; point_id < z0.size(); ++point_id)
			{
				if (pseudo_ground_mask(point_id))
				{
					pseudo_ground_cloud.row(filtered_point_id++) = point_cloud.row(point_id);
				}
			}

			auto [voxelated_ground, _] = voxelize(pseudo_ground_cloud, 1.0, 2000, true);
			project_meta.num_points    = point_cloud.rows();
			project_meta.area_m2       = voxelated_ground.rows();

			spdlog::info("This cloud has {0:.2f} million points, its area is {1:}", project_meta.num_points / 1'000'000.0, project_meta.area_m2);
		}

		TreePeeler stripe_peeler(point_cloud, TreePeeler::Parameters::StripeFromGlobalConfig(params));

		Stripe stripe(params.stripe_lower_limit, params.stripe_upper_limit);
		stripe.cluster_indicator = TreePeeler::filterInitialStripe(z0, params.stripe_lower_limit, params.stripe_upper_limit);

		// Beware the side effect on indicator
		stripe_peeler.peel(stripe.cluster_indicator);

		TreeIndividualizer tree_individualizer(point_cloud, stripe, z0, TreeIndividualizer::Parameters::FromGlobalConfig(params));
		auto               tree_data = tree_individualizer.individualize();

		// minimum_height, maximum_height + section_width
		auto stem_indicator = TreePeeler::filterInitialStripe(z0, tree_data.axis_distance, params.stem_search_diameter / 2.0, params.stem_minimum_height, params.stem_maximum_height + params.stem_section_thickness);

		// TODO: verticality could change at this point
		// use params.verticality_scale_stem;
		// and params.verticality_thresh_stem;

		// Beware the side effect on indicator
		stripe_peeler.peel(stem_indicator);

		ArrayClusterIndicator sections_indicator = (stem_indicator > -1).select(tree_data.cluster_indicator, -1);
		SectionExtractor      section_extractor(point_cloud, sections_indicator, z0, tree_data, SectionExtractor::Parameters::FromGlobalConfig(params));

		// TODO: beware side effect on tree_data
		section_extractor.extract();
		std::vector<int32_t> stem_indicator_vector(stripe.cluster_indicator.data(), stripe.cluster_indicator.data() + stripe.cluster_indicator.size());
		std::vector<double>  z0_vector(z0.data(), z0.data() + z0.size());

		// TODO: use the future draw interface here.
		// TODO:
		// TODO catch xlsx exceptions
#ifdef TDFIN_USES_OPENXLSX
		if (output_basepath.has_value())
		{
			export_xlsx(tree_data, output_basepath.value().string() + ".xlsx", project_meta);
		}
#endif

		const auto drawing    = std::chrono::high_resolution_clock::now();
		const auto stop_total = std::chrono::steady_clock::now();
		spdlog::info("End of 3DFin computation, total time: {0:.2f} s", std::chrono::duration_cast<std::chrono::milliseconds>(stop_total - start_total).count() / 1000.0);
		return std::make_tuple(std::move(stem_indicator_vector), std::move(z0_vector), std::move(tree_data));
	}

} // namespace lib3dfin
