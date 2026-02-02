// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright 2023-2025 Carlos Cabo <carloscabo@uniovi.es>

#include "interface.hpp"

#include "ground.hpp"
#include "individualize.hpp"
#include "peeling.hpp"
#include "section.hpp"
#include "types.hpp"
#include "voxel.hpp"

#include <taskflow/taskflow.hpp>

#ifdef TDFIN_USES_OPENXLSX
#include "xlsx.hpp"
#endif

// stdlib
#include <chrono>

namespace lib3dfin
{
	using TDFResult = std::tuple<std::vector<int32_t>, std::vector<double>, lib3dfin::TreeData>;

	TDFResult process(const float* cloud_data, const double* z0_sf, size_t num_points, const Params& params, const std::optional<fs::path>& output_basepath, const std::optional<GlobalShift>& global_shift)
	{

		ProjectMeta project_meta;

		tf::Executor executor;

		const auto start_total = std::chrono::steady_clock::now();
		spdlog::info("Starting 3DFin computation... ");

		// Convert the point cloud to our datastructure (it uses double precision)
		PointCloud3 point_cloud(num_points, 3);
		for (size_t i = 0; i < num_points; ++i)
		{
			point_cloud.row(i) = Vec3(cloud_data[i * 3], cloud_data[i * 3 + 1], cloud_data[i * 3 + 2]);
		}

		spdlog::info("Analyzing cloud size...");
		// We do the same with z0
		Eigen::VectorXd z0;
		// We compute normalization if needed else we simply map the memory
		if (params.compute_height_normalization || !z0_sf)
		{
			if (!params.compute_height_normalization && !z0_sf)
			{
				spdlog::warn("Input height normalization is null, force computation of height normalization");
			}

			auto [voxelated_ground, _] = voxelize(point_cloud, 1.0, 2000, executor, true);
			project_meta.num_points    = point_cloud.rows();
			project_meta.area_m2       = voxelated_ground.rows();

			spdlog::info("This cloud has {0:.2f} million points, its area is {1:} m^2", project_meta.num_points / 1'000'000.0, project_meta.area_m2);
			HeightNormalization height_normalizer(point_cloud, HeightNormalization::Parameters::FromGlobalConfig(params), executor);
			z0                               = height_normalizer.normalize();
			auto [warning, area_discrepancy] = HeightNormalization::checkHeightNormDiscrepancy(point_cloud, z0, project_meta.area_m2, executor);
			if (warning)
			{
				spdlog::warn("[HeighNorm] Warning: 3DFin has detected a potential error in the terrain modelling.\n"
				             "  This usually happens when the \"cloth resolution\" parameter didn\'t fit well the terrain.\n"
				             "  Learn more about this here https://github.com/3DFin/3DFin_Tutorial");
			}
			spdlog::info("[HeighNorm] Area discrepancy {0:.2f} m^2", area_discrepancy);
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

			auto [voxelated_ground, _] = voxelize(pseudo_ground_cloud, 1.0, 2000, executor, true);
			project_meta.num_points    = point_cloud.rows();
			project_meta.area_m2       = voxelated_ground.rows();

			spdlog::info("This cloud has {0:.2f} million points, its area is {1:} m^2", project_meta.num_points / 1'000'000.0, project_meta.area_m2);
		}
		Stripe                           stripe(params.stripe_lower_limit, params.stripe_upper_limit);
		std::unique_ptr<FilterPredicate> stripe_predicate(new StripeFilterPredicate(params.stripe_lower_limit, params.stripe_upper_limit));
		TreePeeler                       stripe_peeler(point_cloud, z0, std::move(stripe_predicate), TreePeeler::Parameters::StripeFromGlobalConfig(params), executor);
		stripe.cluster_indicator = stripe_peeler.peel();

		TreeIndividualizer tree_individualizer(point_cloud, stripe, z0, TreeIndividualizer::Parameters::FromGlobalConfig(params), executor);
		auto               tree_data = tree_individualizer.individualize();

		std::unique_ptr<FilterPredicate> stem_predicate(new StemFilterPredicate(params.stem_minimum_height, params.stem_maximum_height + params.stem_section_thickness, params.stem_search_diameter / 2.0, tree_data.axis_distance));
		TreePeeler                       stem_peeler(point_cloud, z0, std::move(stem_predicate), TreePeeler::Parameters::StemFromGlobalConfig(params), executor);
		auto                             stem_indicator = stem_peeler.peel();

		ArrayClusterIndicator sections_indicator = (stem_indicator > -1).select(tree_data.cluster_indicator, -1);
		SectionExtractor      section_extractor(point_cloud, sections_indicator, z0, tree_data, SectionExtractor::Parameters::FromGlobalConfig(params));

		// TODO: try to eliminate side effect on tree_data
		section_extractor.extract();
		std::vector<int32_t> stem_indicator_vector(stripe.cluster_indicator.data(), stripe.cluster_indicator.data() + stripe.cluster_indicator.size());
		std::vector<double>  z0_vector(z0.data(), z0.data() + z0.size());

		// TODO: use the future draw interface here.
		// TODO catch xlsx exceptions
#ifdef TDFIN_USES_OPENXLSX
		if (output_basepath.has_value())
		{
			if (global_shift)
			{
				// auto unwrapper_gs  = global_shift.value();
				project_meta.shift = {global_shift->x_shift, global_shift->y_shift, global_shift->z_shift};
				project_meta.scale = {global_shift->scale};
			}
			export_xlsx(tree_data, output_basepath.value().string() + ".xlsx", project_meta);
		}
#endif

		const auto stop_total = std::chrono::steady_clock::now();
		spdlog::info("End of 3DFin computation. Found {0:} Trees. Total time: {1:.2f} s", tree_data.tree_descriptors.size(), std::chrono::duration_cast<std::chrono::milliseconds>(stop_total - start_total).count() / 1000.0);
		return std::make_tuple(std::move(stem_indicator_vector), std::move(z0_vector), std::move(tree_data));
	}

} // namespace lib3dfin
