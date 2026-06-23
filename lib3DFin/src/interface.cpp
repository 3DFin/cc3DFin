// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright 2023-2026 Carlos Cabo <carloscabo@uniovi.es>

#include "interface.hpp"

#include "ground.hpp"
#include "individualize.hpp"
#include "localization.hpp"
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
	TDFProcessing::TDFProcessing(const double* cloud_data, const double* z0_sf, size_t num_points)
	{
		point_cloud_ = Eigen::Map<const PointCloud3>(cloud_data, num_points, 3);
		z0_          = Eigen::Map<const Eigen::VectorXd>(z0_sf, num_points);
	}

	TDFProcessing::TDFProcessing(const double* cloud_data, size_t num_points)
	{
		point_cloud_ = Eigen::Map<const PointCloud3>(cloud_data, num_points, 3);
	}

	Status TDFProcessing::process()
	{
		// Create the "Global" taskflow executor
		tf::Executor executor;
		const auto   start_total = std::chrono::steady_clock::now();
		spdlog::info("Starting 3DFin computation...");

		spdlog::info("Analyzing cloud size...");
		// We compute normalization if needed else we simply map the memory

		if (params_.compute_height_normalization || z0_.size() == 0)
		{
			if (!params_.compute_height_normalization && z0_.size() == 0)
			{

				spdlog::warn("Input height normalization is empty, force computation of height normalization");
			}

			auto [voxelated_ground, _] = voxelize(point_cloud_, 1.0, 2000, executor, true);
			project_meta_.num_points   = point_cloud_.rows();
			project_meta_.area_m2      = voxelated_ground.rows();

			spdlog::info("This cloud has {0:.2f} million points, its area is {1:} m^2", project_meta_.num_points / 1'000'000.0, project_meta_.area_m2);
			HeightNormalization height_normalizer(point_cloud_, params_.ground, executor);

			try
			{
				z0_ = height_normalizer.normalize();
			}
			catch (std::exception& e)
			{
				spdlog::error("Error in normalize: {}", e.what());
				return Status::DTMTooSmall;
			}

			const auto [warning, area_discrepancy] = HeightNormalization::checkHeightNormDiscrepancy(point_cloud_, z0_, project_meta_.area_m2, executor);
			if (warning)
			{
				spdlog::warn("[HeighNorm] Warning: 3DFin has detected a potential error in the terrain modelling.\n"
				             "  This usually happens when the \"cloth resolution\" parameter didn\'t fit well the terrain.\n"
				             "  Learn more about this here https://github.com/3DFin/3DFin_Tutorial");
			}
			spdlog::info("[HeighNorm] Area discrepancy {0:.2f} %", area_discrepancy);
			dtm_ = height_normalizer.exportDTM();
		}
		else
		{
			spdlog::info("Using provided height normalization");
			assert(z0_.size() == point_cloud_.rows());

			const ArrayMask pseudo_ground_mask = (z0_.array() < 0.5);

			const auto  pseudo_ground_point_count = pseudo_ground_mask.count();
			PointCloud3 pseudo_ground_cloud(pseudo_ground_point_count, 3);

			Eigen::Index filtered_point_id = 0;
			for (Eigen::Index point_id = 0; point_id < z0_.size(); ++point_id)
			{
				if (pseudo_ground_mask(point_id))
				{
					pseudo_ground_cloud.row(filtered_point_id++) = point_cloud_.row(point_id);
				}
			}

			const auto [voxelated_ground, _] = voxelize(pseudo_ground_cloud, 1.0, 2000, executor, true);
			project_meta_.num_points         = point_cloud_.rows();
			project_meta_.area_m2            = voxelated_ground.rows();

			spdlog::info("This cloud has {0:.2f} million points, its area is {1:} m^2", project_meta_.num_points / 1'000'000.0, project_meta_.area_m2);
		}

		TreePeeler stripe_peeler = TreePeeler::StripePeeler(
		    point_cloud_,
		    z0_,
		    params_.stripe_peeling,
		    executor);

		try
		{
			stripe_                   = Stripe(params_.stripe_peeling.stripe_lower_limit, params_.stripe_peeling.stripe_upper_limit);
			stripe_.cluster_indicator = stripe_peeler.peel();
		}
		catch (std::exception& e)
		{
			spdlog::error("Error in peeling: {}", e.what());
			return Status::NoValidClusters;
		}

		TreeIndividualizer tree_individualizer(point_cloud_, stripe_, z0_, params_.tree, executor);
		tree_data_ = tree_individualizer.individualize();

		// Create stem peeling params (same as stripe peeling but with stem verticality params)
		StripePeelingParams stem_peeling_params = params_.stripe_peeling;
		stem_peeling_params.stripe_lower_limit  = params_.stem.stem_minimum_height;
		stem_peeling_params.stripe_upper_limit =
		    params_.stem.stem_maximum_height + params_.stem.stem_section_thickness,
		stem_peeling_params.verticality_nn_scale  = params_.stem.verticality_radius_stem;
		stem_peeling_params.verticality_threshold = params_.stem.verticality_threshold_stem;

		TreePeeler stem_peeler = TreePeeler::StemPeeler(
		    point_cloud_,
		    z0_,
		    stem_peeling_params,
		    executor,
		    params_.stem.stem_search_diameter / 2.0,
		    tree_data_.axis_distance);
		const auto stem_indicator = stem_peeler.peel();

		const ArrayClusterIndicator sections_indicator = (stem_indicator > -1).select(tree_data_.tree_cluster_indicator, -1);
		SectionExtractor            section_extractor(point_cloud_, sections_indicator, z0_, params_.stem, executor);
		LocalizationExtractor       localization_extractor(params_.stem, executor);

		section_extractor.extract(tree_data_);
		localization_extractor.extract(tree_data_);

		const auto stop_total = std::chrono::steady_clock::now();
		spdlog::info("End of 3DFin computation. Found {0:} Trees. Total time: {1:.2f} s", tree_data_.tree_descriptors.size(), std::chrono::duration_cast<std::chrono::milliseconds>(stop_total - start_total).count() / 1000.0);
		return Status::Success;
	}

	void TDFProcessing::exportTabularData() const
	{
#ifdef TDFIN_USES_OPENXLSX
		export_xlsx(tree_data_, output_basepath_.string() + ".xlsx", project_meta_);
#endif
		// else do nothing for now...
	}
} // namespace lib3dfin
