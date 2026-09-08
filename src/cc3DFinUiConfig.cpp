// ##########################################################################
// #                                                                        #
// #                CLOUDCOMPARE PLUGIN: 3DFin                              #
// #                                                                        #
// #  This program is free software; you can redistribute it and/or modify  #
// #  it under the terms of the GNU General Public License as published by  #
// #  the Free Software Foundation; version 2 of the License.               #
// #                                                                        #
// #  This program is distributed in the hope that it will be useful,       #
// #  but WITHOUT ANY WARRANTY; without even the implied warranty of        #
// #  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         #
// #  GNU General Public License for more details.                          #
// #                                                                        #
// #                     COPYRIGHT: Carlos Cabo                             #
// #                                                                        #
// ##########################################################################

#include "cc3DFinUiConfig.h"

// StdLib
#include <cstddef>

// lib3dfin
#include <lib3DFin/config.hpp>

namespace tdf
{

	// Basic fields (always visible in UI)
	static const std::vector<UiField> BASIC_FIELDS = {
	    {"z0_name",
	     "Normalized height field name",
	     "Name of the Z0 field in the point cloud. \n"
	     "If the normalized heights are stored in the Z coordinate "
	     "of the point cloud, then: Z0 field name = \"z\" (lowercase).",
	     "Z0",
	     QVariant(),
	     QVariant(),
	     QVariant(),
	     false,
	     "basic"},
	    {"stripe_upper_limit",
	     "Stripe upper Limit",
	     "Upper (vertical) limit of the stripe where it should be reasonable "
	     "to find stems with minimum presence of shrubs or branches. \n"
	     "Reasonable values are 2-5 meters.",
	     "meters",
	     3.5,
	     0.0,
	     QVariant(),
	     true,
	     "basic"},
	    {"stripe_lower_limit",
	     "Stripe lower Limit",
	     "Lower (vertical) limit of the stripe where it should be reasonable "
	     "to find stems with minimum presence of shrubs or branches. \n"
	     "Reasonable values are 0.3-1.3 meters.",
	     "meters",
	     0.7,
	     0.0,
	     QVariant(),
	     true,
	     "basic"},
	    {"stripe_peeling_num_iterations",
	     "Pruning intensity",
	     "Number of iterations of \"pruning\" during stem identification. \n"
	     "Values between 1 (slight stem peeling/cleaning) "
	     "and 5 (extreme branch peeling/cleaning).",
	     "0-5",
	     2,
	     0,
	     5,
	     true,
	     "basic"},
	    {"cloth_resolution",
	     "Cloth resolution",
	     "Initial cloth grid resolution to generate the DTM that will be used to compute normalized heights.",
	     "meters",
	     0.45,
	     0.0 + std::numeric_limits<double>::epsilon(),
	     QVariant(),
	     true,
	     "basic"}};

	// Advanced fields
	static const std::vector<UiField> ADVANCED_FIELDS = {
	    {"stem_section_maximum_diameter", "Expected maximum diameter", "Maximum diameter expected for any stem.", "meters", 1.0, 0.0, QVariant(), true, "advanced"},
	    {"stem_search_diameter", "Stem search diameter", "Points within this distance from tree axes will be considered as potential stem points.\nReasonable values are 'Maximum diameter'<2 meters (exceptionally greater than 2: very large diameters and/or intricate stems).", "meters", 2.0, 0.0 + std::numeric_limits<double>::epsilon(), QVariant(), true, "advanced"},
	    {"stem_minimum_height", "Lowest section", "Lowest height at which stem diameter will be computed.", "meters", 0.3, 0.0 + std::numeric_limits<double>::epsilon(), QVariant(), true, "advanced"},
	    {"stem_maximum_height", "Highest section", "Highest height at which stem diameter will be computed.", "meters", 25.0, 0.0 + std::numeric_limits<double>::epsilon(), QVariant(), true, "advanced"},
	    {"stem_section_interval", "Distance between sections", "Height of the sections (z length). Diameters will then be computed for every section.", "meters", 0.2, 0.0 + std::numeric_limits<double>::epsilon(), QVariant(), true, "advanced"},
	    {"stem_section_thickness", "Section width", "Sections are this wide. This means that points within this distance (vertical) are considered during circle fitting and diameter computation.", "meters", 0.05, 0.0 + std::numeric_limits<double>::epsilon(), QVariant(), true, "advanced"}};

	// Expert fields
	static const std::vector<UiField> EXPERT_FIELDS = {
	    {"stripe_peeling_resolution_xy", "(x, y) voxel resolution", "(x, y) voxel resolution during stem extraction.", "meters", 0.02, 0.0 + std::numeric_limits<double>::epsilon(), QVariant(), true, "expert"},
	    {"stripe_peeling_resolution_z", "(z) voxel resolution", "(z) voxel resolution during stem extraction.", "meters", 0.02, 0.0 + std::numeric_limits<double>::epsilon(), QVariant(), true, "expert"},
	    {"stripe_peeling_voxels_threshold", "Number of points", "minimum number of points (voxels) per stem within the stripe (DBSCAN clustering). Reasonable values are between 500 and 3000.", "", 1000, 1, QVariant(), true, "expert"},
	    {"verticality_radius_stripe", "Vicinity radius (verticality computation)", "Vicinity radius for PCA during stem identification.", "meters", 0.1, 0.0 + std::numeric_limits<double>::epsilon(), QVariant(), true, "expert"},
	    {"verticality_threshold_stripe", "Verticality threshold", "Verticality threshold during stem identification.\nVerticality is defined as (1 - sin(V)), being V the vertical angle of the normal vector, measured from the horizontal. Note that it does not grow linearly.", "(0, 1)", 0.7, 0.0 + std::numeric_limits<double>::epsilon(), 1.0 - std::numeric_limits<double>::epsilon(), true, "expert"},
	    {"tree_height_range", "Vertical Range", "Proportion (0: none - 1: all) of the vertical range of the stripe that points need to extend through to be valid stems.", "[0, 1]", 0.7, 0.0, 1.0, true, "expert"},
	    {"tree_resolution_xy", "(x, y) voxel resolution", "(x, y) voxel resolution during tree extraction.", "meters", 0.035, 0.0 + std::numeric_limits<double>::epsilon(), QVariant(), true, "expert"},
	    {"tree_resolution_z", "(z) voxel resolution", "(z) voxel resolution during tree extraction.", "meters", 0.035, 0.0 + std::numeric_limits<double>::epsilon(), QVariant(), true, "expert"},
	    {"tree_minimum_points_stem", "Minimum points", "Minimum number of points (voxels) within a stripe to consider it as a potential tree during tree individualization.", "", 20, 1, QVariant(), true, "expert"},
	    {"verticality_radius_stem", "Vicinity radius (verticality computation)", "Vicinity radius for PCA during tree individualization.", "meters", 0.1, 0.0 + std::numeric_limits<double>::epsilon(), QVariant(), true, "expert"},
	    {"verticality_threshold_stem", "Verticality threshold", "Verticality threshold during stem extraction.\nVerticality is defined as (1 - sin(V)), being V the vertical angle of the normal vector, measured from the horizontal.\nNote that it does not grow linearly.", "(0, 1)", 0.7, 0.0 + std::numeric_limits<double>::epsilon(), 1.0 - std::numeric_limits<double>::epsilon(), true, "expert"},
	    {"tree_dist_axis_threshold", "Maximum distance to tree axis", "Points that are closer than this distance to an axis are assigned to that axis during tree individualization process.", "meters", 15.0, 0.0 + std::numeric_limits<double>::epsilon(), QVariant(), true, "expert"},
	    {"tree_height_distance_from_axis", "Distance from axis", "Maximum distance from tree axis at which points will be considered while computing tree height.\nPoints too far away from the tree axis might not be representative of actual tree height.", "meters", 1.5, 0.0 + std::numeric_limits<double>::epsilon(), QVariant(), true, "expert"},
	    {"tree_resolution_height", "Voxel resolution for height computation", "(x, y, z) voxel resolution during tree height computation.", "meters", 0.3, 0.0 + std::numeric_limits<double>::epsilon(), QVariant(), true, "expert"},
	    {"tree_axis_max_vertical_deviation", "Maximum vertical deviation from axis", "Maximum degree of vertical deviation from the axis for a tree height to be considered as valid.", "degrees", 25.0, 0.0 + std::numeric_limits<double>::epsilon(), QVariant(), true, "expert"},
	    {"stem_section_min_points", "Points within section", "Minimum number of points in a section to be considered as valid.", "", 80, 1, QVariant(), true, "expert"},
	    {"stem_section_diameter_proportion", "Inner/outer circle proportion", "Proportion, regarding the circumference fit by fit_circle, that the inner circumference diameter will have as length.", "[0, 1]", 0.5, 0.0, 1.0, true, "expert"},
	    {"stem_section_minimum_diameter", "Minimum expected diameter", "Minimum diameter expected for any section during circle fitting.", "meters", 0.09, 0.0 + std::numeric_limits<double>::epsilon(), QVariant(), true, "expert"},
	    {"stem_section_inner_point_threshold", "Points within inner circle", "Maximum number of points inside the inner circle to consider the fitting as OK.", "", 5, 1, QVariant(), true, "expert"},
	    {"stem_section_clustering_distance", "Maximum point distance", "Maximum distance among points to be considered within the same cluster during circle fitting.", "meters", 0.02, 0.0 + std::numeric_limits<double>::epsilon(), QVariant(), true, "expert"},
	    {"stem_section_sector_count", "Number of sectors", "Number of sectors in which the circumference will be divided into.", "", 16, 1, QVariant(), true, "expert"},
	    {"stem_section_min_occupied_sectors", "Number of occupied sectors", "Minimum number of sectors that must be occupied.", "", 9, 1, QVariant(), true, "expert"},
	    {"stem_section_circle_width", "Circle width", "Width, in meters, around the circumference to look for points.", "meters", 0.02, 0.0 + std::numeric_limits<double>::epsilon(), QVariant(), true, "expert"},
	    {"circa", "N of points to draw each circle", "Number of points that will be used to draw the circles.", "", 200, 1, QVariant(), true, "expert"},
	    {"p_interval", "Interval at which points are drawn while drawing", "Distance at which points will be placed from one to another while drawing the axes point cloud.", "meters", 0.01, 0.0 + std::numeric_limits<double>::epsilon(), QVariant(), true, "expert"},
	    {"axis_downstep", "Axis downstep from stripe center", "From the stripe centroid, how much (downwards direction) will the drawn axes extend.\nBasically, this parameter controls from where will the axes be drawn.", "meters", 0.5, 0.0 + std::numeric_limits<double>::epsilon(), QVariant(), true, "expert"},
	    {"axis_upstep", "Axis upstep from stripe center", "From the stripe centroid, how much (upwards direction) will the drawn axes extend.\nBasically, this parameter controls how long will the drawn axes be.", "meters", 10.0, 0.0 + std::numeric_limits<double>::epsilon(), QVariant(), true, "expert"},
	    {"denoise_resolution", "(x, y, z) voxel resolution", "(x, y, z) voxel resolution during denoising.\nNote that the whole point cloud is voxelated.", "meters", 0.15, 0.0 + std::numeric_limits<double>::epsilon(), QVariant(), true, "expert"},
	    {"denoise_minimum_points", "Minimum number of points", "Clusters with size smaller than this value will be regarded as noise and thus eliminated.", "", 2, 1, QVariant(), true, "expert"},
	    {"dtm_smooth_laplacian_lambda", "DTM smoothing strength", "lambda parameter of laplacian smoothing applied to the DTM, should be approx. [0.1;0.2]", "", 0.125, 0.0 + std::numeric_limits<double>::epsilon(), 1.0, true, "expert"}};

	// Miscellaneous fields
	static const std::vector<UiField> MISC_FIELDS = {
	    {"compute_height_normalization", "Normalize point cloud", "If the point cloud is not height-normalized, a Digital Terrain Model (DTM) will be generated to compute normalized heights for all points.", "", QVariant(), QVariant(), QVariant(), true, "misc"},
	    {"denoise_point_cloud", "Clean noise on DTM", "If it is expected to be noise below ground level (or if you know that there is noise), a denoising step will be added before generating the Digital Terrain Model.", "", QVariant(), QVariant(), QVariant(), true, "misc"},
	    {"export_txt", "Format of output tabular data", "Outputs are gathered in a xlsx (Excel) file by default.\nSelecting \"TXT files\" will make the program output several txt files with the raw data, which may be more convenient for processing the data via scripting.", "", QVariant(), QVariant(), QVariant(), true, "misc"}};

	// Helper function to extract field names
	static std::vector<QString> getFieldNames(const std::vector<UiField>& fields)
	{
		std::vector<QString> names;
		names.reserve(fields.size());
		for (const auto& field : fields)
		{
			names.push_back(field.name);
		}
		return names;
	}

	// Field Groups Definition
	static const std::vector<FieldGroup> FIELD_GROUPS = {
	    {"basic", "Basic", getFieldNames(BASIC_FIELDS)},
	    {"advanced", "Advanced", getFieldNames(ADVANCED_FIELDS)},
	    {"expert", "Expert", getFieldNames(EXPERT_FIELDS)},
	    {"misc", "Settings", getFieldNames(MISC_FIELDS)}};

	const std::unordered_map<QString, UiField>& UiConfig::getAllFields()
	{
		static const std::unordered_map<QString, UiField> ALL_FIELDS = []()
		{
			std::unordered_map<QString, UiField> map;
			auto                                 addFields = [&map](const std::vector<UiField>& fields)
			{
				for (const auto& field : fields)
				{
					map[field.name] = field;
				}
			};
			addFields(BASIC_FIELDS);
			addFields(ADVANCED_FIELDS);
			addFields(EXPERT_FIELDS);
			addFields(MISC_FIELDS);
			return map;
		}();
		return ALL_FIELDS;
	}

	std::vector<UiField> UiConfig::getFieldsByGroup(const QString& group)
	{
		if (group == "basic")
		{
			return BASIC_FIELDS;
		}
		if (group == "advanced")
		{
			return ADVANCED_FIELDS;
		}
		if (group == "expert")
		{
			return EXPERT_FIELDS;
		}
		if (group == "misc")
		{
			return MISC_FIELDS;
		}
		return {};
	}

	const UiField* UiConfig::findField(const QString& name)
	{
		const auto& allFields = getAllFields();
		auto        it        = allFields.find(name);
		return (it != allFields.end()) ? &(it->second) : nullptr;
	}

	bool UiConfig::hasField(const QString& name)
	{
		return findField(name) != nullptr;
	}

	 // NOLINTBEGIN
	void UiConfig::populateFieldFromLib3DFin(UiField& field, const lib3dfin::Params& libParams)
	{
		// Map UI field names to Lib3DFin parameter values
		// if (field.name == "z0_name").. not available
		if (field.name == "stripe_upper_limit")
		{
			field.defaultValue = QVariant(libParams.stripe_peeling.stripe_upper_limit);
		}
		else if (field.name == "stripe_lower_limit")
		{
			field.defaultValue = QVariant(libParams.stripe_peeling.stripe_lower_limit);
		}
		else if (field.name == "stripe_peeling_num_iterations")
		{
			field.defaultValue = QVariant(libParams.stripe_peeling.num_iterations);
		}
		else if (field.name == "cloth_resolution")
		{
			field.defaultValue = QVariant(libParams.ground.cloth_resolution);
		}
		// Advanced fields
		else if (field.name == "stem_section_maximum_diameter")
		{
			field.defaultValue = QVariant(libParams.stem.stem_section_maximum_diameter);
		}
		else if (field.name == "stem_search_diameter")
		{
			field.defaultValue = QVariant(libParams.stem.stem_search_diameter);
		}
		else if (field.name == "stem_minimum_height")
		{
			field.defaultValue = QVariant(libParams.stem.stem_minimum_height);
		}
		else if (field.name == "stem_maximum_height")
		{
			field.defaultValue = QVariant(libParams.stem.stem_maximum_height);
		}
		else if (field.name == "stem_section_interval")
		{
			field.defaultValue = QVariant(libParams.stem.stem_section_interval);
		}
		else if (field.name == "stem_section_thickness")
		{
			field.defaultValue = QVariant(libParams.stem.stem_section_thickness);
		}
		// Expert fields
		else if (field.name == "stripe_peeling_resolution_xy")
		{
			field.defaultValue = QVariant(libParams.stripe_peeling.resolution_xy);
		}
		else if (field.name == "stripe_peeling_resolution_z")
		{
			field.defaultValue = QVariant(libParams.stripe_peeling.resolution_z);
		}
		else if (field.name == "stripe_peeling_voxels_threshold")
		{
			field.defaultValue = QVariant(libParams.stripe_peeling.num_voxels_threshold);
		}
		else if (field.name == "verticality_radius_stripe")
		{
			field.defaultValue = QVariant(libParams.stripe_peeling.verticality_nn_scale);
		}
		else if (field.name == "verticality_threshold_stripe")
		{
			field.defaultValue = QVariant(libParams.stripe_peeling.verticality_threshold);
		}
		else if (field.name == "tree_height_range")
		{
			field.defaultValue = QVariant(libParams.tree.height_range);
		}
		else if (field.name == "tree_resolution_xy")
		{
			field.defaultValue = QVariant(libParams.tree.resolution_xy);
		}
		else if (field.name == "tree_resolution_z")
		{
			field.defaultValue = QVariant(libParams.tree.resolution_z);
		}
		else if (field.name == "tree_minimum_points_stem")
		{
			field.defaultValue = QVariant(libParams.tree.minimum_points_stem);
		}
		else if (field.name == "verticality_radius_stem")
		{
			field.defaultValue = QVariant(libParams.stem.verticality_radius_stem);
		}
		else if (field.name == "verticality_threshold_stem")
		{
			field.defaultValue = QVariant(libParams.stem.verticality_threshold_stem);
		}
		else if (field.name == "tree_dist_axis_threshold")
		{
			field.defaultValue = QVariant(libParams.tree.maximum_dist_axis);
		}
		else if (field.name == "tree_height_distance_from_axis")
		{
			field.defaultValue = QVariant(libParams.tree.height_distance_from_axis);
		}
		else if (field.name == "tree_resolution_height")
		{
			field.defaultValue = QVariant(libParams.tree.resolution_height);
		}
		else if (field.name == "tree_axis_max_vertical_deviation")
		{
			field.defaultValue = QVariant(libParams.tree.axis_maximum_vertical_deviation);
		}
		else if (field.name == "stem_section_min_points")
		{
			field.defaultValue = QVariant(libParams.stem.stem_section_min_points);
		}
		else if (field.name == "stem_section_diameter_proportion")
		{
			field.defaultValue = QVariant(libParams.stem.stem_section_diameter_proportion);
		}
		else if (field.name == "stem_section_minimum_diameter")
		{
			field.defaultValue = QVariant(libParams.stem.stem_section_minimum_diameter);
		}
		else if (field.name == "stem_section_inner_point_threshold")
		{
			field.defaultValue = QVariant(libParams.stem.stem_section_inner_point_threshold);
		}
		else if (field.name == "stem_section_clustering_distance")
		{
			field.defaultValue = QVariant(libParams.stem.stem_section_clustering_distance);
		}
		else if (field.name == "stem_section_sector_count")
		{
			field.defaultValue = QVariant(libParams.stem.stem_section_sector_count);
		}
		else if (field.name == "stem_section_min_occupied_sectors")
		{
			field.defaultValue = QVariant(libParams.stem.stem_section_min_occupied_sectors);
		}
		else if (field.name == "stem_section_circle_width")
		{
			field.defaultValue = QVariant(libParams.stem.stem_section_circle_width);
		}
		// Misc fields
		else if (field.name == "compute_height_normalization")
		{
			field.defaultValue = QVariant(libParams.compute_height_normalization);
		}
		else if (field.name == "denoise_point_cloud")
		{
			field.defaultValue = QVariant(libParams.ground.denoise_point_cloud);
		}
		// Note: export_txt doesn't have a direct mapping in Lib3DFin Params
		// as it's a UI-specific setting for output format
	}
 	// NOLINTEND
	std::unordered_map<QString, UiField> UiConfig::getAllFieldsFromLib3DFin()
	{
		const lib3dfin::Params               libParams{};
		std::unordered_map<QString, UiField> fields;

		// Get all original fields
		const auto& allFields = getAllFields();

		// copies with lb3DFin values
		for (const auto& [name, field] : allFields)
		{
			UiField populatedField = field;
			populateFieldFromLib3DFin(populatedField, libParams);
			fields[name] = populatedField;
		}

		return fields;
	}

} // namespace tdf
