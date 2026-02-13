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

#include "cc3DFinConfig.h"

#include <cstddef>
#include <unordered_map>
#include <vector>

namespace tdf
{
	const std::vector<Field> basicFields = {
	    {"z0_name",
	     "Normalized height field name",
	     "Name of the Z0 field in the point cloud. \n"
	     "If the normalized heights are stored in the Z coordinate "
	     "of the point cloud, then: Z0 field name = \"z\" (lowercase).",
	     "Z0"},
	    {"stripe_upper_limit",
	     "Stripe upper Limit",
	     "Upper (vertical) limit of the stripe where it should be reasonable "
	     "to find stems with minimum presence of shrubs or branches. \n"
	     "Reasonable values are 2-5 meters.",
	     3.5,
	     "meters",
	     0.0},
	    {"stripe_lower_limit",
	     "Stripe lower Limit",
	     "Lower (vertical) limit of the stripe where it should be reasonable "
	     "to find stems with minimum presence of shrubs or branches. \n"
	     "Reasonable values are 0.3-1.3 meters.",
	     0.7,
	     "meters",
	     0.0},
	    {"stripe_peeling_num_iterations",
	     "Prunning intensity",
	     "Number of iterations of \"pruning\" during stem identification. \n"
	     "Values between 1 (slight stem peeling/cleaning) "
	     "and 5 (extreme branch peeling/cleaning).",
	     2,
	     "0-5",
	     0,
	     5},
	    {"cloth_resolution",
	     "Cloth resolution",
	     "Initial cloth grid resolution to generate the DTM that will be used to compute normalized heights.",
	     0.45,
	     "meters",
	     0.0 + std::numeric_limits<double>::epsilon()}};

	const std::vector<Field> advancedFields = {
	    {"stem_section_maximum_diameter",
	     "Expected maximum diameter",
	     "Maximum diameter expected for any stem.",
	     1.0,
	     "meters",
	     0.0},
	    {"stem_search_diameter",
	     "Stem search diameter",
	     "Points within this distance from tree axes will be considered "
	     "as potential stem points.\n"
	     "Reasonable values are 'Maximum diameter'<2 meters "
	     "(exceptionally greater than 2: very large diameters and/or intricate stems).",
	     2.0,
	     "meters",
	     0.0 + std::numeric_limits<double>::epsilon()},
	    {"stem_minimum_height",
	     "Lowest section",
	     "Lowest height at which stem diameter will be computed.",
	     0.3,
	     "meters",
	     0.0 + std::numeric_limits<double>::epsilon()},
	    {"stem_maximum_height",
	     "Highest section",
	     "Highest height at which stem diameter will be computed.",
	     25.0,
	     "meters",
	     0.0 + std::numeric_limits<double>::epsilon()},
	    {"stem_section_interval",
	     "Distance between sections",
	     "Height of the sections (z length). Diameters will then be computed for every section.",
	     0.2,
	     "meters",
	     0.0 + std::numeric_limits<double>::epsilon()},
	    {"stem_section_thickness",
	     "Section width",
	     "Sections are this wide. This means that points within this distance "
	     "(vertical) are considered during circle fitting and diameter coumputation.",
	     0.05,
	     "meters",
	     0.0 + std::numeric_limits<double>::epsilon()}};

	const std::vector<Field> expertFields = {
	    {"stripe_peeling_resolution_xy",
	     "(x, y) voxel resolution",
	     "(x, y) voxel resolution during stem extraction.",
	     0.02,
	     "meters",
	     0.0 + std::numeric_limits<double>::epsilon()},
	    {"stripe_peeling_resolution_z",
	     "(z) voxel resolution",
	     "(z) voxel resolution during stem extraction.",
	     0.02,
	     "meters",
	     0.0 + std::numeric_limits<double>::epsilon()},
	    {"stripe_peeling_voxels_threshold",
	     "Number of points",
	     "minimum number of points (voxels) per stem within the stripe "
	     "(DBSCAN clustering). Reasonable values are between 500 and 3000.",
	     1000,
	     "",
	     1},
	    {"verticality_radius_stripe",
	     "Vicinity radius (verticality computation)",
	     "Vicinity radius for PCA during stem identification.",
	     0.1,
	     "meters",
	     0.0 + std::numeric_limits<double>::epsilon()},
	    {"verticality_threshold_stripe",
	     "Verticality threshold",
	     "Verticality threshold durig stem identification.\n"
	     "Verticality is defined as (1 - sin(V)), being V the vertical angle of the "
	     "normal vector, measured from the horizontal. "
	     "Note that it does not grow linearly.",
	     0.7,
	     "(0, 1)",
	     0.0 + std::numeric_limits<double>::epsilon(),
	     1.0 - std::numeric_limits<double>::epsilon()},
	    {"tree_height_range",
	     "Vertical Range",
	     "Proportion (0: none - 1: all) of the vertical range of the stripe "
	     "that points need to extend through to be valid stems.",
	     0.7,
	     "[0, 1]",
	     0.0,
	     1.0},
	    {"tree_resolution_xy",
	     "(x, y) voxel resolution",
	     "(x, y) voxel resolution during tree extraction.",
	     0.035,
	     "meters",
	     0.0 + std::numeric_limits<double>::epsilon()},
	    {"tree_resolution_z",
	     "(z) voxel resolution",
	     "(z) voxel resolution during tree extraction.",
	     0.035,
	     "meters",
	     0.0 + std::numeric_limits<double>::epsilon()},
	    {"tree_minimum_points_stem",
	     "Minimum points",
	     "Minimum number of points (voxels) within a stripe to consider it "
	     "as a potential tree during tree individualization.",
	     20,
	     "",
	     1},
	    {"verticality_radius_stem",
	     "Vicinity radius (verticality computation)",
	     "Vicinity radius for PCA during tree individualization.",
	     0.1,
	     "meters",
	     0.0 + std::numeric_limits<double>::epsilon()},
	    {"verticality_threshold_stem",
	     "Verticality threshold",
	     "Verticality threshold durig stem extraction.\n"
	     "Verticality is defined as (1 - sin(V)), being V the vertical angle of the "
	     "normal vector, measured from the horizontal.\n"
	     "Note that it does not grow linearly.",
	     0.7,
	     "(0, 1)",
	     0.0 + std::numeric_limits<double>::epsilon(),
	     1.0 - std::numeric_limits<double>::epsilon()},
	    {"tree_dist_axis_threshold",
	     "Maximum distance to tree axis",
	     "Points that are closer than this distance to an axis "
	     "are assigned to that axis during tree individualization process.",
	     15.0,
	     "meters",
	     0.0 + std::numeric_limits<double>::epsilon()},
	    {"tree_height_distance_from_axis",
	     "Distance from axis",
	     "Maximum distance from tree axis at which points will "
	     "be considered while computing tree height.\n"
	     "Points too far away from the tree axis might not be representative"
	     "of actual tree height.",
	     1.5,
	     "meters",
	     0.0 + std::numeric_limits<double>::epsilon()},
	    {"tree_resolution_height",
	     "Voxel resolution for height computation",
	     "(x, y, z) voxel resolution during tree height computation.",
	     0.3,
	     "meters",
	     0.0 + std::numeric_limits<double>::epsilon()},
	    {"tree_axis_max_vertical_deviation",
	     "Maximum vertical deviation from axis",
	     "Maximum degree of vertical deviation from the axis for a tree height to be considered as valid.",
	     25.0,
	     "degrees",
	     0.0 + std::numeric_limits<double>::epsilon()},
	    {"stem_section_min_points",
	     "Points within section",
	     "Minimum number of points in a section to be considered as valid.",
	     80,
	     "",
	     1},
	    {"stem_section_diameter_proportion",
	     "Inner/outer circle proportion",
	     "Proportion, regarding the circumference fit by fit_circle, "
	     "that the inner circumference diameter will have as length.",
	     0.5,
	     "[0, 1]",
	     0.0,
	     1.0},
	    {"stem_section_minimum_diameter",
	     "Minimum expected diameter",
	     "Minimum diameter expected for any section during circle fitting.",
	     0.09,
	     "meters",
	     0.0 + std::numeric_limits<double>::epsilon()},
	    {"stem_section_inner_point_threshold",
	     "Points within inner circle",
	     "Maximum number of points inside the inner circle to consider the fitting as OK.",
	     5,
	     "",
	     1},
	    {"stem_section_clustering_distance",
	     "Maximum point distance",
	     "Maximum distance among points to be considered within the same cluster during circle fitting.",
	     0.02,
	     "meters",
	     0.0 + std::numeric_limits<double>::epsilon()},
	    {"stem_section_sector_count",
	     "Number of sectors",
	     "Number of sectors in which the circumference will be divided into.",
	     16,
	     "",
	     1},
	    {"stem_section_min_occupied_sectors",
	     "Number of occupied sectors",
	     "Minimum number of sectors that must be occupied.",
	     9,
	     "",
	     1},
	    {"stem_section_circle_width",
	     "Circle width",
	     "Width, in meters, around the circumference to look for points.",
	     0.02,
	     "meters",
	     0.0 + std::numeric_limits<double>::epsilon()},
	    {"circa",
	     "N of points to draw each circle",
	     "Number of points that will be used to draw the circles.",
	     200,
	     "",
	     1},
	    {"p_interval",
	     "Interval at which points are drawn while drawing",
	     "Distance at which points will be placed from one to another while drawing the axes point cloud.",
	     0.01,
	     "meters",
	     0.0 + std::numeric_limits<double>::epsilon()},
	    {"axis_downstep",
	     "Axis downstep from stripe center",
	     "From the stripe centroid, how much (downwards direction) "
	     "will the drawn axes extend.\nBasically, this parameter controls"
	     "from where will the axes be drawn.",
	     0.5,
	     "meters",
	     0.0 + std::numeric_limits<double>::epsilon()},
	    {"axis_upstep",
	     "Axis upstep from stripe center",
	     "From the stripe centroid, how much (upwards direction) "
	     "will the drawn axes extend.\nBasically, this parameter control"
	     "how long will the drawn axes be.",
	     10.0,
	     "meters",
	     0.0 + std::numeric_limits<double>::epsilon()},
	    {"denoise_resolution",
	     "(x, y, z) voxel resolution",
	     "(x, y, z) voxel resolution during denoising.\nNote that the whole point cloud is voxelated.",
	     0.15,
	     "meters",
	     0.0 + std::numeric_limits<double>::epsilon()},
	    {"denoise_minimum_points",
	     "Minimum number of points",
	     "Clusters with size smaller than this value will be regarded as noise and thus eliminated.",
	     2,
	     "",
	     int(1)},
	    {"dtm_smooth_laplacian_lambda",
	     "DTM smoothing strenght",
	     "lambda parameter of laplacian smoothing applied to the DTM, should be approx. [0.1;0.2]",
	     0.125,
	     "",
	     0.0 + std::numeric_limits<double>::epsilon(),
	     1.0}};

	const std::vector<Field> miscFields = {
	    {"compute_height_normalization",
	     "Normalize point cloud",
	     "If the point cloud is not height-normalized, a Digital Terrain "
	     "Model (DTM) will be generated to compute normalized heights for all points.",
	     true},
	    {"denoise_point_cloud",
	     "Clean noise on DTM",
	     "If it is expected to be noise below ground level (or if you know "
	     "that there is noise), a denoising step will be added before "
	     "generating the Digital Terrain Model.",
	     false},
	    {"export_txt",
	     "Format of output tabular data",
	     "Outputs are gathered in a xlsx (Excel) file by default.\n"
	     "Selecting \"TXT files\" will make the program output several txt files "
	     "with the raw data, which may be more convenient for processing "
	     "the data via scripting.",
	     false},
	};

	const std::unordered_map<QString, Field> Field::getConfigFields()
	{
		std::unordered_map<QString, Field> mapConfigFields;
		auto                               vecToFieldMap = [&mapConfigFields](const std::vector<Field>& vecField)
		{
			for (const auto& field : vecField)
			{
				mapConfigFields[field.name] = field;
			}
		};

		vecToFieldMap(basicFields);
		vecToFieldMap(advancedFields);
		vecToFieldMap(expertFields);
		vecToFieldMap(miscFields);

		return mapConfigFields;
	}
} // namespace tdf
