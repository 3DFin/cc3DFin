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
	    {
	        "z0_name",
	        "Normalized height field name",
	        "Name of the Z0 field in the point cloud. \n"
	        "If the normalized heights are stored in the Z coordinate "
	        "of the point cloud, then: Z0 field name = \"z\" (lowercase).",
	        "Z0",
	    },
	    {
	        "stripe_upper_limit",
	        "Stripe upper Limit",
	        "Upper (vertical) limit of the stripe where it should be reasonable "
	        "to find stems with minimum presence of shrubs or branches. \n"
	        "Reasonable values are 2-5 meters.",
	        3.5,
	        // TODO gt          = 0,
	        "meters",
	    },
	    {"stripe_lower_limit",
	     "Stripe lower Limit",
	     "Lower (vertical) limit of the stripe where it should be reasonable "
	     "to find stems with minimum presence of shrubs or branches. \n"
	     "Reasonable values are 0.3-1.3 meters.",
	     0.7,
	     // TODO gt = 0
	     "meters"},
	    {
	        "stripe_peeling_num_iterations",
	        "Prunning intensity",
	        "Number of iterations of \"pruning\" during stem identification. \n"
	        "Values between 1 (slight stem peeling/cleaning) "
	        "and 5 (extreme branch peeling/cleaning).",
	        // TODO ge = 0,
	        // TODO le = 5,
	        2,
	        "0-5",
	    },
	    {
	        "cloth_resolution",
	        "Cloth resolution",
	        "Initial cloth grid resolution to generate the DTM that will be used to compute normalized heights.",
	        // TODO gt=0,
	        0.45,
	        "meters",
	    } // TODO: lower limit validator
	};

	const std::vector<Field> advancedFields = {
	    {"maximum_diameter",
	     "Expected maximum diameter",
	     "Maximum diameter expected for any stem.",
	     // TODO: gt          = 0,
	     1.0,
	     "meters"},
	    {
	        "stem_search_diameter",
	        "Stem search diameter",
	        "Points within this distance from tree axes will be considered "
	        "as potential stem points.\n"
	        "Reasonable values are 'Maximum diameter'<2 meters "
	        "(exceptionally greater than 2: very large diameters and/or intricate stems).",
	        // gt=0,
	        2.0,
	        "meters",
	    },
	    {
	        "minimum_height",
	        "Lowest section",
	        "Lowest height at which stem diameter will be computed.",
	        // gt=0,
	        0.3,
	        "meters",
	    },
	    {
	        "maximum_height",
	        "Highest section",
	        "Highest height at which stem diameter will be computed.",
	        // gt=0,
	        25.0,
	        "meters",
	    },
	    {
	        "section_len",
	        "Distance between sections",
	        "Height of the sections (z length). Diameters will then be computed for every section.",
	        // TODO gt=0,
	        0.2,
	        "meters",
	    },
	    {
	        "section_wid",
	        "Section width",
	        "Sections are this wide. This means that points within this distance "
	        "(vertical) are considered during circle fitting and diameter coumputation.",
	        // gt=0,
	        0.05,
	        "meters",
	    },
	};

	const std::vector<Field> expertFields = {
	    {
	        "stripe_peeling_resolution_xy",
	        "(x, y) voxel resolution",
	        "(x, y) voxel resolution during stem extraction.",
	        // TODO / gt=0,
	        0.02,
	        "meters",
	    },
	    {
	        "stripe_peeling_resolution_z",
	        "(z) voxel resolution",
	        "(z) voxel resolution during stem extraction.",
	        // TODO: gt=0,
	        0.02,
	        "meters",
	    },
	    {
	        "stripe_peeling_voxels_threshold",
	        "Number of points",
	        "minimum number of points (voxels) per stem within the stripe "
	        "(DBSCAN clustering). Reasonable values are between 500 and 3000.",
	        // TODO gt=0,
	        1000,
	    },
	    {
	        "verticality_radius_stripe",
	        "Vicinity radius (verticality computation)",
	        "Vicinity radius for PCA during stem identification.",
	        // TODO gt=0,
	        0.1,
	        "meters",
	    },
	    {
	        "verticality_threshold_stripe",
	        "Verticality threshold",
	        "Verticality threshold durig stem identification.\n"
	        "Verticality is defined as (1 - sin(V)), being V the vertical angle of the "
	        "normal vector, measured from the horizontal. "
	        "Note that it does not grow linearly.",
	        // TODO gt=0.0,
	        // TDOO lt=1.0,
	        0.7,
	        "(0, 1)",
	    },
	    {
	        "tree_height_range",
	        "Vertical Range",
	        "Proportion (0: none - 1: all) of the vertical range of the stripe "
	        "that points need to extend through to be valid stems.",
	        // TODO ge=0,
	        // TODO le=1,
	        0.7,
	        "[0, 1]",
	    },
	    {
	        "tree_resolution_xy",
	        "(x, y) voxel resolution",
	        "(x, y) voxel resolution during tree extraction.",
	        // TODO / gt=0,
	        0.035,
	        "meters",
	    },
	    {
	        "tree_resolution_z",
	        "(z) voxel resolution",
	        "(z) voxel resolution during tree extraction.",
	        // gt=0,
	        0.035,
	        "meters",
	    },
	    {
	        "tree_minimum_points_stem",
	        "Minimum points",
	        "Minimum number of points (voxels) within a stripe to consider it "
	        "as a potential tree during tree individualization.",
	        // TODO gt=0,
	        20,
	    },
	    {
	        "verticality_scale_stems",
	        "Vicinity radius (verticality computation)",
	        "Vicinity radius for PCA during tree individualization.",
	        // TODO gt=0,
	        // TODO lt=1.0,
	        0.1,
	        "meters",
	    },
	    {
	        "verticality_thresh_stems",
	        "Verticality threshold",
	        "Verticality threshold durig stem extraction.\n"
	        "Verticality is defined as (1 - sin(V)), being V the vertical angle of the "
	        "normal vector, measured from the horizontal.\n"
	        "Note that it does not grow linearly.",
	        0.7,
	        // TODO gt=0.0,
	        // TODO le=1.0,
	        "(0, 1)",
	    },
	    {
	        "tree_dist_axis_threshold",
	        "Maximum distance to tree axis",
	        "Points that are closer than this distance to an axis "
	        "are assigned to that axis during tree individualization process.",
	        // TODO gt=0.0,
	        15.0,
	        "meters",
	    },
	    {
	        "tree_height_distance_from_axis",
	        "Distance from axis",
	        "Maximum distance from tree axis at which points will "
	        "be considered while computing tree height.\n"
	        "Points too far away from the tree axis might not be representative"
	        "of actual tree height.",
	        // gt=0,
	        1.5,
	        "meters",
	    },
	    {
	        "tree_resolution_height",
	        "Voxel resolution for height computation",
	        "(x, y, z) voxel resolution during tree height computation.",
	        // gt=0,
	        0.3,
	        "meters",
	    },
	    {
	        "tree_axis_max_vertical_deviation",
	        "Maximum vertical deviation from axis",
	        "Maximum degree of vertical deviation from the axis for a tree height to be considered as valid.",
	        // gt=0,
	        25.0,
	        "degrees",
	    },
	    {
	        "number_points_section",
	        "Points within section",
	        "Minimum number of points in a section to be considered as valid.",
	        // gt=0,
	        80,
	    },
	    {
	        "diameter_proportion",
	        "Inner/outer circle proportion",
	        "Proportion, regarding the circumference fit by fit_circle, "
	        "that the inner circumference diameter will have as length.",
	        // TODO ge=0.0,
	        // TODO le=1.0,
	        0.5,
	        "[0, 1]",
	    },
	    {
	        "minimum_diameter",
	        "Minimum expected diameter",
	        "Minimum diameter expected for any section during circle fitting.",
	        // TODO gt=0,
	        0.09,
	        "meters",
	    },
	    {
	        "point_threshold",
	        "Points within inner circle",
	        "Maximum number of points inside the inner circle to consider the fitting as OK.",
	        // TODO gt=0,
	        5,
	    },
	    {
	        "point_distance",
	        "Maximum point distance",
	        "Maximum distance among points to be considered within the same cluster during circle fitting.",
	        // TODO: gt=0,
	        0.02,
	        "meters",
	    },
	    {
	        "number_sectors",
	        "Number of sectors",
	        "Number of sectors in which the circumference will be divided into.",
	        // gt=0,
	        16,
	    },
	    {
	        "m_number_sectors",
	        "Number of occupied sectors",
	        "Minimum number of sectors that must be occupied.",
	        // gt          = 0,
	        9,
	    },
	    {
	        "circle_width",
	        "Circle width",
	        "Width, in meters, around the circumference to look for points.",
	        // TODO gt=0,
	        0.02,
	        "meters",
	    },
	    {
	        "circa",
	        "N of points to draw each circle",
	        "Number of points that will be used to draw the circles.",
	        // TODO: gt=0,
	        200,
	    },
	    {
	        "p_interval",
	        "Interval at which points are drawn while drawing",
	        "Distance at which points will be placed from one to another while drawing the axes point cloud.",
	        // TODO: gt=0,
	        0.01,
	        "meters",
	    },
	    {
	        "axis_downstep",
	        "Axis downstep from stripe center",
	        "From the stripe centroid, how much (downwards direction) "
	        "will the drawn axes extend.\nBasically, this parameter controls"
	        "from where will the axes be drawn.",
	        // TODO gt=0,
	        0.5,
	        "meters",
	    },
	    {
	        "axis_upstep",
	        "Axis upstep from stripe center",
	        "From the stripe centroid, how much (upwards direction) "
	        "will the drawn axes extend.\nBasically, this parameter control"
	        "how long will the drawn axes be.",
	        // TODO: gt          = 0,
	        10.0,
	        "meters",
	    },
	    {
	        "denoise_resolution",
	        "(x, y, z) voxel resolution",
	        "(x, y, z) voxel resolution during denoising.\nNote that the whole point cloud is voxelated.",
	        // TODO: gt=0,
	        0.15,
	        "meters",
	    },
	    {
	        "denoise_minimum_points",
	        "Minimum number of points",
	        "Clusters with size smaller than this value will be regarded as noise and thus eliminated.",
	        // TODO: gt=0,
	        2,

	    }};

	const std::vector<Field> miscFields = {
	    {

	        "compute_height_normalization",
	        "Normalize point cloud",
	        "If the point cloud is not height-normalized, a Digital Terrain "
	        "Model (DTM) will be generated to compute normalized heights for all points.",
	        false,
	    },
	    {
	        "denoise_point_cloud",
	        "Clean noise on DTM",
	        "If it is expected to be noise below ground level (or if you know "
	        "that there is noise), a denoising step will be added before "
	        "generating the Digital Terrain Model.",
	        false,
	    },
	    {
	        "export_txt",
	        "Format of output tabular data",
	        "Outputs are gathered in a xlsx (Excel) file by default.\n"
	        "Selecting \"TXT files\" will make the program output several txt files "
	        "with the raw data, which may be more convenient for processing "
	        "the data via scripting.",
	        false,
	    },
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
