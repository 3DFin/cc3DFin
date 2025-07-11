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
#include <vector>

namespace tdf
{
	std::vector<Field> basicFields = {
	    {
	        "z0_name",
	        "Normalized height field name",
	        "Name of the Z0 field in the point cloud. \n"
	        "If the normalized heights are stored in the Z coordinate "
	        "of the point cloud, then: Z0 field name = \"z\" (lowercase).",
	        "Z0",
	    },
	    {
	        "upper_limit",
	        "Stripe upper Limit",
	        "Upper (vertical) limit of the stripe where it should be reasonable "
	        "to find stems with minimum presence of shrubs or branches. \n"
	        "Reasonable values are 2-5 meters.",
	        3.5,
	        // TODO gt          = 0,
	        "meters",
	    },
	    {"lower_limit",
	     "Stripe lower Limit",
	     "Lower (vertical) limit of the stripe where it should be reasonable "
	     "to find stems with minimum presence of shrubs or branches. \n"
	     "Reasonable values are 0.3-1.3 meters.",
	     0.7,
	     // TODO gt = 0
	     "meters"},
	    {
	        "number_of_iterations",
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
	        "res_cloth",
	        "Cloth resolution",
	        "Initial cloth grid resolution to generate the DTM that will be used to compute normalized heights.",
	        // TODO gt=0,
	        0.7,
	        "meters",
	    } // TODO: lower limit validator
	};

	std::vector<Field> advancedFields = {
	    {"maximum_diameter",
	     "Expected maximum diameter",
	     "Maximum diameter expected for any stem.",
	     // TODO: gt          = 0,
	     1.0,
	     "meters"}};

	std::vector<Field> Field::getConfigFields()
	{
		return basicFields;
	}
} // namespace tdf
