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

#pragma once

// CloudCompare
#include "ccPointCloud.h"

// QT
#include <QString>

// lib3DFin
#include <lib3DFin/interface.hpp>
#include <lib3DFin/types.hpp>

// StdLib
#include <memory>

class ccHObject;
namespace lib3dfin
{
	class TDFProcessing;
}

class cc3DFinDrawer
{
  public:
	cc3DFinDrawer(ccPointCloud* sourceCloud);
	std::unique_ptr<ccHObject> drawAll(const lib3dfin::TDFProcessing& result);

  private:
	void drawDTM(const lib3dfin::DTMData& dtm);
	void drawCircles(const std::vector<lib3dfin::TreeDescriptor>& tree_descriptors);
	void drawAxis(const std::vector<lib3dfin::TreeDescriptor>& tree_descriptors);
	void drawTreeLocators(const std::vector<lib3dfin::TreeDescriptor>& tree_descriptors);
	void drawTreeHeights(const std::vector<lib3dfin::TreeDescriptor>& tree_descriptors);
	void exportEnrichedCloud(const lib3dfin::TreeData& tree_data, const Eigen::VectorXd& z0);
	void exportStripe(const lib3dfin::ArrayClusterIndicator& stem_indicator);

	ccPointCloud*              m_source;
	std::unique_ptr<ccHObject> m_group;
};
