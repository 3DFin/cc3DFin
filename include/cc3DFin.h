#pragma once

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

#include "ccStdPluginInterface.h"

#include <lib3DFin/types.hpp>

//! 3DFin qCC plugin
class cc3DFin : public QObject
    , public ccStdPluginInterface
{
	Q_OBJECT
	Q_INTERFACES(ccPluginInterface ccStdPluginInterface)

	// The info.json file provides information about the plugin to the loading system and
	// it is displayed in the plugin information dialog.
	Q_PLUGIN_METADATA(IID "uniovi.cloudcompare.plugin.cc3DFin" FILE "../info.json")

  public:
	explicit cc3DFin(QObject* parent = nullptr);
	~cc3DFin() override = default;

	// Inherited from ccStdPluginInterface
	void onNewSelection(const ccHObject::Container& selectedEntities) override;

	QList<QAction*> getActions() override;

  private:
	void initCustomColorScale();

	void do3DFinAction();

	void drawCircles(const std::vector<lib3dfin::CircleSections>& all_tree_circles, const std::vector<lib3dfin::TreeDescriptor>& tree_descriptors);

	void drawAxis(const std::vector<lib3dfin::TreeDescriptor>& tree_descriptors);

	void drawTreeLocators(const std::vector<lib3dfin::TreeDescriptor>& tree_descriptors);

	void drawTreeHeights(const std::vector<lib3dfin::TreeDescriptor>& tree_descriptors);

	void exportEnrichedCloud(const lib3dfin::TreeData& tree_data, const std::vector<double>& z0);

	void exportStripe(const lib3dfin::TreeData& tree_data);

  private:
	//! Default action
	QAction* m_action;

	std::unique_ptr<ccHObject> m_base_group{nullptr};

	ccPointCloud* m_current_cloud{nullptr};

	const QString s_color_scale_uuid = "{25ec76a1-9b8d-4e4a-a129-21ae313ef8ba}";
};
