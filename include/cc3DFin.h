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

#include <lib3DFin/config.hpp>
#include <lib3DFin/types.hpp>
#include <optional>

class cc3DFinDlg;

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

	//= Inherited from ccStdPluginInterface
	void            onNewSelection(const ccHObject::Container& selectedEntities) override;
	QList<QAction*> getActions() override;

  private:
	static void initCustomColorScale();

	void                                 do3DFinAction();
	void                                 compute3DFin(const lib3dfin::Params& params, cc3DFinDlg& dialog);
	[[nodiscard]] std::optional<Eigen::VectorXd>       loadZ0Values(const std::string& sfName) const;
	[[nodiscard]] std::optional<lib3dfin::PointCloud3> loadPointCloudCoordinates() const;

	QAction*      m_action;
	ccPointCloud* m_currentCloud{nullptr};
};
