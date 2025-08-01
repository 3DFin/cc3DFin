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

#include "cc3DFin.h"

#include "cc3DFinDlg.h"
#include "ccHObject.h"
#include "ccHObjectCaster.h"
#include "ccLog.h"
#include "ccPointCloud.h"

#include "config.hpp"
#include "interface.hpp"

// ccCoreLib
#include <ScalarField.h>

#include <QMainWindow>
#include <QtGui>
#include <vector>

cc3DFin::cc3DFin(QObject* parent)
    : QObject(parent)
    , ccStdPluginInterface(":/CC/plugin/3DFin/info.json")
    , m_action(nullptr)
{
}

// This method should enable or disable your plugin actions
// depending on the currently selected entities ('selectedEntities').
void cc3DFin::onNewSelection(const ccHObject::Container& selectedEntities)
{
	if (m_action == nullptr)
	{
		return;
	}
	m_action->setEnabled(!selectedEntities.empty());
}

// This method returns all the 'actions' your plugin can perform.
// getActions() will be called only once, when plugin is loaded.
QList<QAction*> cc3DFin::getActions()
{
	// default action (if it has not been already created, this is the moment to do it)
	if (!m_action)
	{
		// Here we use the default plugin name, description, and icon,
		// but each action should have its own.
		m_action = new QAction(getName(), this);
		m_action->setToolTip(getDescription());
		m_action->setIcon(getIcon());
		connect(m_action, &QAction::triggered, this, &cc3DFin::do3DFinAction);
	}

	return {m_action};
}

void cc3DFin::do3DFinAction()
{
	assert(m_app);
	if (!m_app)
	{
		return;
	}

	// we need one point cloud
	if (!m_app->haveOneSelection())
	{
		m_app->dispToConsole("Select only one cloud!", ccMainAppInterface::ERR_CONSOLE_MESSAGE);
		return;
	}

	// a real point cloud
	const ccHObject::Container& selectedEntities = m_app->getSelectedEntities();

	ccHObject* ent = selectedEntities[0];
	if (!ent->isA(CC_TYPES::POINT_CLOUD))
	{
		m_app->dispToConsole("Select a cloud!", ccMainAppInterface::ERR_CONSOLE_MESSAGE);
		return;
	}

	// cast to PC
	ccPointCloud* pc = static_cast<ccPointCloud*>(ent);

	QStringList scalarFieldNames;
	for (int i = 0; i < pc->getNumberOfScalarFields(); ++i)
	{
		const CCCoreLib::ScalarField* sf = pc->getScalarField(i);
		if (sf)
			scalarFieldNames.push_back(QString(sf->getName().c_str()));
	}

	pc->placeIteratorAtBeginning();
	const auto z0_values = lib3dfin::process(&(pc->getNextPoint()->u[0]), static_cast<size_t>(pc->size()));

	auto sf_id = pc->addScalarField("z0_test");
	ccLog::Print(QString(sf_id));

	auto* sf = pc->getScalarField(sf_id);

	size_t count = 0;
	for(float z0_value: z0_values)
	{
	    sf->setLocalValue(count++, z0_value);
	}

	sf->computeMinAndMax();
	ccLog::Print("3DFin done !");
	pc->setCurrentDisplayedScalarField(sf_id);
	//cc3DFinDlg tdfDlg(m_app->getMainWindow(), scalarFieldNames);

	//tdfDlg.exec();

	QApplication::processEvents();
}
