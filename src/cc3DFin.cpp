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

#include "CCGeom.h"
#include "cc3DFinDlg.h"
#include "ccHObject.h"
#include "ccHObjectCaster.h"
#include "ccLog.h"
#include "ccPointCloud.h"

// lib3DFin
#include <lib3DFin/config.hpp>
#include <lib3DFin/interface.hpp>
#include <lib3DFin/types.hpp>

// ccCoreLib
#include <QMainWindow>
#include <QtGui>
#include <ScalarField.h>

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
	const auto [stripe, cloud, axes, circles] = lib3dfin::process(&(pc->getNextPoint()->u[0]), static_cast<size_t>(pc->size()));

	ccPointCloud* stripe_pc = new ccPointCloud;
	stripe_pc->reserve(cloud.size() / 3);

	for (size_t point_id = 0; point_id < cloud.size() / 3; ++point_id)
	{
		auto id = point_id * 3;
		stripe_pc->addPoint(CCVector3(cloud[id], cloud[id + 1], cloud[id + 2]));
	}

	m_app->addToDB(stripe_pc);

	auto        id      = pc->addScalarField("dist_id");
	auto*       dist_id = pc->getScalarField(id);
	std::size_t count   = 0;
	for (auto elem : stripe)
	{
		dist_id->setValue(count, elem);
		count++;
	}
	dist_id->computeMinAndMax();
	pc->setCurrentDisplayedScalarField(id);

	m_app->redrawAll();
	// draw circles
	drawCircles(circles, axes.tree_descriptors);
	ccLog::Print("3DFin done !");
	// cc3DFinDlg tdfDlg(m_app->getMainWindow(), scalarFieldNames);

	// tdfDlg.exec();

	QApplication::processEvents();
}

void cc3DFin::drawCircles(const std::vector<lib3dfin::CircleSections>& all_tree_circles, const std::vector<lib3dfin::TreeDescriptor>& tree_descriptors)
{
	// TODO: global shift
	size_t tree_id = 0;

	// Create a point cloud for circle points
	std::unique_ptr<ccPointCloud> circle_points_pc(new ccPointCloud(QString("circle points debug")));

	// Add scalar fields for circle properties
	int tree_id_sf_id    = circle_points_pc->addScalarField("Tree ID");
	int radius_sf_id     = circle_points_pc->addScalarField("Radius");
	int height_sf_id     = circle_points_pc->addScalarField("Height");
	int status_sf_id     = circle_points_pc->addScalarField("Status");
	int num_points_sf_id = circle_points_pc->addScalarField("Number of points inner circle");
	int sector_sf_id     = circle_points_pc->addScalarField("Sector coverage");
	int outlier_sf_id    = circle_points_pc->addScalarField("Outlier probability");

	auto* tree_id_sf    = circle_points_pc->getScalarField(tree_id_sf_id);
	auto* radius_sf     = circle_points_pc->getScalarField(radius_sf_id);
	auto* height_sf     = circle_points_pc->getScalarField(height_sf_id);
	auto* status_sf     = circle_points_pc->getScalarField(status_sf_id);
	auto* num_points_sf = circle_points_pc->getScalarField(num_points_sf_id);
	auto* sector_sf     = circle_points_pc->getScalarField(sector_sf_id);
	auto* outlier_sf    = circle_points_pc->getScalarField(outlier_sf_id);

	for (const auto& tree_circles : all_tree_circles)
	{
		for (const auto& circle_data : tree_circles)
		{
			// Only draw successful circles
			if (circle_data.status >= lib3dfin::CircleData::Status::SUCCESS)
			{
				const auto& circle = circle_data.circle;
				// We need to shift the circle center to go from z0 coordinates to the actual coordinates
				const double height   = circle_data.z0 + tree_descriptors[tree_id].height_difference;
				const float  f_height = static_cast<float>(height);

				// Add circle center
				circle_points_pc->addPoint({static_cast<float>(circle.center.x()),
				                            static_cast<float>(circle.center.y()),
				                            f_height});

				// Set scalar field values
				tree_id_sf->addElement(tree_id);
				radius_sf->addElement(circle.radius);
				height_sf->addElement(height);
				status_sf->addElement(static_cast<double>(circle_data.status));
				num_points_sf->addElement(circle_data.number_points_inner);
				sector_sf->addElement(circle_data.sector_percentage);
				outlier_sf->addElement(circle_data.outlier_probability);

				// Generate circle points for visualization
				const uint32_t num_circle_points = 200; // sampling
				for (uint32_t i = 0; i < num_circle_points; ++i)
				{
					const double angle = 2.0 * M_PI * i / num_circle_points;
					const double x     = circle.center.x() + circle.radius * cos(angle);
					const double y     = circle.center.y() + circle.radius * sin(angle);

					circle_points_pc->addPoint({static_cast<float>(x),
					                            static_cast<float>(y),
					                            f_height});

					tree_id_sf->addElement(tree_id);
					radius_sf->addElement(circle.radius);
					height_sf->addElement(height);
					status_sf->addElement(static_cast<double>(circle_data.status));
					num_points_sf->addElement(circle_data.number_points_inner);
					sector_sf->addElement(circle_data.sector_percentage);
					outlier_sf->addElement(circle_data.outlier_probability);
				}
			}
		}
		++tree_id;
	}
	if (circle_points_pc->size() > 0)
	{
		tree_id_sf->computeMinAndMax();
		radius_sf->computeMinAndMax();
		height_sf->computeMinAndMax();
		status_sf->computeMinAndMax();
		num_points_sf->computeMinAndMax();
		sector_sf->computeMinAndMax();
		outlier_sf->computeMinAndMax();

		// Set default displayed scalar field to radius
		circle_points_pc->setCurrentDisplayedScalarField(status_sf_id);

		m_app->addToDB(circle_points_pc.release());
	}
}
