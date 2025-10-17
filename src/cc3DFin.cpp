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
#include "cc2DLabel.h"
#include "cc3DFinDlg.h"
#include "ccHObject.h"
#include "ccHObjectCaster.h"
#include "ccLog.h"
#include "ccPointCloud.h"
#include "ccPolyline.h"

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

	// Cast to PC
	ccPointCloud* pc = static_cast<ccPointCloud*>(ent);

	// Get scalar field names for plugin UI
	QStringList scalarFieldNames;
	for (int i = 0; i < pc->getNumberOfScalarFields(); ++i)
	{
		const CCCoreLib::ScalarField* sf = pc->getScalarField(i);
		if (sf)
			scalarFieldNames.push_back(QString(sf->getName().c_str()));
	}

	pc->placeIteratorAtBeginning();
	const auto [stripe, cloud, tree_data, circles] = lib3dfin::process(&(pc->getNextPoint()->u[0]), static_cast<size_t>(pc->size()));

	// reset the base group
	pc->setEnabled(false);
	m_base_group.reset(new ccHObject("3DFin group"));

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
		++count;
	}
	dist_id->computeMinAndMax();
	pc->setCurrentDisplayedScalarField(id);

	drawCircles(circles, tree_data.tree_descriptors);
	drawTreeLocators(tree_data.tree_descriptors);
	drawTreeHeights(tree_data.tree_descriptors);
	drawAxes(tree_data.tree_descriptors);
	m_app->addToDB(m_base_group.release());
	m_app->redrawAll();
	// draw circles
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
	ccPointCloud* circle_points_pc = new ccPointCloud(QString("Fitted sections"));

	// Add scalar fields for circle properties
	int tree_id_sf_id    = circle_points_pc->addScalarField("Tree_ID");
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
		circle_points_pc->toggleSF();
		m_base_group->addChild(circle_points_pc);
	}
}

void cc3DFin::drawAxes(const std::vector<lib3dfin::TreeDescriptor>& tree_descriptors)
{
	// TODO: global shift
	// TODO: go back to point cloud sampling since we need to visualize the tilt deviation scalar field
	//  Create a group to hold all axes as polylines
	ccHObject* axes_group = new ccHObject(QString("Tree Axes"));

	size_t tree_id = 0;
	for (const auto& desc : tree_descriptors)
	{
		Eigen::Vector3d bottom_point = desc.bottom_point;
		Eigen::Vector3d top_point    = desc.top_point;
		double          length       = desc.height_difference;

		// Create a polyline for the axis
		ccPointCloud* axis_points = new ccPointCloud();
		axis_points->addPoint(CCVector3(bottom_point.x(), bottom_point.y(), bottom_point.z()));
		axis_points->addPoint(CCVector3(top_point.x(), top_point.y(), top_point.z()));

		ccPolyline* axis_line = new ccPolyline(axis_points);
		axis_line->addPointIndex(0);
		axis_line->addPointIndex(1);
		axis_line->setName(QString("Axis %1").arg(tree_id));
		axis_line->setColor(ccColor::Rgb(255, 0, 0));
		axis_line->showColors(true);
		axis_line->setWidth(3);

		axis_points->setEnabled(false);

		axes_group->addChild(axis_line);

		++tree_id;
	}

	m_base_group->addChild(axes_group);
}
// https : // github.com/3DFin/3DFin/blob/main/src/three_d_fin/cloudcompare/plugin_processing.py
void cc3DFin::drawTreeLocators(const std::vector<lib3dfin::TreeDescriptor>& tree_descriptors)
{

	// TODO global shift
	ccPointCloud* tree_locations = new ccPointCloud("Tree Locations");

	tree_locations->setPointSize(8);
	tree_locations->setColor(255, 0, 255, 255);
	tree_locations->toggleColors();
	int    id_dbh  = tree_locations->addScalarField("dbh");
	auto   dbh_sf  = tree_locations->getScalarField(id_dbh);
	size_t tree_id = 0;

	for (const auto& desc : tree_descriptors)
	{
		tree_locations->addPoint(CCVector3(desc.location.x(), desc.location.y(), desc.location.z()));
		dbh_sf->addElement(desc.dbh);
		cc2DLabel* label = new cc2DLabel();
		label->addPickedPoint(tree_locations, tree_id);
		if (desc.dbh < std::numeric_limits<float>::epsilon())
		{
			label->setName(QString("Tree %1 | Not Reliable").arg(tree_id + 1));
		}
		else
		{
			label->setName(QString("Tree %1 | %2").arg(tree_id + 1).arg(desc.dbh));
		}
		label->displayPointLegend(true);
		label->toggleVisibility();
		label->setDisplayedIn2D(false);
		tree_locations->addChild(label);
		++tree_id;
	}
	tree_locations->toggleColors();
	m_base_group->addChild(tree_locations);
}

void cc3DFin::drawTreeHeights(const std::vector<lib3dfin::TreeDescriptor>& tree_descriptors)
{
	// TODO global shift
	ccPointCloud* tree_heights = new ccPointCloud("Highest points");

	tree_heights->setPointSize(8);
	tree_heights->setColor(255, 0, 255, 255);
	tree_heights->toggleColors();
	int    id_z0       = tree_heights->addScalarField("z0");
	auto   z0_sf       = tree_heights->getScalarField(id_z0);
	int    id_deviated = tree_heights->addScalarField("deviated");
	auto   deviated_sf = tree_heights->getScalarField(id_deviated);
	size_t tree_id     = 0;
	tree_heights->setPointSize(8);

	// add labels with z0 values
	for (const auto& desc : tree_descriptors)
	{
		tree_heights->addPoint(CCVector3(desc.highest_point.x(), desc.highest_point.y(), desc.highest_point.z()));
		z0_sf->addElement(desc.highest_z0);
		deviated_sf->addElement(desc.valid);
		cc2DLabel* label = new cc2DLabel(QString("point %1").arg(tree_id + 1));
		label->addPickedPoint(tree_heights, tree_id);
		label->setName(QString::number(desc.highest_z0));
		label->displayPointLegend(true);
		label->toggleVisibility();
		label->setDisplayedIn2D(false);
		tree_heights->addChild(label);
		++tree_id;
	}

	tree_heights->toggleColors();
	m_base_group->addChild(tree_heights);
}
