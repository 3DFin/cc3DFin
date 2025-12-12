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
#include "ccColorScalesManager.h"
#include "ccHObject.h"
#include "ccHObjectCaster.h"
#include "ccLog.h"
#include "ccPointCloud.h"
#include "ccPolyline.h"
#include "ccScalarField.h"

// lib3DFin
#include <lib3DFin/config.hpp>
#include <lib3DFin/interface.hpp>
#include <lib3DFin/types.hpp>

// spdlog
#include <spdlog/sinks/qt_sinks.h>
#include <spdlog/spdlog.h>

// ccCoreLib
#include <QMainWindow>
#include <QtConcurrent>
#include <QtGui>
#include <ScalarField.h>

// stdlib
#include <chrono>

cc3DFin::cc3DFin(QObject* parent)
    : QObject(parent)
    , ccStdPluginInterface(":/CC/plugin/3DFin/info.json")
    , m_action(nullptr)
{
	initCustomColorScale();
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
	m_current_cloud = static_cast<ccPointCloud*>(ent);

	// Get scalar field names for plugin UI
	QStringList scalarFieldNames;
	for (int i = 0; i < m_current_cloud->getNumberOfScalarFields(); ++i)
	{
		const CCCoreLib::ScalarField* sf = m_current_cloud->getScalarField(i);
		if (sf)
			scalarFieldNames.push_back(QString(sf->getName().c_str()));
	}

	m_current_cloud->setEnabled(false);
	m_current_cloud->placeIteratorAtBeginning();

	cc3DFinDlg tdfDlg(m_app->getMainWindow(), scalarFieldNames);
	connect(tdfDlg.compute_btn, &QPushButton::clicked, [this, &tdfDlg]
	        {
		        const auto params    = tdfDlg.get3DFinParameters();
		        int        max_lines = 1000;
		        auto       logger    = spdlog::qt_color_logger_mt("3DFin", tdfDlg.logTextEdit, max_lines);

					logger->set_pattern("[%T] %^[%l]%$ %v");
		            tdfDlg.tabWidget->setCurrentIndex(3); // switch to log tab
		        compute3DFin(params, logger); });
	// draw circles
	tdfDlg.exec();
	m_current_cloud = nullptr;

	QApplication::processEvents();
}

void cc3DFin::initCustomColorScale()
{
	auto maybe_colorscale = ccColorScalesManager::GetUniqueInstance()->getScale(s_color_scale_uuid);

	if (maybe_colorscale != nullptr)
	{
		ccLog::Print("Color Scale already exists");
		return;
	}

	ccColorScale::Shared customColorScale = ccColorScale::Create("3DFin");
	customColorScale->setUuid(s_color_scale_uuid);
	customColorScale->setRelative();

	customColorScale->insert(ccColorScaleElement(0., {91, 155, 213}));
	customColorScale->insert(ccColorScaleElement(0.28571428571, {112, 173, 71}));
	customColorScale->insert(ccColorScaleElement(0.642857142857, {255, 192, 0}));
	customColorScale->insert(ccColorScaleElement(1, {237, 125, 49}));

	ccColorScalesManager::GetUniqueInstance()->addScale(customColorScale);
}

void cc3DFin::drawCircles(const std::vector<lib3dfin::TreeDescriptor>& tree_descriptors)
{
	size_t tree_id = 0;

	// Create a point cloud for circle points
	ccPointCloud* circle_points_pc = new ccPointCloud(QString("Fitted sections"));
	circle_points_pc->copyGlobalShiftAndScale(*m_current_cloud);

	// Add scalar fields for circle properties
	int tree_id_sf_id    = circle_points_pc->addScalarField("tree_ID");
	int radius_sf_id     = circle_points_pc->addScalarField("radius");
	int height_sf_id     = circle_points_pc->addScalarField("height");
	int status_sf_id     = circle_points_pc->addScalarField("status");
	int num_points_sf_id = circle_points_pc->addScalarField("number of points inner circle");
	int sector_sf_id     = circle_points_pc->addScalarField("sector coverage");
	int outlier_sf_id    = circle_points_pc->addScalarField("outlier probability");

	auto* tree_id_sf    = circle_points_pc->getScalarField(tree_id_sf_id);
	auto* radius_sf     = circle_points_pc->getScalarField(radius_sf_id);
	auto* height_sf     = circle_points_pc->getScalarField(height_sf_id);
	auto* status_sf     = circle_points_pc->getScalarField(status_sf_id);
	auto* num_points_sf = circle_points_pc->getScalarField(num_points_sf_id);
	auto* sector_sf     = circle_points_pc->getScalarField(sector_sf_id);
	auto* outlier_sf    = circle_points_pc->getScalarField(outlier_sf_id);

	for (const auto& tree_data : tree_descriptors)
	{
		for (const auto& circle_data : tree_data.circle_data)
		{
			// Only draw successful circles
			if (circle_data.status >= lib3dfin::CircleData::Status::SUCCESS
			    && circle_data.status != lib3dfin::CircleData::Status::DIAMETER_TOO_LARGE
			    && circle_data.status != lib3dfin::CircleData::Status::DIAMETER_TOO_SMALL) // Maybe we could display diameters too small, it wont hurt
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

void cc3DFin::drawAxis(const std::vector<lib3dfin::TreeDescriptor>& tree_descriptors)
{
	// TODO: go back to point cloud sampling since we need to visualize the tilt deviation scalar field
	// Create a group to hold all axes as polylines
	constexpr double step_size = 0.1; // TODO parameters

	ccPointCloud* axis_points  = new ccPointCloud(QString("tree axes"));
	int           axis_tilt_id = axis_points->addScalarField("tilting_degree");
	auto          axis_tilt_sf = axis_points->getScalarField(axis_tilt_id);
	size_t        tree_id      = 0;

	for (const auto& desc : tree_descriptors)
	{
		const Eigen::Vector3d axis_step      = step_size * desc.axis; // TODO parameters
		const double          tilting_degree = desc.axis_vertical_deviation;
		Eigen::Vector3d       bottom_point   = desc.bottom_point;
		Eigen::Vector3d       top_point      = desc.top_point;
		Eigen::Vector3d       curr_point     = bottom_point;
		while (curr_point.z() < top_point.z())
		{
			axis_points->addPoint(CCVector3(curr_point.x(), curr_point.y(), curr_point.z()));
			curr_point += axis_step;
			axis_tilt_sf->addElement(tilting_degree);
		}
	}
	axis_tilt_sf->computeMinAndMax();
	axis_points->setCurrentDisplayedScalarField(axis_tilt_id);
	axis_points->toggleSF();
	axis_points->setEnabled(false);
	m_base_group->addChild(axis_points);
}
// https://github.com/3DFin/3DFin/blob/main/src/three_d_fin/cloudcompare/plugin_processing.py
void cc3DFin::drawTreeLocators(const std::vector<lib3dfin::TreeDescriptor>& tree_descriptors)
{
	ccPointCloud* tree_locations = new ccPointCloud("Tree Locations");
	tree_locations->copyGlobalShiftAndScale(*m_current_cloud);

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
	dbh_sf->computeMinAndMax();
	tree_locations->toggleColors();
	m_base_group->addChild(tree_locations);
}

void cc3DFin::drawTreeHeights(const std::vector<lib3dfin::TreeDescriptor>& tree_descriptors)
{
	ccPointCloud* tree_heights = new ccPointCloud("Highest points");
	tree_heights->copyGlobalShiftAndScale(*m_current_cloud);

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
	z0_sf->computeMinAndMax();
	deviated_sf->computeMinAndMax();
	tree_heights->toggleColors();
	m_base_group->addChild(tree_heights);
}

void cc3DFin::exportEnrichedCloud(const lib3dfin::TreeData& tree_data, const std::vector<double>& z0)
{
	ccPointCloud* enriched_cloud = new ccPointCloud(m_current_cloud->getName());

	enriched_cloud->copyGlobalShiftAndScale(*m_current_cloud);

	if (!enriched_cloud->reserve(m_current_cloud->size()))
	{
		ccLog::Error("[3DFin] Not enough memory!");
		delete enriched_cloud;
		return;
	}

	int dist_axes_id = enriched_cloud->addScalarField("dist_axes");
	int tree_id_id   = enriched_cloud->addScalarField("tree_ID");
	int z0_id        = enriched_cloud->addScalarField("Z0");

	auto dist_axes_sf = enriched_cloud->getScalarField(dist_axes_id);
	auto tree_id_sf   = enriched_cloud->getScalarField(tree_id_id);
	auto z0_sf        = enriched_cloud->getScalarField(z0_id);

	try
	{
		dist_axes_sf->reserve(enriched_cloud->size());
		tree_id_sf->reserve(enriched_cloud->size());
		z0_sf->reserve(enriched_cloud->size());
	}
	catch (const std::bad_alloc)
	{
		ccLog::Error("[3DFin] Failed to reserve memory for scalar fields");
		delete enriched_cloud;
		return;
	}

	for (unsigned i = 0; i < m_current_cloud->size(); i++)
	{
		enriched_cloud->addPoint(*m_current_cloud->getPoint(i));
		dist_axes_sf->addElement(tree_data.axis_distance(i));
		tree_id_sf->addElement(tree_data.cluster_indicator(i));
		z0_sf->addElement(z0[i]);
	}

	dist_axes_sf->computeMinAndMax();
	tree_id_sf->computeMinAndMax();
	z0_sf->computeMinAndMax();

	auto color_scale = ccColorScalesManager::GetUniqueInstance()->getScale(s_color_scale_uuid);

	enriched_cloud->setCurrentDisplayedScalarField(dist_axes_id);
	enriched_cloud->getCurrentDisplayedScalarField()->setColorScale(color_scale);
	enriched_cloud->toggleSF();
	enriched_cloud->setEnabled(false);

	m_base_group->addChild(enriched_cloud);
}

void cc3DFin::exportStripe(const std::vector<int32_t>& stem_indicator)
{
	ccPointCloud* stripe_cloud = new ccPointCloud("Stems in stripe");
	stripe_cloud->copyGlobalShiftAndScale(*m_current_cloud);
	int  tree_id_id = stripe_cloud->addScalarField("tree_ID");
	auto tree_id_sf = stripe_cloud->getScalarField(tree_id_id);

	unsigned count_valid = std::count_if(stem_indicator.begin(), stem_indicator.end(), [](int32_t stem_id)
	                                     { return stem_id >= 0; });

	if (!stripe_cloud->reserve(m_current_cloud->size()))
	{
		ccLog::Error("[3DFin] Not enough memory!");
		delete stripe_cloud;
		return;
	}

	try
	{
		tree_id_sf->reserve(stripe_cloud->size());
	}
	catch (const std::bad_alloc)
	{
		ccLog::Error("[3DFin] Failed to reserve memory for scalar fields");
		delete stripe_cloud;
		return;
	}

	for (size_t i = 0; i < stem_indicator.size(); i++)
	{
		auto stem_id = stem_indicator[i];
		if (stem_id >= 0)
		{
			stripe_cloud->addPoint(*m_current_cloud->getPoint(i));
			tree_id_sf->addElement(stem_id);
		}
	}

	tree_id_sf->computeMinAndMax();
	stripe_cloud->setCurrentDisplayedScalarField(tree_id_id);
	stripe_cloud->toggleSF();
	stripe_cloud->setEnabled(false);
	m_base_group->addChild(stripe_cloud);
}

void cc3DFin::compute3DFin(const lib3dfin::Params& params, std::shared_ptr<spdlog::logger> logger)
{
	QFuture<lib3dfin::TDFResult> TdfFutureResult = QtConcurrent::run(lib3dfin::process, &(m_current_cloud->getNextPoint()->u[0]), static_cast<size_t>(m_current_cloud->size()), params, logger);

	// Create watcher to notify when done
	// will be cleaned by using ::deleteLater()
	auto* TdfComputationWatcher = new QFutureWatcher<lib3dfin::TDFResult>(this);

	connect(TdfComputationWatcher, &QFutureWatcher<lib3dfin::TDFResult>::finished, this, [=]()
	        {
		// Retrieve result
		lib3dfin::TDFResult result = TdfComputationWatcher->future().result();

		auto& stem_indicator = std::get<0>(result);
		auto& z0             = std::get<1>(result);
		auto& tree_data      = std::get<2>(result);

		m_base_group.reset(new ccHObject(m_current_cloud->getName() + "_3DFin"));
		drawCircles(tree_data.tree_descriptors);
		drawTreeLocators(tree_data.tree_descriptors);
		drawTreeHeights(tree_data.tree_descriptors);
		drawAxis(tree_data.tree_descriptors);
		exportEnrichedCloud(tree_data, z0);
		exportStripe(stem_indicator);
		m_app->addToDB(m_base_group.release());
		m_app->redrawAll();

		TdfComputationWatcher->deleteLater(); });

	// TODO: error handling
	TdfComputationWatcher->setFuture(TdfFutureResult);
}
