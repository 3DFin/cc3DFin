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

#include "cc3DFinDrawer.h"

// CCCore
#include "CCGeom.h"

// qCC
#include "cc2DLabel.h"
#include "ccColorScalesManager.h"
#include "ccHObject.h"
#include "ccLog.h"
#include "ccMesh.h"
#include "ccPointCloud.h"
#include "ccScalarField.h"

// plugin
#include "cc3DFinUiConfig.h"

// lib3DFin
#include <lib3DFin/config.hpp>
#include <lib3DFin/interface.hpp>
#include <lib3DFin/types.hpp>

// spdlog
#include <spdlog/spdlog.h>

// ccCoreLib
#include <ScalarField.h>

cc3DFinDrawer::cc3DFinDrawer(ccPointCloud* sourceCloud)
    : m_source(sourceCloud)
{
}

std::unique_ptr<ccHObject> cc3DFinDrawer::drawAll(const lib3dfin::TDFProcessing& result)
{
	assert(m_source);

	m_group = std::make_unique<ccHObject>(m_source->getName() + "_3DFIn");

	const auto& stemIndicator = result.getStemIndicator();
	const auto& z0Out         = result.getZ0();
	const auto& treeData      = result.getTreeData();
	const auto& dtm           = result.getDTM();

	// DTM is optional. It depends if we compute height normalization in the process.
	if (!dtm.tri_ids.empty())
	{
		drawDTM(dtm);
	}

	drawCircles(treeData.tree_descriptors);
	drawTreeLocators(treeData.tree_descriptors);
	drawTreeHeights(treeData.tree_descriptors);
	drawAxis(treeData.tree_descriptors);
	exportEnrichedCloud(treeData, z0Out);
	exportStripe(stemIndicator);

	return std::move(m_group);
}

void cc3DFinDrawer::drawDTM(const lib3dfin::DTMData& dtm)
{

	auto& triIds   = dtm.tri_ids;
	auto& dtmMask  = dtm.dtm_mask;
	auto& dtmCloud = dtm.dtm;

	ccPointCloud* fullVertices     = new ccPointCloud("DTM vertices");
	ccPointCloud* filteredVertices = new ccPointCloud("DTM vertices");

	int   maskSfId = fullVertices->addScalarField("Invalid ground");
	auto* maskSf   = fullVertices->getScalarField(maskSfId);

	ccMesh* fullMesh = new ccMesh(fullVertices);
	fullMesh->setName("DTM mesh");
	fullMesh->addChild(fullVertices);

	ccMesh* filteredMesh = new ccMesh(filteredVertices);
	filteredMesh->setName("DTM mesh (filtered)");
	filteredMesh->addChild(filteredVertices);

	fullMesh->copyGlobalShiftAndScale(*m_source);
	fullVertices->copyGlobalShiftAndScale(*m_source);
	fullVertices->setEnabled(false);

	if (!fullVertices->reserve(dtmCloud.size()) || !filteredVertices->reserve(dtmCloud.size())
	    || !fullMesh->reserve(triIds.size() / 3) || !filteredMesh->reserve(triIds.size() / 3))
	{
		ccLog::Error("[3DFin] Unable to initialize DTM entity");
		delete filteredMesh;
		filteredMesh = nullptr;
		delete fullMesh;
		fullMesh = nullptr;
		return;
	}

	std::map<size_t, size_t> pointRemapping;

	size_t valid_point_id = 0;
	for (size_t point_id = 0; point_id < dtmCloud.rows(); point_id++)
	{
		const auto& point = dtmCloud.row(point_id);
		CCVector3   ccPoint(point.x(), point.y(), point.z());
		fullVertices->addPoint(ccPoint);
		maskSf->addElement(static_cast<double>(!dtmMask[point_id]));

		if (dtmMask[point_id])
		{
			filteredVertices->addPoint(ccPoint);
			pointRemapping[point_id] = valid_point_id++;
		}
	}

	for (size_t tri_id = 0; tri_id < (triIds.size() / 3); ++tri_id)
	{
		size_t triStart = tri_id * 3;
		size_t a        = triIds[triStart];
		size_t b        = triIds[triStart + 1];
		size_t c        = triIds[triStart + 2];
		fullMesh->addTriangle(a, b, c);
		if (dtmMask[a] && dtmMask[b] && dtmMask[c])
		{
			filteredMesh->addTriangle(pointRemapping[a], pointRemapping[b], pointRemapping[c]);
		}
	}

	fullMesh->computeNormals(false);
	fullMesh->setEnabled(false);
	filteredMesh->showWired(true);
	filteredMesh->setEnabled(false);
	maskSf->computeMinAndMax();
	fullVertices->setCurrentDisplayedScalarField(maskSfId);
	fullMesh->toggleSF();
	fullVertices->toggleSF();
	filteredMesh->shrinkToFit();
	filteredVertices->shrinkToFit();

	m_group->addChild(filteredMesh);
	m_group->addChild(fullMesh);
}

void cc3DFinDrawer::drawCircles(const std::vector<lib3dfin::TreeDescriptor>& tree_descriptors)
{
	size_t tree_id = 0;

	// Create a point cloud for circle points
	ccPointCloud* circle_points_pc = new ccPointCloud(QString("Fitted sections"));
	circle_points_pc->copyGlobalShiftAndScale(*m_source);

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
			    && circle_data.status != lib3dfin::CircleData::Status::DIAMETER_TOO_SMALL) // Maybe we could display diameters too small, it wont hurt the display scale/
			{
				const auto& circle = circle_data.circle;
				// We need to shift the circle center to go from z0 coordinates to the actual coordinates
				const double height   = circle_data.z0 + tree_descriptors[tree_id].dims.height_difference;
				const auto   f_height = static_cast<PointCoordinateType>(height);

				// Add circle center
				circle_points_pc->addPoint({static_cast<PointCoordinateType>(circle.center.x()),
				                            static_cast<PointCoordinateType>(circle.center.y()),
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

					circle_points_pc->addPoint({static_cast<PointCoordinateType>(x),
					                            static_cast<PointCoordinateType>(y),
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
		m_group->addChild(circle_points_pc);
	}
}

void cc3DFinDrawer::drawAxis(const std::vector<lib3dfin::TreeDescriptor>& tree_descriptors)
{
	const double step_size = 0.1; // TODO(RJ) parameters

	auto*  axis_points  = new ccPointCloud(QString("Tree axes"));
	int    axis_tilt_id = axis_points->addScalarField("tilting_degree");
	auto*  axis_tilt_sf = axis_points->getScalarField(axis_tilt_id);
	size_t tree_id      = 0;

	for (const auto& desc : tree_descriptors)
	{
		const Eigen::Vector3d  axis_step      = step_size * desc.axis.direction; // TODO(RJ) parameters
		const double           tilting_degree = desc.axis.vertical_deviation_deg;
		const Eigen::Vector3d& top_point      = desc.axis.top_point;
		Eigen::Vector3d        curr_point     = desc.axis.bottom_point;
		while (curr_point.z() < top_point.z())
		{
			axis_points->addPoint({static_cast<PointCoordinateType>(curr_point.x()),
			                       static_cast<PointCoordinateType>(curr_point.y()),
			                       static_cast<PointCoordinateType>(curr_point.z())});
			curr_point += axis_step;
			axis_tilt_sf->addElement(tilting_degree);
		}
	}

	axis_tilt_sf->computeMinAndMax();
	axis_points->setCurrentDisplayedScalarField(axis_tilt_id);
	axis_points->toggleSF();
	axis_points->setEnabled(false);
	m_group->addChild(axis_points);
}

void cc3DFinDrawer::drawTreeLocators(const std::vector<lib3dfin::TreeDescriptor>& tree_descriptors)
{
	auto* tree_locations = new ccPointCloud("Tree Locations");
	tree_locations->copyGlobalShiftAndScale(*m_source);

	tree_locations->setPointSize(8);
	int    id_dbh  = tree_locations->addScalarField("dbh");
	auto*  dbh_sf  = tree_locations->getScalarField(id_dbh);
	size_t tree_id = 0;

	for (const auto& desc : tree_descriptors)
	{
		tree_locations->addPoint({static_cast<PointCoordinateType>(desc.location.position.x()),
		                          static_cast<PointCoordinateType>(desc.location.position.y()),
		                          static_cast<PointCoordinateType>(desc.location.position.z())});
		dbh_sf->addElement(desc.dims.dbh);
		auto* label = new cc2DLabel;
		label->addPickedPoint(tree_locations, tree_id);
		if (desc.dims.dbh < std::numeric_limits<float>::epsilon())
		{
			label->setName(QString("Tree %1 | Not Reliable").arg(tree_id + 1));
		}
		else
		{
			label->setName(QString("Tree %1 | %2").arg(tree_id + 1).arg(desc.dims.dbh));
		}
		label->displayPointLegend(true);
		label->toggleVisibility();
		label->setDisplayedIn2D(false);
		tree_locations->addChild(label);
		++tree_id;
	}
	dbh_sf->computeMinAndMax();
	tree_locations->setColor(255, 0, 255);
	tree_locations->toggleColors();
	m_group->addChild(tree_locations);
}

void cc3DFinDrawer::drawTreeHeights(const std::vector<lib3dfin::TreeDescriptor>& tree_descriptors)
{
	auto* tree_heights = new ccPointCloud("Highest points");
	tree_heights->copyGlobalShiftAndScale(*m_source);
	tree_heights->setPointSize(8);
	int    id_z0       = tree_heights->addScalarField("z0");
	auto*  z0_sf       = tree_heights->getScalarField(id_z0);
	int    id_deviated = tree_heights->addScalarField("deviated");
	auto*  deviated_sf = tree_heights->getScalarField(id_deviated);
	size_t tree_id     = 0;
	tree_heights->setPointSize(8);

	// add labels with z0 values
	for (const auto& desc : tree_descriptors)
	{
		tree_heights->addPoint({static_cast<PointCoordinateType>(desc.dims.highest_point.x()),
		                        static_cast<PointCoordinateType>(desc.dims.highest_point.y()),
		                        static_cast<PointCoordinateType>(desc.dims.highest_point.z())});
		z0_sf->addElement(desc.dims.highest_z0);
		deviated_sf->addElement(static_cast<double>(desc.axis.valid));
		auto* label = new cc2DLabel(QString("point %1").arg(tree_id + 1));
		label->addPickedPoint(tree_heights, tree_id);
		label->setName(QString::number(desc.dims.highest_z0));
		label->displayPointLegend(true);
		label->toggleVisibility();
		label->setDisplayedIn2D(false);
		tree_heights->addChild(label);
		++tree_id;
	}
	z0_sf->computeMinAndMax();
	deviated_sf->computeMinAndMax();
	tree_heights->setColor(255, 0, 255);
	tree_heights->toggleColors();
	m_group->addChild(tree_heights);
}

void cc3DFinDrawer::exportEnrichedCloud(const lib3dfin::TreeData& tree_data, const Eigen::VectorXd& z0)
{
	auto enriched_cloud = std::make_unique<ccPointCloud>(m_source->getName());

	enriched_cloud->copyGlobalShiftAndScale(*m_source);

	if (!enriched_cloud->reserve(m_source->size()))
	{
		ccLog::Error("[3DFin] Not enough memory!");
		return;
	}

	int dist_axes_id = enriched_cloud->addScalarField("dist_axes");
	int tree_id_id   = enriched_cloud->addScalarField("tree_ID");
	int z0_id        = enriched_cloud->addScalarField("Z0");

	auto* dist_axes_sf = enriched_cloud->getScalarField(dist_axes_id);
	auto* tree_id_sf   = enriched_cloud->getScalarField(tree_id_id);
	auto* z0_sf        = enriched_cloud->getScalarField(z0_id);

	try
	{
		dist_axes_sf->reserve(enriched_cloud->size());
		tree_id_sf->reserve(enriched_cloud->size());
		z0_sf->reserve(enriched_cloud->size());
	}
	catch (const std::bad_alloc&)
	{
		ccLog::Error("[3DFin] Failed to reserve memory for scalar fields");
		return;
	}

	for (unsigned i = 0; i < m_source->size(); i++)
	{
		enriched_cloud->addPoint(*m_source->getPoint(i));
		dist_axes_sf->addElement(tree_data.axis_distance(i));
		tree_id_sf->addElement(tree_data.tree_cluster_indicator(i));
		z0_sf->addElement(z0(i));
	}

	dist_axes_sf->computeMinAndMax();
	tree_id_sf->computeMinAndMax();
	z0_sf->computeMinAndMax();

	auto color_scale = ccColorScalesManager::GetUniqueInstance()->getScale(tdf::COLOR_SCALE_UUID);

	enriched_cloud->setCurrentDisplayedScalarField(dist_axes_id);
	enriched_cloud->getCurrentDisplayedScalarField()->setColorScale(color_scale);
	enriched_cloud->toggleSF();
	enriched_cloud->setEnabled(false);

	m_group->addChild(enriched_cloud.release());
}

void cc3DFinDrawer::exportStripe(const lib3dfin::ArrayClusterIndicator& stem_indicator)
{
	auto stripe_cloud = std::make_unique<ccPointCloud>("Stems in stripe");

	stripe_cloud->copyGlobalShiftAndScale(*m_source);
	int   tree_id_id = stripe_cloud->addScalarField("tree_ID");
	auto* tree_id_sf = stripe_cloud->getScalarField(tree_id_id);

	unsigned count_valid = std::count_if(std::begin(stem_indicator), std::end(stem_indicator), [](int32_t stem_id)
	                                     { return stem_id >= 0; });

	if (!stripe_cloud->reserve(m_source->size()))
	{
		ccLog::Error("[3DFin] Not enough memory!");
		return;
	}

	try
	{
		tree_id_sf->reserve(stripe_cloud->size());
	}
	catch (const std::bad_alloc)
	{
		ccLog::Error("[3DFin] Failed to reserve memory for scalar fields");
		return;
	}

	for (size_t i = 0; i < stem_indicator.size(); i++)
	{
		auto stem_id = stem_indicator(i);
		if (stem_id >= 0)
		{
			stripe_cloud->addPoint(*m_source->getPoint(i));
			tree_id_sf->addElement(stem_id);
		}
	}

	tree_id_sf->computeMinAndMax();
	stripe_cloud->setCurrentDisplayedScalarField(tree_id_id);
	stripe_cloud->toggleSF();
	stripe_cloud->setEnabled(false);
	m_group->addChild(stripe_cloud.release());
}
