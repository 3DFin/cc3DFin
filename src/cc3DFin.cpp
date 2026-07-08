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

// CCCore
#include "CCGeom.h"

// qCC

#include "ccColorScalesManager.h"
#include "ccHObject.h"
#include "ccHObjectCaster.h"
#include "ccLog.h"
#include "ccPointCloud.h"

// plugin
#include "cc3DFinDlg.h"
#include "cc3DFinDrawer.h"
#include "cc3DFinUiConfig.h"

// lib3DFin
#include <lib3DFin/config.hpp>
#include <lib3DFin/interface.hpp>
#include <lib3DFin/types.hpp>

// spdlog
#include <spdlog/sinks/qt_sinks.h>
#include <spdlog/spdlog.h>

// ccCoreLib
#include <ScalarField.h>

// QT
#include <QMainWindow>
#include <QtConcurrent>
#include <QtGui>

// StdLib
#include <span>

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
	if (m_action == nullptr)
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
	if (m_app == nullptr)
	{
		return;
	}

	// we need one point cloud
	if (!m_app->haveOneSelection())
	{
		ccLog::Error("Select only one cloud!");
		return;
	}

	// a real point cloud
	const ccHObject::Container& selectedEntities = m_app->getSelectedEntities();

	ccHObject* ent = selectedEntities[0];
	assert(ent);
	if (!ent->isA(CC_TYPES::POINT_CLOUD))
	{
		ccLog::Error("Select a cloud!");
		return;
	}

	// Cast to PC
	m_currentCloud = dynamic_cast<ccPointCloud*>(ent);

	// Get scalar field names for plugin UI
	QStringList scalarFieldNames;
	for (int i = 0; i < m_currentCloud->getNumberOfScalarFields(); ++i)
	{
		const CCCoreLib::ScalarField* scalarField = m_currentCloud->getScalarField(i);
		if (scalarField != nullptr)
			scalarFieldNames.push_back(QString::fromStdString(scalarField->getName()));
	}

	m_currentCloud->setEnabled(false);

	cc3DFinDlg tdfDlg(m_app->getMainWindow(), scalarFieldNames);

	int  maxLines = 2000;
	auto logger   = spdlog::qt_color_logger_mt("3DFin", tdfDlg.logTextEdit, maxLines);
	logger->set_pattern("[%T] %^[%L]%$ %v");
	spdlog::set_default_logger(logger);

	m_app->freezeUI(true);
	connect(tdfDlg.compute_btn, &QPushButton::clicked, [this, &tdfDlg]
	        {
				if(!tdfDlg.checkFieldsValidity())
				    return;
				const auto params    = tdfDlg.get3DFinParameters();
				tdfDlg.setComputationMode(true);
		        compute3DFin(params, tdfDlg); });
	tdfDlg.exec();

	// Cleanup. Drop the logger and unfreeze ui
	spdlog::drop("3DFin");
	m_app->freezeUI(false);

	QApplication::processEvents();
}

void cc3DFin::initCustomColorScale()
{
	auto maybeColorScale = ccColorScalesManager::GetUniqueInstance()->getScale(tdf::COLOR_SCALE_UUID);

	if (maybeColorScale != nullptr)
	{
		ccLog::Print("[3DFin] Color Scale already exists");
		return;
	}

	ccColorScale::Shared customColorScale = ccColorScale::Create("3DFin");
	customColorScale->setUuid(tdf::COLOR_SCALE_UUID);
	customColorScale->setRelative();

	customColorScale->insert(ccColorScaleElement(0., {91, 155, 213}));
	customColorScale->insert(ccColorScaleElement(1, {237, 125, 49}));
	customColorScale->insert(ccColorScaleElement(0.28571428571, {112, 173, 71}));
	customColorScale->insert(ccColorScaleElement(0.642857142857, {255, 192, 0}));

	ccColorScalesManager::GetUniqueInstance()->addScale(customColorScale);
}

void cc3DFin::compute3DFin(const lib3dfin::Params& params, cc3DFinDlg& dialog)
{
	assert(m_currentCloud);
	const QString cloudName     = m_currentCloud->getName();
	auto          baseOutputDir = dialog.checkBaseOutputValidity(cloudName);
	m_currentCloud->placeIteratorAtBeginning();

	// Convert Cloud GS into lib3DFin "exchange" format
	lib3dfin::GlobalShift tdfGlobalShift{
	    m_currentCloud->getGlobalShift().x,
	    m_currentCloud->getGlobalShift().y,
	    m_currentCloud->getGlobalShift().z,
	    m_currentCloud->getGlobalScale()};

	std::vector<double> tdfPointCloud;
	try
	{
		tdfPointCloud.reserve(m_currentCloud->size() * 3);
	}
	catch (const std::bad_alloc&)
	{
		ccLog::Error("[3DFin] Point cloud allocation failure (OoM)");
		return;
	}

	for (unsigned point_id = 0; point_id < m_currentCloud->size(); ++point_id)
	{
		const CCVector3* point = m_currentCloud->getPoint(point_id);
		tdfPointCloud.push_back(static_cast<double>(point->x));
		tdfPointCloud.push_back(static_cast<double>(point->y));
		tdfPointCloud.push_back(static_cast<double>(point->z));
	}

	std::unique_ptr<lib3dfin::TDFProcessing> tdfProcessing;

	tdfProcessing = std::make_unique<lib3dfin::TDFProcessing>(std::span<const double>(tdfPointCloud));

	std::vector<double> z0Vec;

	auto maybeZ0 = dialog.getZ0FieldName();
	if (maybeZ0.has_value())
	{
		const auto z0Name = maybeZ0.value();
		const auto z0Id   = m_currentCloud->getScalarFieldIndexByName(z0Name);
		if (z0Id != -1)
		{
			const auto* z0Sf = m_currentCloud->getScalarField(z0Id);

			// try to allocate the sf array
			try
			{
				z0Vec.reserve(z0Sf->size());
			}
			catch (const std::bad_alloc&)
			{
				ccLog::Error("[3DFin] Scalar field allocation failure (OoM)");
				return;
			}

			// copy the scalar_field to double
			for (size_t svId = 0; svId < z0Sf->size(); ++svId)
			{
				z0Vec.push_back(z0Sf->getValue(svId));
			}
		}
		tdfProcessing->setExternalZ0(std::span<const double>(z0Vec));
	}

	tdfProcessing->setParams(params);
	tdfProcessing->setGlobalShift(tdfGlobalShift);
	tdfProcessing->setOutputPath(baseOutputDir.value());

	auto TdfFutureResult = QtConcurrent::run(
	    [tdf = tdfProcessing.get()]
	    {
		    return tdf->process();
	    });

	// Create watcher to notify when its done
	// will be cleaned by using QFutureWatcher::deleteLater()
	auto* tdfComputationWatcher = new QFutureWatcher<lib3dfin::Status>(this);

	// we move z0vec and tdfPointCloud for memory clean up at the end of the computation.
	connect(tdfComputationWatcher, &QFutureWatcher<lib3dfin::Status>::finished, this, [tdfComputationWatcher, this, &dialog, tdfProcessing = std::move(tdfProcessing), z0Vec = std::move(z0Vec), tdfCloud = std::move(tdfPointCloud)]()
	        {


		if(tdfComputationWatcher->result() != lib3dfin::Status::Success)
		{
		    dialog.setComputationMode(false);
            tdfComputationWatcher->deleteLater();
		    return;
		}

		tdfProcessing->exportTabularData();

		cc3DFinDrawer drawer(m_currentCloud);
		m_base_group = drawer.drawAll(*tdfProcessing);

		dialog.setComputationMode(false);

		m_app->addToDB(m_base_group.release());
		m_app->redrawAll();

		tdfComputationWatcher->deleteLater(); });

	tdfComputationWatcher->setFuture(TdfFutureResult);
}
