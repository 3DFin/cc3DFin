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

// Local
#include "cc3DFinDlg.h"

#include "cc3DFinConfig.h"
#include "cc3DFinExpertDlg.h"
#include "ccLog.h"
#include "ccSerializableObject.h"

// qCC_db
#include <ccPointCloud.h>

// Qt
#include <QApplication>
#include <QComboBox>
#include <QFileDialog>
#include <QMessageBox>
#include <QRadioButton>
#include <QtGui>

// System
#include <cassert>
#include <filesystem>
namespace fs = std::filesystem;

cc3DFinDlg::cc3DFinDlg(QWidget* parent, const QStringList& sfNames)
    : QDialog(parent, Qt::Dialog)
    , m_scalarFields(sfNames)
    , m_fields(tdf::Field::getConfigFields())
    , Ui::cc3DFinDlg()
{
	setupUi(this);

	connect(output_dir_btn, &QPushButton::clicked, this, &cc3DFinDlg::askOutputPath);
	connect(tutorial_link_btn, &QPushButton::clicked, this, &cc3DFinDlg::showTutorial);
	connect(documentation_link_btn, &QPushButton::clicked, this, &cc3DFinDlg::showDocumentation);
	connect(expert_info_btn, &QPushButton::clicked, this, &cc3DFinDlg::showExpertDialog);

	// Configure the logging tab
	logTextEdit->setReadOnly(true);
	connect(logTextEdit, &QTextEdit::textChanged, this, [=]()
	        { logTextEdit->moveCursor(QTextCursor::End); logTextEdit->ensureCursorVisible(); });

	populateFields();
}

void cc3DFinDlg::setComputationMode(bool state)
{
	if (state)
	{
		m_isComputationActive = true;
		tabWidget->setCurrentIndex(3); // switch to log tab
		compute_btn->setText("Computing...");
		tabWidget->tabBar()->setDisabled(true);
		bottomFrame->setDisabled(true);
	}
	else
	{
		m_isComputationActive = false;
		compute_btn->setText("Compute");
		tabWidget->tabBar()->setDisabled(false);
		bottomFrame->setDisabled(false);
	}
}

void cc3DFinDlg::closeEvent(QCloseEvent* event)
{
	if (m_isComputationActive)
	{
		// Prevent closing the dialog while computation is active.
		// we could show a dialog to inform the user...
		// ...we could implement a graceful cancelation inside the lib3DFin computation
		event->ignore();
	}
	else
	{
		event->accept();
	}
}

void cc3DFinDlg::onTextChanged()
{
	auto* lineEdit = qobject_cast<QLineEdit*>(sender());
	if (!lineEdit)
		return;

	if (lineEdit->hasAcceptableInput())
	{
		lineEdit->setStyleSheet("");
		lineEdit->setToolTip("");
		m_InvalidEditFields.remove(lineEdit);
	}
	else
	{
		lineEdit->setToolTip("Invalid value");
		lineEdit->setStyleSheet("border: 1px solid red;");
		m_InvalidEditFields.insert(lineEdit);
	}
}

void cc3DFinDlg::populateFields()
{
	QString homePath = QStandardPaths::writableLocation(QStandardPaths::HomeLocation);
	output_dir_in->setText(homePath);

	for (const auto [fieldName, field] : m_fields)
	{
		QWidget* widget = nullptr;
		// handle "corner cases" first
		if ((widget = findChild<QWidget*>(fieldName + "_rb_1")))
		{
			auto* rb1 = qobject_cast<QRadioButton*>(widget);
			auto* rb2 = qobject_cast<QRadioButton*>(findChild<QWidget*>(fieldName + "_rb_2"));
			if (rb1 && rb2)
			{
				rb1->setChecked(field.value.toBool());
				rb2->setChecked(!field.value.toBool());
				rb1->setToolTip(field.description);
				rb2->setToolTip(field.description);
				export_txt_lbl->setText(field.label);
				export_txt_lbl->setToolTip(field.description);
				rb1->setDisabled(true);
				rb2->setDisabled(true);
			}
			else
			{
				ccLog::PrintDebug("[3DFin] fail to configure export field");
			}
		}
		else if ((widget = findChild<QWidget*>(fieldName + "_chk")) != nullptr)
		{
			populateToolTipAndLabel(field, widget);
			if (auto* checkBox = qobject_cast<QCheckBox*>(widget))
			{
				checkBox->setChecked(field.value.toBool());
			}
			else
			{
				ccLog::PrintDebug("[3DFin] fail to configure checkbox field :" + fieldName);
			}
		}
		else if ((widget = findChild<QWidget*>(fieldName + "_in")))
		{
			populateToolTipAndLabel(field, widget);

			const auto& value    = field.value;
			QLineEdit*  lineEdit = qobject_cast<QLineEdit*>(widget);
			if (lineEdit)
			{
				lineEdit->setText(value.toString());
				if (value.type() == QVariant::Double)
				{
					auto validator = std::make_unique<QDoubleValidator>(widget);
					validator->setLocale(QLocale::c());
					if (field.topValue.type() == QVariant::Double)
					{
						validator->setTop(field.topValue.toDouble());
					}
					if (field.bottomValue.type() == QVariant::Double)
					{
						validator->setBottom(field.bottomValue.toDouble());
					}
					connect(lineEdit, &QLineEdit::textChanged, this, &cc3DFinDlg::onTextChanged);
					lineEdit->setValidator(validator.release());
				}
				else if (value.type() == QVariant::Int)
				{
					auto validator = std::make_unique<QIntValidator>(widget);
					validator->setLocale(QLocale::c());
					if (field.topValue.type() == QVariant::Int)
					{
						validator->setTop(field.topValue.toInt());
					}
					if (field.bottomValue.type() == QVariant::Int)
					{
						validator->setBottom(field.bottomValue.toInt());
					}
					connect(lineEdit, &QLineEdit::textChanged, this, &cc3DFinDlg::onTextChanged);
					lineEdit->setValidator(validator.release());
				}
			}
			else
			{
				ccLog::PrintDebug("[3DFin] fail to configure lineEdit field: " + fieldName);
			}
		}
	}
	populateSfCombo();
}

bool cc3DFinDlg::checkFieldsValidity()
{
	if (!m_InvalidEditFields.isEmpty())
	{
		QMessageBox::critical(this, "Invalid input", "Please correct Invalid fields");
		return false;
	}
	return true;
}

std::optional<fs::path> cc3DFinDlg::checkBaseOutputValidity(const QString& baseName)
{
	// get the output_path
	fs::path outPath = fs::path(output_dir_in->text().toStdString()) / fs::path(baseName.toStdString()).stem();

	if (!fs::exists(fs::path(outPath.string() + ".xlsx")))
		return outPath;

	auto userchoice = QMessageBox::question(this, "", "Results of previous computations exists in " + output_dir_in->text() + " do you want to ovewrite?");

	if (userchoice == QMessageBox::Yes)
		return outPath;
	else
		return std::nullopt;
}

lib3dfin::Params cc3DFinDlg::get3DFinParameters()
{
	// Collect params from the GUI
	lib3dfin::Params params;

	params.compute_height_normalization = compute_height_normalization_chk->isChecked();

	// Fields are pre validated, the casts should succeed

	params.cloth_resolution = cloth_resolution_in->text().toDouble();

	params.denoise_point_cloud    = denoise_point_cloud_chk->isChecked();
	params.denoise_resolution     = denoise_resolution_in->text().toDouble();
	params.denoise_minimum_points = denoise_minimum_points_in->text().toUInt();

	params.stripe_lower_limit = stripe_lower_limit_in->text().toDouble();
	params.stripe_upper_limit = stripe_upper_limit_in->text().toDouble();

	params.verticality_radius_stripe       = verticality_radius_stripe_in->text().toDouble();
	params.verticality_threshold_stripe    = verticality_threshold_stripe_in->text().toDouble();
	params.stripe_peeling_voxels_threshold = stripe_peeling_voxels_threshold_in->text().toInt();
	params.stripe_peeling_resolution_xy    = stripe_peeling_resolution_xy_in->text().toDouble();
	params.stripe_peeling_resolution_z     = stripe_peeling_resolution_z_in->text().toDouble();
	params.stripe_peeling_num_iterations   = stripe_peeling_num_iterations_in->text().toInt();

	params.tree_resolution_xy               = tree_resolution_xy_in->text().toDouble();
	params.tree_resolution_z                = tree_resolution_z_in->text().toDouble();
	params.tree_height_range                = tree_height_range_in->text().toDouble();
	params.tree_dist_axis_threshold         = tree_dist_axis_threshold_in->text().toDouble();
	params.tree_minimum_points_stem         = tree_minimum_points_stem_in->text().toUInt();
	params.tree_axis_max_vertical_deviation = tree_axis_max_vertical_deviation_in->text().toDouble();
	params.tree_height_distance_from_axis   = tree_height_distance_from_axis_in->text().toDouble();
	params.tree_resolution_height           = tree_resolution_height_in->text().toDouble();

	params.stem_search_diameter               = stem_search_diameter_in->text().toDouble();
	params.stem_minimum_height                = stem_minimum_height_in->text().toDouble();
	params.stem_maximum_height                = stem_maximum_height_in->text().toDouble();
	params.verticality_radius_stem            = verticality_radius_stem_in->text().toDouble();
	params.verticality_threshold_stem         = verticality_threshold_stem_in->text().toDouble();
	params.stem_section_thickness             = stem_section_thickness_in->text().toDouble();
	params.stem_section_circle_width          = stem_section_circle_width_in->text().toDouble();
	params.stem_section_interval              = stem_section_interval_in->text().toDouble();
	params.stem_section_clustering_distance   = stem_section_clustering_distance_in->text().toDouble();
	params.stem_section_minimum_diameter      = stem_section_minimum_diameter_in->text().toDouble();
	params.stem_section_maximum_diameter      = stem_section_maximum_diameter_in->text().toDouble();
	params.stem_section_inner_point_threshold = stem_section_inner_point_threshold_in->text().toInt();
	params.stem_section_min_points            = stem_section_min_points_in->text().toInt();
	params.stem_section_diameter_proportion   = stem_section_diameter_proportion_in->text().toDouble();
	params.stem_section_sector_count          = stem_section_sector_count_in->text().toInt();
	params.stem_section_min_occupied_sectors  = stem_section_min_occupied_sectors_in->text().toInt();

	return params;
}

void cc3DFinDlg::askOutputPath()
{
	QString initialPathText = output_dir_in->text();
	QDir    initialPath(initialPathText);

	bool hasValidInitialDir = initialPath.exists() && initialPath.isReadable();
	QDir initialDir         = hasValidInitialDir ? initialPath : QDir(QStandardPaths::writableLocation(QStandardPaths::HomeLocation));

	output_dir_in->setText(initialDir.absolutePath());
	QFileDialog dialog(this, "3DFin output directory");
	dialog.setFileMode(QFileDialog::Directory);
	dialog.setOption(QFileDialog::ShowDirsOnly, true);
	if (dialog.exec() == QDialog::Accepted)
	{
		QString outputDir = dialog.selectedFiles().first();
		if (!outputDir.isEmpty())
		{
			output_dir_in->setText(QDir(outputDir).absolutePath());
		}
	}
}

void cc3DFinDlg::showTutorial()
{
	QDesktopServices::openUrl(QUrl("https://github.com/3DFin/3DFin_Tutorial/"));
}

void cc3DFinDlg::showDocumentation()
{
	QFile pdfFile(":3dfin/assets/documentation.pdf");

	if (!pdfFile.open(QIODevice::ReadOnly))
	{
		ccLog::Error("Failed to open 3DFin PDF!");
		return;
	}

	// We need to create a temporary file to open with system PDF viewer
	QString tempPath  = QStandardPaths::writableLocation(QStandardPaths::TempLocation);
	QString outputPdf = tempPath + QDir::separator() + "3dfin_documentation.pdf";

	QFile outFile(outputPdf);
	if (!outFile.open(QIODevice::WriteOnly))
	{
		ccLog::Error("Failed to write temporary 3DFin PDF!");
		return;
	}

	outFile.write(pdfFile.readAll());
	outFile.close();

	// Open with system PDF viewer
	QDesktopServices::openUrl(QUrl::fromLocalFile(outputPdf));
}

void cc3DFinDlg::showExpertDialog()
{
	cc3DFinExpertDlg dialog(this);
	dialog.exec();
}

void cc3DFinDlg::populateSfCombo()
{
	// populate scalar field list
	if (m_scalarFields.empty())
	{
		compute_height_normalization_chk->setChecked(true);
		compute_height_normalization_chk->setDisabled(true);
		return;
	}

	z0_name_cbx->addItems(m_scalarFields);
	const auto& sfComboField = m_fields["z0_name"];
	int         z0index      = z0_name_cbx->findText("Z0");
	if (z0index != -1)
	{
		z0_name_cbx->setCurrentIndex(z0index);
	}
	else
	{
		compute_height_normalization_chk->setChecked(true);
	}
	z0_name_lbl->setToolTip(sfComboField.description);
	z0_name_cbx->setToolTip(sfComboField.description);
}

void cc3DFinDlg::populateToolTipAndLabel(const tdf::Field& field, QWidget* widget)
{
	assert(widget);
	const auto& tooltip = field.description;
	if (!tooltip.isEmpty())
	{
		auto* label = findChild<QLabel*>(field.name + "_lbl");
		if (label != nullptr)
			label->setToolTip(tooltip);
	}

	const auto& hint      = field.hint;
	auto*       hintLabel = findChild<QLabel*>(field.name + "_ht");
	if (hintLabel != nullptr && !hint.isEmpty())
	{
		hintLabel->setText(hint);
	}
}
