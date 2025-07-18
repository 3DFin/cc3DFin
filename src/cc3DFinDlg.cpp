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
#include <QRadioButton>
#include <QtGui>

// System
#include <cassert>

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
	connect(compute_btn, &QPushButton::clicked, this, &cc3DFinDlg::computeClicked);

	populateFields();
}

void cc3DFinDlg::populateFields()
{
	QString homePath = QStandardPaths::writableLocation(QStandardPaths::HomeLocation);
	output_dir_in->setText(homePath);

	populateSfCombo();

	for (const auto [fieldName, field] : m_fields)
	{
		QWidget* widget = nullptr;

		// handle "corner" cases fist
		if ((widget = findChild<QWidget*>(fieldName + "_rb_1")))
		{
			QRadioButton* rb1 = qobject_cast<QRadioButton*>(widget);
			QRadioButton* rb2 = qobject_cast<QRadioButton*>(findChild<QWidget*>(fieldName + "_rb_2"));
			if (rb1 && rb2)
			{
				rb1->setChecked(field.value.toBool());
				rb2->setChecked(!field.value.toBool());
				rb1->setToolTip(field.description);
				rb2->setToolTip(field.description);
				export_txt_lbl->setText(field.value.toString());
				export_txt_lbl->setToolTip(field.description);
			}
			else
			{
				ccLog::Warning("[3DFin] fail to configure export field");
			}
		}
		else if ((widget = findChild<QWidget*>(fieldName + "_chk")))
		{
			populateToolTipAndLabel(field, widget);
			if (QCheckBox* checkBox = qobject_cast<QCheckBox*>(widget))
			{
				checkBox->setChecked(field.value.toBool());
			}
			else
			{
				ccLog::Warning("[3DFin] fail to configure checkbox field :" + fieldName);
			}
		}
		else if ((widget = findChild<QWidget*>(fieldName + "_in")))
		{
			populateToolTipAndLabel(field, widget);

			const auto& value = field.value;
			if (QLineEdit* lineEdit = qobject_cast<QLineEdit*>(widget))
			{
				lineEdit->setText(value.toString());

				if (value.type() == QVariant::Double)
				{
					// TODO gt
					auto* validator = new QDoubleValidator(this);
					validator->setLocale(QLocale::c());
					lineEdit->setValidator(validator);
				}
				else if (value.type() == QVariant::Int)
				{
					auto* validator = new QIntValidator(this);
					validator->setLocale(QLocale::c());
					lineEdit->setValidator(validator);
				}
			}
			else
			{
				ccLog::Warning("[3DFin] fail to configure lineEdit field: " + fieldName);
			}
		}
	}
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
	dialog.setOption(QFileDialog::DontUseNativeDialog, true); // TODO: issue with native dialog on macOS
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

void cc3DFinDlg::computeClicked()
{
    //do nothing for now
}

void cc3DFinDlg::populateSfCombo()
{
	// populate scalar field list
	z0_name_cbx->addItems(m_scalarFields);
	const auto& sfComboField = m_fields["z0_name"];
	int         z0index      = z0_name_cbx->findText("Z0");
	if (z0index != -1)
	{
		z0_name_cbx->setCurrentIndex(z0index);
	}
	else
	{
		m_fields["do_normalize"].value = true;
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
		widget->setToolTip(tooltip);

		QLabel* label = findChild<QLabel*>(field.name + "_lbl");
		if (label)
			label->setToolTip(tooltip);
	}

	const auto& hint      = field.hint;
	QLabel*     hintLabel = findChild<QLabel*>(field.name + "_ht");
	if (hintLabel && !hint.isEmpty())
	{
		hintLabel->setText(hint);
	}
}
