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

// qCC_db
#include <ccPointCloud.h>
// qCC_gl
#include <ccGLWindowInterface.h>

// Qt
#include <QApplication>
#include <QtGui>
#include <QComboBox>

cc3DFinDlg::cc3DFinDlg(QWidget* parent)
    : QDialog(parent, Qt::Tool)
    , Ui::cc3DFinDlg()
{
	setupUi(this);
	populateFields();
}

void cc3DFinDlg::populateFields()
{
	for (const auto field : tdf::Field::getConfigFields())
	{

		QWidget* widget = findChild<QWidget*>(field.name + "_in");
		if (!widget)
			continue;

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

		const auto& value = field.value;
		if (QLineEdit* lineEdit = qobject_cast<QLineEdit*>(widget))
		{
			lineEdit->setText(value.toString());

			if (value.type() == QVariant::Double)
			{
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

		else if (QCheckBox* checkBox = qobject_cast<QCheckBox*>(widget))
		{
			checkBox->setChecked(value.toBool());
		}

		else if (QComboBox* comboBox = qobject_cast<QComboBox*>(widget))
		{
			int index = comboBox->findText(value.toString());
			if (index != -1)
				comboBox->setCurrentIndex(index);
		}
	}
}
