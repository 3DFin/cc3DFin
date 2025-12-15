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

// local
#include "cc3DFinConfig.h"
#include "ui_cc3dfindlg.h"

// lib3dfin
#include <lib3DFin/config.hpp>

// QT
#include <QDialog>
#include <QStringList>

// system
#include <unordered_map>

//! Dialog for cc3DFin plugin
class cc3DFinDlg : public QDialog
    , public Ui::cc3DFinDlg
{
	Q_OBJECT

  public:
	//! Default constructor
	cc3DFinDlg(QWidget* parent, const QStringList& sfNames);

	//! Destrcuctor
	virtual ~cc3DFinDlg() override = default;
	lib3dfin::Params get3DFinParameters();
	void             setComputationMode(bool);

  protected: // methods
	void populateFields();
	void populateSfCombo();
	void populateToolTipAndLabel(const tdf::Field& field, QWidget* widget);

  protected slots: // slots
	void askOutputPath();
	void showTutorial();
	void showDocumentation();
	void showExpertDialog();

  protected: // members
	const QStringList&                      m_scalarFields;
	std::unordered_map<QString, tdf::Field> m_fields;
};
