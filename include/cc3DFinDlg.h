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
#include "cc3DFinUiConfig.h"
#include "ui_cc3dfindlg.h"

// lib3dfin
#include <lib3DFin/config.hpp>

// QT
#include <QDialog>
#include <QStringList>

// system
#include <filesystem>
#include <unordered_map>

namespace fs = std::filesystem;

//! Dialog for cc3DFin plugin
class cc3DFinDlg : public QDialog
    , public Ui::cc3DFinDlg
{
	Q_OBJECT

  public:
	//! Default constructor
	cc3DFinDlg(QWidget* parent, const QStringList& sfNames);

	//! Destructor
	~cc3DFinDlg() override = default;
	lib3dfin::Params        get3DFinParameters();
	bool                    checkFieldsValidity();
	void                    setComputationMode(bool);
	std::optional<fs::path> checkBaseOutputValidity(const QString& baseName);

  protected: // Methods
	void populateFields();
	void populateSfCombo();
	void populateToolTipAndLabel(const tdf::UiField& field, QWidget* widget);
	void closeEvent(QCloseEvent* event) override;

  protected: // Slots
      
	void        askOutputPath();
	void        showExpertDialog();
	void        onTextChanged();
	static void showDocumentation();
	static void showTutorial();

  protected: // Members
	const QStringList&                        m_scalarFields;
	std::unordered_map<QString, tdf::UiField> m_fields;
	QSet<QLineEdit*>                          m_InvalidEditFields;
	bool                                      m_isComputationActive{false};
};
