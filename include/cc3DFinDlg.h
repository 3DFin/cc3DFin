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

// Local
#include "cc3DFinUiConfig.h"

// lib3dfin
#include <lib3DFin/config.hpp>

// Qt
#include <QDialog>
#include <QStringList>

// System
#include <filesystem>
#include <unordered_map>

// Forward declaration
namespace Ui
{
	class cc3DFinDlg;
}

class QLineEdit;

namespace fs = std::filesystem;

//! Dialog for cc3DFin plugin
class cc3DFinDlg : public QDialog
{
	Q_OBJECT

  public:
	//! Default constructor
	cc3DFinDlg(QWidget* parent, const QStringList& sfNames);

	//! Destructor
	~cc3DFinDlg() override;

	QPushButton* getComputeButton();

	lib3dfin::Params           get3DFinParameters();
	bool                       checkFieldsValidity();
	void                       setComputationMode(bool);
	std::optional<fs::path>    checkBaseOutputValidity(const QString& baseName);
	std::optional<std::string> getZ0FieldName() const;

  protected:
	//! Methods
	void populateFields();
	void populateSfCombo();
	void populateToolTipAndLabel(const tdf::UiField& field, QWidget* widget);
	void closeEvent(QCloseEvent* event) override;

	//! Slots
	void        askOutputPath();
	void        showExpertDialog();
	void        onTextChanged();
	static void showDocumentation();
	static void showTutorial();

	//! Members
	const QStringList&                        m_scalarFields;
	std::unordered_map<QString, tdf::UiField> m_fields;
	QSet<QLineEdit*>                          m_InvalidEditFields;
	bool                                      m_isComputationActive{false};
	std::unique_ptr<Ui::cc3DFinDlg>           m_ui;
};
