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

// Ui
#include <ui_cc3DFinExpertDlg.h>

// Qt
#include <QDialog>

//! Dialog for cc3DFin plugin
class cc3DFinExpertDlg : public QDialog
{
	Q_OBJECT

  public:
	//! Default constructor
	explicit cc3DFinExpertDlg(QWidget* parent = nullptr)
	    : QDialog(parent)
	    , m_ui(std::make_unique<Ui::cc3DfinExpertDlg>())
	{
		m_ui->setupUi(this);
		connect(m_ui->buttonBox, &QDialogButtonBox::accepted, this, &QDialog::accept);
	}

	//! Destructor
	~cc3DFinExpertDlg() override = default;

  private:
	std::unique_ptr<Ui::cc3DfinExpertDlg> m_ui;
};
