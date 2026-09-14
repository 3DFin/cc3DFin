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
#include "ui_cc3DFinExpertDlg.h"

// QT
#include <QDialog>

//! Dialog for cc3DFin plugin
class cc3DFinExpertDlg : public QDialog
    , public Ui::cc3DfinExpertDlg
{
	Q_OBJECT

  public:
	//! Default constructor
	cc3DFinExpertDlg(QWidget* parent = nullptr)
	    : QDialog(parent)
	    , Ui::cc3DfinExpertDlg()
	{
		setupUi(this);
        connect(buttonBox, &QDialogButtonBox::accepted, this, &QDialog::accept);
	}

	//! Destructor
	~cc3DFinExpertDlg() override = default;
};
