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

#include "ui_cc3dfindlg.h"

#include <QDialog>

//! Dialog for cc3DFin plugin
class cc3DFinDlg : public QDialog
    , public Ui::cc3DFinDlg
{
	Q_OBJECT

  public:
	//! Default constructor
	cc3DFinDlg(QWidget* parent = nullptr);

	//! Destrcuctor
	virtual ~cc3DFinDlg() override = default;

  protected: // methods
  protected: // members
};
