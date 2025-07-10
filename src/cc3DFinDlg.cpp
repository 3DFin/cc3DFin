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

// qCC_db
#include <ccPointCloud.h>
// qCC_gl
#include <ccGLWindowInterface.h>

// Qt
#include <QApplication>
#include <QtGui>

cc3DFinDlg::cc3DFinDlg(QWidget* parent)
    : QDialog(parent, Qt::Tool)
    , Ui::cc3DFinDlg()
{
	setupUi(this);
}
