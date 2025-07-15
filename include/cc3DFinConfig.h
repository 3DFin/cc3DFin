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

#include <QVariant>
#include <qchar.h>
#include <qvariant.h>
#include <vector>

namespace tdf
{
	//! 3DFin configuration
	struct Field
	{
		QString  name;
		QString  label;
		QString  description;
		QVariant value;
		QString  hint;

		static const std::vector<Field> getConfigFields();
	};

} // namespace tdf
