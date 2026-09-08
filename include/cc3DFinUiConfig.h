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

#pragma once

// QT
#include <QString>
#include <QVariant>

// StdLib
#include <unordered_map>
#include <vector>

// lib3dfin
#include <lib3DFin/config.hpp>

namespace tdf
{

	inline const QString COLOR_SCALE_UUID = "{25ec76a1-9b8d-4e4a-a129-21ae313ef8ba}";

	//! UI field configuration for dialog fields
	struct UiField
	{
		QString  name;           // 3DFin field name (to match UI element)
		QString  label;          // Display labels for the field
		QString  description;    // Tooltip / description text
		QString  hint;           // Hint text, when needed ( like in python we have Lable-input-hint strucutre)
		QVariant defaultValue;   // Default value for the field
		QVariant bottomValue;    // Minimum value (for numeric fields)
		QVariant topValue;       // Maximum value (for numeric fields)
		bool     required{true}; // Whether the field is required or not
		QString  group;          // Group ("basic", "advanced", "expert", "misc"), to match tabs and future config file
	};

	//! Field group categories
	struct FieldGroup
	{
		QString              name;
		QString              displayName;
		std::vector<QString> fieldNames;
	};

	//! UI configuration manager
	class UiConfig
	{

	  private:
		//! Populate a field with default value from Lib3DFin Params
		static void populateFieldFromLib3DFin(UiField& field, const lib3dfin::Params& libParams);

		//! Get all field configurations
		static const std::unordered_map<QString, UiField>& getAllFields();

	  public:
		//! Get all ui fields with default values populated from lib3DFin Params struct
		static std::unordered_map<QString, UiField> getAllFieldsFromLib3DFin();

		//! Get field configurations for a specific group
		static std::vector<UiField> getFieldsByGroup(const QString& group);

		//! Find a field by name
		static const UiField* findField(const QString& name);

		//! Check if a field exists
		static bool hasField(const QString& name);
	};

} // namespace tdf
