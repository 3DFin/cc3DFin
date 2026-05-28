#pragma once

// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright 2023-2026 Carlos Cabo <carloscabo@uniovi.es>
#include "types.hpp"

#include <OpenXLSX.hpp>
#include <spdlog/spdlog.h>

namespace lib3dfin
{
	void export_xlsx(const TreeData& tree_data, const std::string& filename, const ProjectMeta& meta)
	{

		if (tree_data.tree_descriptors.empty())
		{
			spdlog::error("[XLSX] Nothing to write");
			return;
		}

		using SheetDescriptor = std::pair<std::string, std::string>;

		constexpr size_t                                  num_worksheets    = 9;
		const std::array<SheetDescriptor, num_worksheets> sheet_descriptors = {
		    {{"Plot metrics", "Total height(TH) of each tree(T).\nDiameter at breast height(DBH) of each tree(T).\n(x, y)coordinates(X and Y) of each tree(T).)"},
		     {"Diameters", "Diameter of every section (S) of every tree (T). Units are meters."},
		     {"X", "(x) coordinates of every section (S) of every tree (T). Units are meters"},
		     {"Y", "(y) coordinates of every section (S) of every tree (T). Units are meters."},
		     {"Sections", "Normalized height (Z0) of every section (S).\nUnits are meters."},
		     {"Q(Overall Quality 0-1)", "Overall quality of every section (S) of every tree (T).\n0 : Section does not pass quality checks - 1 : Section passes quality checks."},
		     {"Q1(Outlier Probability)", "(Normalized height (Z0) of every section (S).\nUnits are meters."},
		     {"Q2(Sector Occupancy)", "Percentage of occupied sectors of every section (S) of every tree (T).\nIt takes values between 0 and 100."},
		     {"Q3(Points Inner Circle)", "Number of points in the inner circle of every section (S) of every tree (T).\nThe lowest, the better."}}};

		// Container to store our reference to worksheets
		std::vector<OpenXLSX::XLWorksheet> worksheets;

		// Use first cell of each worksheet as a header
		const OpenXLSX::XLCellReference header_cell_ref("A1");

		OpenXLSX::XLDocument doc;

		try
		{
			doc.create(filename, OpenXLSX::XLForceOverwrite);
		}
		catch (const std::exception& e)
		{
			spdlog::error("[3DFin] Failed to create XLSX file: {}", e.what());
			return;
		}

		// Define the style of the table header (column and rows)
		OpenXLSX::XLCellFormats& formats = doc.styles().cellFormats();
		OpenXLSX::XLFonts&       fonts   = doc.styles().fonts();

		const OpenXLSX::XLStyleIndex bold_format = formats.create();
		const OpenXLSX::XLStyleIndex font_bold   = fonts.create();
		fonts[font_bold].setBold();
		formats[bold_format].setFontIndex(font_bold);

		// worksheet 1 is "special". It's already there at workbook creation and it needs some custom (table) headers.
		auto& sheet_1 = worksheets.emplace_back(doc.workbook().worksheet(1));
		sheet_1.setName(sheet_descriptors[0].first);
		sheet_1.cell(header_cell_ref) = sheet_descriptors[0].second;

		std::stringstream meta_description;
		meta_description << "This cloud has " << meta.num_points / 1'000'000.0 << " million points and its area is " << meta.area_m2 << " m2";
		sheet_1.cell("A2") = meta_description.str().c_str();

		// Table Header
		auto table_header       = sheet_1.range("C3:F3");
		sheet_1.row(3).values() = std::vector<std::string>{"", "", "TH", "DBH", "X", "Y"};
		table_header.setFormat(bold_format);

		// Other sheets
		for (size_t worksheet_id = 1; worksheet_id < num_worksheets; ++worksheet_id)
		{
			// sheet_2
			const auto& cur_sheet_desc = sheet_descriptors[worksheet_id];
			doc.workbook().addWorksheet(cur_sheet_desc.first);
			auto& cur_sheet                 = worksheets.emplace_back(doc.workbook().worksheet(worksheet_id + 1));
			cur_sheet.cell(header_cell_ref) = cur_sheet_desc.second;
		}

		uint32_t tree_id = 1;
		for (const auto& tree : tree_data.tree_descriptors)
		{
			uint32_t    row_id     = tree_id + 2; // header offset
			std::string row_header = "T" + std::to_string(tree_id);

			const auto header_row_ref = OpenXLSX::XLCellReference(row_id, 2);
			for (uint32_t worksheet_id = 1; worksheet_id < num_worksheets; ++worksheet_id)
			{
				if (worksheet_id != 4)
				{
					worksheets[worksheet_id].cell(header_row_ref) = row_header;
				}
			}

			// Special case for sheet_1 row_id is shifted by 1 because of the subheader
			worksheets[0].cell(OpenXLSX::XLCellReference(row_id + 1, 2)) = row_header;
			worksheets[0].cell(OpenXLSX::XLCellReference(row_id + 1, 3)) = tree.highest_z0 / meta.scale;
			worksheets[0].cell(OpenXLSX::XLCellReference(row_id + 1, 4)) = tree.dbh / meta.scale;

			const Vec3 global_tree_location = (tree.location / meta.scale) - meta.shift;

			worksheets[0].cell(OpenXLSX::XLCellReference(row_id + 1, 5)) = global_tree_location.x();
			worksheets[0].cell(OpenXLSX::XLCellReference(row_id + 1, 6)) = global_tree_location.y();

			uint32_t col_id = 3;
			for (const auto& section : tree.circle_data)
			{

				const auto section_ref = OpenXLSX::XLCellReference(row_id, col_id);

				if (section.status == CircleData::Status::SUCCESS)
				{
					worksheets[1].cell(section_ref) = (section.circle.radius * 2.0) / meta.scale;

					const Vec2 global_circle_center = (section.circle.center / meta.scale) - meta.shift.head<2>();
					worksheets[2].cell(section_ref) = section.circle.center.x();
					worksheets[3].cell(section_ref) = section.circle.center.y();
					worksheets[5].cell(section_ref) = 0;
				}
				else
				{
					worksheets[1].cell(section_ref) = 0.0;
					worksheets[2].cell(section_ref) = 0.0;
					worksheets[3].cell(section_ref) = 0.0;
					worksheets[5].cell(section_ref) = 1;
				}

				// Outlier probability
				worksheets[6].cell(section_ref) = section.outlier_probability;
				worksheets[7].cell(section_ref) = section.sector_percentage;
				worksheets[8].cell(section_ref) = section.number_points_inner;

				++col_id;
			}
			++tree_id;
		}

		// special case for sheet_5 (section z0) and column headers
		const auto sample_circle_data = tree_data.tree_descriptors.front().circle_data;

		uint32_t column_id = 1;
		uint32_t row_id    = 2;
		for (const auto& circle : sample_circle_data)
		{
			std::string column_header = "S" + std::to_string(column_id);

			// index for the common case (sheet 2 - 9 exculding sheet 5)
			const auto regular_column_header_ref = OpenXLSX::XLCellReference(row_id, column_id + 2);

			for (uint32_t worksheet_id = 1; worksheet_id < num_worksheets; ++worksheet_id)
			{
				if (worksheet_id != 4)
				{
					worksheets[worksheet_id].cell(regular_column_header_ref) = column_header;
				}
			}

			worksheets[4].cell(OpenXLSX::XLCellReference(row_id, column_id))     = column_header;
			worksheets[4].cell(OpenXLSX::XLCellReference(row_id + 1, column_id)) = circle.z0 / meta.scale;
			++column_id;
		}

		doc.save();
		doc.close();
	}

} // namespace lib3dfin
