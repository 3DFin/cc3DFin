// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright 2023-2025 Carlos Cabo <carloscabo@uniovi.es>

#include "interface.hpp"

#include "ground.hpp"
#include "individualize.hpp"
#include "peeling.hpp"
#include "section.hpp"
#include "types.hpp"

#ifdef TDFIN_USES_OPENXLSX
#include <OpenXLSX.hpp>
#endif

namespace lib3dfin
{

#ifdef TDFIN_USES_OPENXLSX
	void export_xlsx(const TreeData& tree_data, const std::string& filename)
	{

		if (tree_data.tree_descriptors.empty())
		{
			std::cout << "[3DFin] Nothing to write" << std::endl;
			return;
		}

		// TODO: global shift
		using SheetDescriptor = std::pair<std::string, std::string>;

		constexpr size_t num_worksheets = 9;
		// TODO: global shift
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
			std::cerr << "[3DFin] Failed to create XLSX file: " << e.what() << '\n';
			return;
		}

		// define the style of the table header (column and rows)
		OpenXLSX::XLCellFormats& formats = doc.styles().cellFormats();
		OpenXLSX::XLFonts&       fonts   = doc.styles().fonts();

		const OpenXLSX::XLStyleIndex bold_format = formats.create();
		const OpenXLSX::XLStyleIndex font_bold   = fonts.create();
		fonts[font_bold].setBold();
		formats[bold_format].setFontIndex(font_bold);

		// worksheet 1 is "special". it's already there and it needs some custom (table) headers
		auto& sheet_1 = worksheets.emplace_back(doc.workbook().worksheet(1));

		// sheet_1
		sheet_1.setName(sheet_descriptors[0].first);
		sheet_1.cell(header_cell_ref) = sheet_descriptors[0].second;

		// TODO interpolate string with real data.
		sheet_1.cell("A2") = "This cloud has 61.100854 million points and its area is 3176 m2";

		// Table Header
		auto table_header       = sheet_1.range("C3:F3");
		sheet_1.row(3).values() = std::vector<std::string>{"", "", "TH", "DBH", "X", "Y"};
		table_header.setFormat(bold_format);

		// other sheets
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

			// special case for sheet_1 row_id is shifted by 1 because of the subheader
			sheet_1.cell(OpenXLSX::XLCellReference(row_id + 1, 2)) = row_header;
			sheet_1.cell(OpenXLSX::XLCellReference(row_id + 1, 3)) = tree.highest_z0;
			sheet_1.cell(OpenXLSX::XLCellReference(row_id + 1, 4)) = tree.dbh;
			sheet_1.cell(OpenXLSX::XLCellReference(row_id + 1, 5)) = tree.location.x();
			sheet_1.cell(OpenXLSX::XLCellReference(row_id + 1, 6)) = tree.location.y();

			uint32_t col_id = 3;
			for (const auto& section : tree.circle_data)
			{

				const auto section_ref = OpenXLSX::XLCellReference(row_id, col_id);
				if (section.status == CircleData::Status::SUCCESS)
				{
					worksheets[1].cell(section_ref) = section.circle.radius;
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
			worksheets[4].cell(OpenXLSX::XLCellReference(row_id + 1, column_id)) = circle.z0;
			++column_id;
		}

		doc.save();
		doc.close();
	}
#endif

	std::tuple<std::vector<int32_t>, std::vector<double>, TreeData> process(const float* cloud_data, size_t num_points)
	{

		// Convert point cloud to double
		PointCloud3 point_cloud(num_points, 3);
		for (size_t i = 0; i < num_points; ++i)
		{
			point_cloud.row(i) = Vec3(cloud_data[i * 3], cloud_data[i * 3 + 1], cloud_data[i * 3 + 2]);
		}

		// TODO: check height normalization calculation, this doe not seams to give the same
		//  extent as its use in python
		HeightNormalization height_normalizer(point_cloud, HeightNormalization::Parameters());
		const auto          z0 = height_normalizer.normalize();
		TreePeeler          stripe_peeler(point_cloud, TreePeeler::Parameters());
		Stripe              stripe(0.7, 3.5);
		stripe.cluster_indicator = TreePeeler::filterInitialStripe(z0, 0.7, 3.5);

		// side effect on indicator
		stripe_peeler.peel(stripe.cluster_indicator);

		TreeIndividualizer tree_individualizer(point_cloud, stripe, z0, TreeIndividualizer::Parameters());
		auto               tree_data = tree_individualizer.individualize();

		// stem_search_diameter / 2, minimum_height, maximum_height + section_width
		auto stem_indicator = TreePeeler::filterInitialStripe(z0, tree_data.axis_distance, 2.0 / 2, 0.3, 25 + 0.05);

		// TODO: verticality could change at this point
		// double verticality_scale_stem  = 0.1;  // verticality_thresh_stems
		// double verticality_thresh_stem = 0.7;

		// TODO Beware Side effect on indicator
		stripe_peeler.peel(stem_indicator);

		ArrayClusterIndicator sections_indicator = (stem_indicator > -1).select(tree_data.cluster_indicator, -1);
		SectionExtractor      section_extractor(point_cloud, sections_indicator, z0, tree_data, SectionExtractor::Parameters());

		// TODO: beware side effect on tree_data
		section_extractor.extract();

		std::vector<int32_t> stem_indicator_vector(stripe.cluster_indicator.data(), stripe.cluster_indicator.data() + stripe.cluster_indicator.size());

		std::vector<double> z0_vector(z0.data(), z0.data() + z0.size());

#ifdef TDFIN_USES_OPENXLSX
		export_xlsx(tree_data, "3DFin.xlsx");
#endif
		return std::make_tuple(std::move(stem_indicator_vector), std::move(z0_vector), std::move(tree_data));
	}

} // namespace lib3dfin
