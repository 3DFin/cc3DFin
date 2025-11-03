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
		// TODO: global shift
		std::string sheet_1_name       = "Plot metrics";
		std::string sheet_1_header     = R"(Total height (TH) of each tree (T).
        Diameter at breast height (DBH) of each tree (T).
        (x, y) coordinates (X and Y) of each tree (T).)";
		std::string sheet_1_sub_header = R"(This cloud has 61.100854 million points and its area is 3176 m2)";

		std::string sheet_2_name   = "Diameters";
		std::string sheet_2_header = "Diameter of every section (S) of every tree (T). Units are meters.";

		std::string sheet_3_name   = "X";
		std::string sheet_3_header = "(x) coordinates of every section (S) of every tree (T). Units are meters.";

		std::string sheet_4_name   = "Y";
		std::string sheet_4_header = "(y) coordinates of every section (S) of every tree (T). Units are meters.";

		std::string sheet_5_name   = "Sections";
		std::string sheet_5_header = R"(Normalized height (Z0) of every section (S).
        Units are meters.)";

		std::string sheet_6_name   = "Q(Overall Quality 0-1)";
		std::string sheet_6_header = R"(Overall quality of every section (S) of every tree (T).
        0: Section does not pass quality checks - 1: Section passes quality checks.)";

		std::string sheet_7_name   = "Q1(Outlier Probability)";
		std::string sheet_7_header = R"(Normalized height (Z0) of every section (S).
        Units are meters.)";

		std::string sheet_8_name   = "Q2(Sector Occupancy)";
		std::string sheet_8_header = R"(Percentage of occupied sectors of every section (S) of every tree (T).
        It takes values between 0 and 100.)";

		std::string sheet_9_name   = "Q3(Points Inner Circle)";
		std::string sheet_9_header = R"(Number of points in the inner circle of every section (S) of every tree (T).
        The lowest, the better.)";

		OpenXLSX::XLDocument doc{};
		// TODO catch exceptions
		doc.create(filename, OpenXLSX::XLForceOverwrite);
		OpenXLSX::XLWorksheet sheet_1 = doc.workbook().worksheet(1); // worksheet are 1 based in Excel

		// define the style of the table header (column and rows)
		OpenXLSX::XLCellFormats& formats = doc.styles().cellFormats();
		OpenXLSX::XLFonts&       fonts   = doc.styles().fonts();

		const OpenXLSX::XLStyleIndex bold_format = formats.create();
		const OpenXLSX::XLStyleIndex font_bold   = fonts.create();
		fonts[font_bold].setBold();
		formats[bold_format].setFontIndex(font_bold);

		// sheet_1
		sheet_1.setName(sheet_1_name);
		sheet_1.cell("A1").value() = sheet_1_header;
		sheet_1.cell("A2").value() = sheet_1_sub_header;

		// Table Header
		auto table_header       = sheet_1.range("C3:F3");
		sheet_1.row(3).values() = std::vector<std::string>{"", "", "TH", "DBH", "X", "Y"};
		table_header.setFormat(bold_format);

		// sheet_2
		doc.workbook().addWorksheet(sheet_2_name);
		OpenXLSX::XLWorksheet sheet_2 = doc.workbook().worksheet(2);
		sheet_2.cell("A1").value()    = sheet_2_header;
		// sheet_3
		doc.workbook().addWorksheet(sheet_3_name);
		OpenXLSX::XLWorksheet sheet_3 = doc.workbook().worksheet(3);
		sheet_3.cell("A1").value()    = sheet_3_header;
		// sheet_4
		doc.workbook().addWorksheet(sheet_4_name);
		OpenXLSX::XLWorksheet sheet_4 = doc.workbook().worksheet(4);
		sheet_4.cell("A1").value()    = sheet_4_header;
		// sheet_5
		doc.workbook().addWorksheet(sheet_5_name);
		OpenXLSX::XLWorksheet sheet_5 = doc.workbook().worksheet(5);
		sheet_5.cell("A1").value()    = sheet_5_header;
		// sheet_6
		doc.workbook().addWorksheet(sheet_6_name);
		OpenXLSX::XLWorksheet sheet_6 = doc.workbook().worksheet(6);
		sheet_6.cell("A1").value()    = sheet_6_header;
		// sheet_7
		doc.workbook().addWorksheet(sheet_7_name);
		OpenXLSX::XLWorksheet sheet_7 = doc.workbook().worksheet(7);
		sheet_7.cell("A1").value()    = sheet_7_header;
		// sheet_8
		doc.workbook().addWorksheet(sheet_8_name);
		OpenXLSX::XLWorksheet sheet_8 = doc.workbook().worksheet(8);
		sheet_8.cell("A1").value()    = sheet_8_header;
		// sheet_9
		doc.workbook().addWorksheet(sheet_9_name);
		OpenXLSX::XLWorksheet sheet_9 = doc.workbook().worksheet(9);
		sheet_9.cell("A1").value()    = sheet_9_header;

		size_t tree_id = 1;
		for (const auto& tree : tree_data.tree_descriptors)
		{
			size_t      row_id     = tree_id + 2; // header offset
			std::string row_header = "T" + std::to_string(tree_id);

			const auto header_row_ref            = OpenXLSX::XLCellReference(row_id, 2);
			sheet_2.cell(header_row_ref).value() = row_header;
			sheet_3.cell(header_row_ref).value() = row_header;
			sheet_4.cell(header_row_ref).value() = row_header;
			sheet_6.cell(header_row_ref).value() = row_header;
			sheet_7.cell(header_row_ref).value() = row_header;
			sheet_8.cell(header_row_ref).value() = row_header;
			sheet_9.cell(header_row_ref).value() = row_header;

			// special case for sheet_1 row_id is shifted by 1 because of the subheader
			sheet_1.cell(OpenXLSX::XLCellReference(row_id + 1, 2)).value() = row_header;
			sheet_1.cell(OpenXLSX::XLCellReference(row_id + 1, 3)).value() = tree.highest_z0;
			sheet_1.cell(OpenXLSX::XLCellReference(row_id + 1, 4))         = tree.dbh;
			sheet_1.cell(OpenXLSX::XLCellReference(row_id + 1, 5))         = tree.location.x();
			sheet_1.cell(OpenXLSX::XLCellReference(row_id + 1, 6))         = tree.location.y();

			size_t col_id = 3;
			for (const auto& section : tree.circle_data)
			{

				const auto section_ref = OpenXLSX::XLCellReference(row_id, col_id);
				if (section.status == CircleData::Status::SUCCESS)
				{
					sheet_2.cell(section_ref).value() = section.circle.radius;
					sheet_3.cell(section_ref).value() = section.circle.center.x();
					sheet_4.cell(section_ref).value() = section.circle.center.y();
					sheet_6.cell(section_ref).value() = 0;
				}
				else
				{
					sheet_2.cell(section_ref).value() = 0.0;
					sheet_3.cell(section_ref).value() = 0.0;
					sheet_4.cell(section_ref).value() = 0.0;
					sheet_6.cell(section_ref).value() = 1;
				}

				// Outlier probability
				sheet_7.cell(section_ref).value() = section.outlier_probability;
				sheet_8.cell(section_ref).value() = section.sector_percentage;
				sheet_9.cell(section_ref).value() = section.number_points_inner;

				++col_id;
			}
			++tree_id;
		}

		// special case for sheet_5 (section z0) and column headers
		const auto sample_circle_data = tree_data.tree_descriptors.front().circle_data;

		size_t column_id = 1;
		size_t row_id    = 2;
		for (const auto& circle : sample_circle_data)
		{
			std::string column_header = "S" + std::to_string(column_id);

			// index for the common case (sheet 2 - 9 exculding sheet 5)
			const auto regular_column_header_ref = OpenXLSX::XLCellReference(row_id, column_id + 2);

			sheet_2.cell(regular_column_header_ref).value() = column_header;
			sheet_3.cell(regular_column_header_ref).value() = column_header;
			sheet_4.cell(regular_column_header_ref).value() = column_header;
			sheet_6.cell(regular_column_header_ref).value() = column_header;
			sheet_7.cell(regular_column_header_ref).value() = column_header;
			sheet_8.cell(regular_column_header_ref).value() = column_header;
			sheet_9.cell(regular_column_header_ref).value() = column_header;

			sheet_5.cell(OpenXLSX::XLCellReference(row_id, column_id)).value()     = column_header;
			sheet_5.cell(OpenXLSX::XLCellReference(row_id + 1, column_id)).value() = circle.z0;
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
