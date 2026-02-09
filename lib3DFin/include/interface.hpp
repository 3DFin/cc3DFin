#pragma once

// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright 2023-2025 Carlos Cabo <carloscabo@uniovi.es>

#include "config.hpp"
#include "types.hpp"

#include <spdlog/spdlog.h>

// StdLib
#include <cstddef>
#include <filesystem>
namespace fs = std::filesystem;

namespace lib3dfin
{
	struct GlobalShift
	{
		double x_shift{0.};
		double y_shift{0.};
		double z_shift{0.};
		double scale{1.};
	};

	class TDFProcessing
	{
	  public:
		explicit TDFProcessing(const double* cloud_data, const double* z0_sf, size_t num_points);
		explicit TDFProcessing(const double* cloud_data, size_t num_points);

		~TDFProcessing()                               = default;
		TDFProcessing& operator=(const TDFProcessing&) = delete;

		void setParams(const Params& params)
		{
			params_ = params;
		}

		void setGlobalShift(const GlobalShift& global_shift)
		{
			project_meta_.shift = {global_shift.x_shift, global_shift.y_shift, global_shift.z_shift};
			project_meta_.scale = global_shift.scale;
		}

		void setOutputPath(const fs::path output_basepath)
		{
			output_basepath_ = output_basepath;
		}

		bool process();

		const TreeData& getTreeData() const
		{
			return tree_data_;
		};

		const Eigen::VectorXd& getZ0() const
		{
			return z0_;
		}

		const std::pair<std::vector<size_t>, PointCloud3>& getDTM() const
		{
			return dtm_;
		}

		const ArrayClusterIndicator& getStemIndicator() const
		{
			return stripe_.cluster_indicator;
		}

		std::pair<std::vector<size_t>, PointCloud3> getDTM()
		{
			return dtm_;
		}

		// TODO: use the future draw interface here.
		// TODO catch xlsx exceptions
		void exportTabularData() const;

	  private:
		fs::path                                    output_basepath_{fs::current_path() / "3DFin"};
		Params                                      params_;
		ProjectMeta                                 project_meta_;
		PointCloud3                                 point_cloud_;
		Eigen::VectorXd                             z0_;
		Stripe                                      stripe_;
		std::pair<std::vector<size_t>, PointCloud3> dtm_;
		TreeData                                    tree_data_;
	};

} // namespace lib3dfin
