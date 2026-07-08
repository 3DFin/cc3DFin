#pragma once

// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright 2023-2026 Carlos Cabo <carloscabo@uniovi.es>

#include "config.hpp"
#include "types.hpp"

#include <spdlog/spdlog.h>

// StdLib
#include <cstddef>
#include <filesystem>
#include <span>

namespace fs = std::filesystem;

namespace lib3dfin
{

	enum class Status
	{
		Success = 0,
		InvalidInput,
		DTMTooSmall,
		NoValidClusters,
		Error,
	};

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
		explicit TDFProcessing(std::span<const double> cloud_data);

		~TDFProcessing()                               = default;
		TDFProcessing& operator=(const TDFProcessing&) = delete;

		explicit TDFProcessing(PointCloud3 point_cloud);

		void setExternalZ0(std::span<const double> z0_data);
		void setExternalZ0(Eigen::VectorXd z0);

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

		Status process();

		const TreeData& getTreeData() const
		{
			return tree_data_;
		};

		const Eigen::VectorXd& getZ0() const
		{
			return z0_;
		}

		const ArrayClusterIndicator& getStemIndicator() const
		{
			return stripe_.cluster_indicator;
		}

		const DTMData& getDTM() const
		{
			return dtm_;
		}

		// TODO catch xlsx exceptions
		void exportTabularData() const;

	  private:
		fs::path        output_basepath_{fs::current_path() / "3DFin"};
		Params          params_;
		ProjectMeta     project_meta_;
		PointCloud3     point_cloud_;
		Eigen::VectorXd z0_;
		Stripe          stripe_;
		DTMData         dtm_;
		TreeData        tree_data_;
	};

} // namespace lib3dfin
