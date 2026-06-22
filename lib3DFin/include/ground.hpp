#pragma once

// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright 2023-2026 Carlos Cabo <carloscabo@uniovi.es>

// local
#include "config.hpp"
#include "types.hpp"

// taskflow
#include <taskflow/taskflow.hpp>

namespace lib3dfin
{
	class HeightNormalization
	{
	  public: // methods
		explicit HeightNormalization(const PointCloud3& point_cloud, const GroundParams& params, tf::Executor& executor);

		Eigen::VectorXd normalize();

		static std::pair<bool, double> checkHeightNormDiscrepancy(const PointCloud3& point_cloud, const Eigen::VectorXd& z0, double original_area, tf::Executor& executor, double res_xy = 1.0, double z_min = -0.1, double z_max = 0.15, double threshold = 0.1);

		const DTMData exportDTM() const;

	  private: // methods
		PointCloud3 denoiseCloud();
		void        generateDTM(const PointCloud3& dtm_point_cloud);
		void        cleanDTMmad();
		void        smoothDTMmedian();
		void        smoothDTMLaplacian();
		void        normalizeGrid();

	  private: // members
		const PointCloud3& point_cloud_;
		const GroundParams params_;
		PointCloud3        dtm_;
		std::vector<bool>  mask_;
		size_t             width_{0};
		size_t             height_{0};
		tf::Executor&      executor_;
	};

} // namespace lib3dfin
