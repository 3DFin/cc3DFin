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
	using TDFResult = std::tuple<std::vector<int32_t>, std::vector<double>, lib3dfin::TreeData>;

	struct GlobalShift
	{
		double x_shift{0.};
		double y_shift{0.};
		double z_shift{0.};
		double scale{1.};
	};

	TDFResult process(const float* cloud_data, const double* z0_sf, size_t num_points, const Params& params, const std::optional<fs::path>& output_basepath, const std::optional<GlobalShift>& global_shift);
} // namespace lib3dfin
