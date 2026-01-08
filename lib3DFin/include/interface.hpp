#pragma once

// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright 2023-2025 Carlos Cabo <carloscabo@uniovi.es>

#include "config.hpp"
#include "types.hpp"

#include <cstddef>
#include <spdlog/spdlog.h>

namespace lib3dfin
{
	using TDFResult = std::tuple<std::vector<int32_t>, std::vector<double>, lib3dfin::TreeData>;

	TDFResult process(const float* cloud_data, const double* z0_sf, size_t num_points, const Params& params, std::shared_ptr<spdlog::logger>, const std::string& output_dir);
} // namespace lib3dfin
