#pragma once

// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright 2023-2025 Carlos Cabo <carloscabo@uniovi.es>

// Local
#include "types.hpp"

// Taskflow
#include <taskflow/taskflow.hpp>

namespace lib3dfin
{
	std::vector<int32_t> connected_components(const PointCloud3& xyz, const double eps, const uint32_t min_samples, tf::Executor& executor);
} // namespace lib3dfin
