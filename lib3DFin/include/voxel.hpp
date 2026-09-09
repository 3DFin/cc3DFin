#pragma once

// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright 2023-2026 Carlos Cabo <carloscabo@uniovi.es>

// local
#include "types.hpp"

// taskflow
#include <taskflow/taskflow.hpp>

namespace lib3dfin
{
	std::tuple<PointCloud3, VecIndex<uint32_t>> voxelize(
	    const PointCloud3& xyz,
	    double       res_xy,
	    double       res_z,
	    tf::Executor&      executor,
	    bool         verbose);
} // namespace lib3dfin
