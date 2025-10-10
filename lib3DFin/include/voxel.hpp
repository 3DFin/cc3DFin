#pragma once

// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright 2023-2025 Carlos Cabo <carloscabo@uniovi.es>

#include "types.hpp"

namespace lib3dfin
{
	std::tuple<PointCloud3, VecIndex<uint32_t>> voxelize(
	    const PointCloud3& xyz,
	    const double       res_xy,
	    const double       res_z,
	    const bool         verbose);
} // namespace lib3dfin
