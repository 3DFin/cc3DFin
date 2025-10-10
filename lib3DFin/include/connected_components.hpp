#pragma once

// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright 2023-2025 Carlos Cabo <carloscabo@uniovi.es>

#include "types.hpp"

namespace lib3dfin
{
	VecIndex<int32_t> connected_components(const PointCloud3& xyz, const double eps, const uint32_t min_samples);
} // namespace lib3dfin
