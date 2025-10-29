#pragma once

// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright 2023-2025 Carlos Cabo <carloscabo@uniovi.es>

#include "types.hpp"
#include <cstddef>

namespace lib3dfin
{
	std::tuple<std::vector<int32_t>, std::vector<double>, TreeData> process(const float* cloud_data, size_t num_points);

} // namespace lib3dfin
