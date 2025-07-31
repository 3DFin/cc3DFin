#pragma once

// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright 2023-2025 Carlos Cabo <carloscabo@uniovi.es>

#include "types.hpp"

namespace lib3dfin
{
	template <typename real_t>
	VecIndex<int32_t> connected_components(const RefPointCloud<real_t>& xyz, const real_t eps, const uint32_t min_samples);
} // namespace lib3dfin
