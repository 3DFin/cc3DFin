#pragma once

// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright 2023-2025 Carlos Cabo <carloscabo@uniovi.es>

#include "types.hpp"

namespace lib3dfin
{
   	template <typename real_t>
	PointCloud3<real_t> filter_stripe(const RefPointCloud<real_t>& point_cloud, const Eigen::VectorX<real_t>& z0, real_t stripe_lower_limit, real_t stripe_upper_limit);

    template <typename real_t>
	void verticality_clustering(const PointCloud3<real_t>& stripe, real_t scale = 0.1, real_t vert_threshold = 0.7, uint32_t n_points = 1000, real_t resolution_xy = 0.02, real_t resolution_z = 0.02, uint32_t n_iter = 2);
}
