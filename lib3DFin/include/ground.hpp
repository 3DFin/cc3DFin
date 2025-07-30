#pragma once

// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright 2023-2025 Carlos Cabo <carloscabo@uniovi.es>

#include "types.hpp"

namespace lib3dfin
{
	template <typename real_t>
	PointCloud3<real_t> generate_dtm(const RefPointCloud<real_t>& point_cloud, const real_t resolution);

	template <typename real_t>
	PointCloud3<real_t> clean_ground(const PointCloud3<real_t>& point_cloud, const real_t resolution, const real_t minimum_points);

	template <typename real_t>
	PointCloud3<real_t> clean_cloth(const PointCloud3<real_t>& cloth);

	template <typename real_t>
	Eigen::VectorX<real_t> normalize_height(const RefPointCloud<real_t>& point_cloud, const PointCloud3<real_t>& dtm);
} // namespace lib3dfin
