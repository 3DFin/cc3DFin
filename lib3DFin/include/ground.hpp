#pragma once

// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright 2023-2025 Carlos Cabo <carloscabo@uniovi.es>

#include "types.hpp"

namespace lib3dfin
{
	//    template <typename real_t>
	// Eigen::VectorXd compute_normalized_height(bool do_clean)
	// {
	// }

	template <typename real_t>
	PointCloud3<real_t> clean_ground(const PointCloud3<real_t>& point_cloud, const real_t resolution, const real_t minimum_points);

	template <typename real_t>
	PointCloud3<real_t> generate_dtm(const PointCloud3<real_t>& point_cloud, const real_t resolution);

	template <typename real_t>
	PointCloud3<real_t> clean_cloth(PointCloud3<real_t>& cloth);

	template <typename real_t>
	PointCloud3<real_t> normalize_height(const PointCloud3<real_t>& point_cloud, const PointCloud3<real_t>& dtm);

	// Point area_warning, area_discrepancy = dm.check_normalization_discrepancy(coords[:, [0, 1, 3]], cloud_shape)

} // namespace lib3dfin
