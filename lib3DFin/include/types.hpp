#pragma once

// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright 2023-2025 Carlos Cabo <carloscabo@uniovi.es>

#include <Eigen/Dense>
#include <Eigen/Eigenvalues>

namespace lib3dfin
{
	template <typename real_t>
	using PointCloud3 = Eigen::Matrix<real_t, Eigen::Dynamic, 3>;

	template <typename real_t>
	using PointCloud2 = Eigen::Matrix<real_t, Eigen::Dynamic, 2>;

	template <typename real_t>
	using Vec3 = Eigen::Vector<real_t, 3>;

	template <typename int_t>
	using VecIndex = Eigen::Vector<int_t, Eigen::Dynamic>;

} // namespace lib3dfin
