#pragma once

// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright 2023-2025 Carlos Cabo <carloscabo@uniovi.es>

#include <Eigen/Dense>
#include <Eigen/Eigenvalues>
#include <Eigen/src/Core/util/Constants.h>

namespace lib3dfin
{
	template <typename real_t>
	using PointCloud3 = Eigen::Matrix<real_t, Eigen::Dynamic, 3, Eigen::RowMajor>;

	template <typename real_t>
	using RefPointCloud = Eigen::Ref<const PointCloud3<real_t>>;

	template <typename real_t>
	using PointCloud2 = Eigen::Matrix<real_t, Eigen::Dynamic, 2, Eigen::RowMajor>;

	template <typename real_t>
	using Vec3 = Eigen::Vector<real_t, 3>;

	using ArrayMask = Eigen::Array<bool, Eigen::Dynamic, 1>;


	template <typename int_t>
	using VecIndex = Eigen::Vector<int_t, Eigen::Dynamic>;

} // namespace lib3dfin
