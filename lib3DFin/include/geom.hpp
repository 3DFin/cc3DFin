#pragma once

// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright 2023-2026 Carlos Cabo <carloscabo@uniovi.es>

#include <Eigen/Dense>
#include <optional>

namespace lib3dfin
{

	// Types are defined in types.hpp
	using PointCloud3 = Eigen::Matrix<double, Eigen::Dynamic, 3, Eigen::RowMajor>;
	using Vec3        = Eigen::Vector<double, 3>;
	using Plane3      = Eigen::Hyperplane<double, 3>;

	std::optional<Vec3> vector_plane_intersection(
	    const Vec3&   axis_pos,
	    const Vec3&   axis_dir,
	    const Plane3& plane);

	std::optional<std::pair<Vec3, Vec3>> axis_bb_intersection(
	    const Vec3& axis_pos,
	    const Vec3& axis_dir,
	    const Vec3& bottom_pos,
	    const Vec3& top_pos);

	PointCloud3 computeAxisSampling(
	    const Vec3& axis_pos,
	    const Vec3& axis_dir,
	    const Vec3& bb_min,
	    const Vec3& bb_max,
	    double      sample_step);

} // namespace lib3dfin
