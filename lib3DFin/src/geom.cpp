// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright 2023-2026 Carlos Cabo <carloscabo@uniovi.es>

#include "geom.hpp"

#include <cmath>

namespace lib3dfin
{

	std::optional<Vec3> vector_plane_intersection(
	    const Vec3&   axis_pos,
	    const Vec3&   axis_dir,
	    const Plane3& plane)
	{
		constexpr float nearParallelThreshold = 1e-5;
		const double denom = plane.normal().dot(axis_dir);
		if (std::abs(denom) < nearParallelThreshold)
		{
			return std::nullopt;
		}
		const double t = -(plane.normal().dot(axis_pos) + plane.offset()) / denom;
		return axis_pos + t * axis_dir;
	}

	std::optional<std::pair<Vec3, Vec3>> axis_bb_intersection(
	    const Vec3& axis_pos,
	    const Vec3& axis_dir,
	    const Vec3& bottom_pos,
	    const Vec3& top_pos)
	{

		Plane3     bottom_plane(Vec3(0, 0, 1), bottom_pos);
		const auto bottom_inter = vector_plane_intersection(axis_pos, axis_dir, bottom_plane);
		if (bottom_inter == std::nullopt)
		{
			return std::nullopt;
		}

		Plane3     top_plane(Vec3(0, 0, -1), top_pos);
		const auto top_inter = vector_plane_intersection(axis_pos, axis_dir, top_plane);
		if (top_inter == std::nullopt)
		{
			return std::nullopt;
		}
		return std::make_pair(bottom_inter.value(), top_inter.value());
	}

	PointCloud3 computeAxisSampling(
	    const Vec3& axis_pos,
	    const Vec3& axis_dir,
	    const Vec3& bb_min,
	    const Vec3& bb_max,
	    double      sample_step)
	{
		const auto maybe_range = axis_bb_intersection(axis_pos, axis_dir, bb_min, bb_max);
		if (maybe_range == std::nullopt)
		{
			return {0, 3};
		}

		const auto& bottom_point   = maybe_range.value().first;
		const auto& top_point      = maybe_range.value().second;
		const auto  range_distance = (top_point - bottom_point).norm();
		const auto  num_sample     = static_cast<size_t>(std::ceil(range_distance / sample_step));
		PointCloud3 axis_point_cloud(num_sample, 3);
		// get the upward pointing vector
		const auto axis_sample_axis = axis_dir(2) < 0 ? -axis_dir : axis_dir;
		for (Eigen::Index point_id = 0; point_id < num_sample; ++point_id)
		{
			axis_point_cloud.row(point_id) = bottom_point + axis_sample_axis * (static_cast<double>(point_id) * sample_step);
		}
		return axis_point_cloud;
	}

} // namespace lib3dfin
