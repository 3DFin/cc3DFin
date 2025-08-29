#pragma once

// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright 2023-2025 Carlos Cabo <carloscabo@uniovi.es>

#include <Eigen/Dense>

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

	template <typename real_t>
	using Vec2 = Eigen::Vector<real_t, 2>;

	using ArrayMask = Eigen::Array<bool, Eigen::Dynamic, 1>;

	using ArrayClusterIndicator = Eigen::Array<int32_t, Eigen::Dynamic, 1>;

	template <typename int_t>
	using VecIndex = Eigen::Vector<int_t, Eigen::Dynamic>;

	inline constexpr int32_t NO_CLUSTER_ID = -1;

	template <typename real_t>
	struct Circle
	{
		Vec2<real_t> center{0.0, 0.0};
		real_t       radius{0.0};
	};

	template <typename real_t>
	struct Stripe
	{
		Stripe(real_t lower_limit_, real_t upper_limit_)
		    : lower_limit(lower_limit_)
		    , upper_limit(upper_limit_)
		{
			// todo check upper_limit > lower_limit
		}
		ArrayClusterIndicator cluster_indicator;
		real_t                lower_limit{0.0};
		real_t                upper_limit{0.0};
	};

	template <typename real_t>
	struct TreeDescriptor
	{
		TreeDescriptor(Eigen::Index tree_id_)
		    : tree_id(tree_id_)
		{
		}

		static std::optional<Vec3<real_t>> vector_plane_intersection(
		    const Vec3<real_t>&                 axis_pos,
		    const Vec3<real_t>&                 axis_dir,
		    const Eigen::Hyperplane<real_t, 3>& plane)
		{
			const real_t denom = plane.normal().dot(axis_dir);
			if (std::abs(denom) < 1e-8)
			{
				return std::nullopt;
			}
			const real_t t = -(plane.normal().dot(axis_pos) + plane.offset()) / denom;
			return axis_pos + t * axis_dir;
		}

		static std::optional<std::pair<Vec3<real_t>, Vec3<real_t>>> axis_bb_intersection(
		    const Vec3<real_t>& axis_pos,
		    const Vec3<real_t>& axis_dir,
		    const Vec3<real_t>& bottom_pos,
		    const Vec3<real_t>& top_pos)
		{
			Eigen::Hyperplane<real_t, 3> bottom_plane(Vec3<real_t>(0, 0, 1), bottom_pos);
			const auto                   bottom_inter = vector_plane_intersection(axis_pos, axis_dir, bottom_plane);
			if (bottom_inter == std::nullopt)
				return std::nullopt;

			Eigen::Hyperplane<real_t, 3> top_plane(Vec3<real_t>(0, 0, -1), top_pos);
			const auto                   top_inter = vector_plane_intersection(axis_pos, axis_dir, top_plane);
			if (top_inter == std::nullopt)
				return std::nullopt;
			return std::make_pair(bottom_inter.value(), top_inter.value());
		}

		void setAxis(const Vec3<real_t>& axis_, const real_t max_deviation)
		{
			axis                    = axis_;
			axis_vertical_deviation = std::abs(std::atan(std::hypot(axis(0), axis(1)) / axis(2)) * (180.0 / M_PI));
			valid                   = axis_vertical_deviation < max_deviation;
		}

		void setHeighestPoint(const Vec3<real_t>& heighest_point_)
		{
			heighest_point = heighest_point_;
			heighest_z0    = heighest_point(2) - height_difference;
		}

		PointCloud3<real_t> computeAxisSampling(const Vec3<real_t>& bb_min, const Vec3<real_t> bb_max, real_t sample_step) const
		{
			const auto          maybe_range    = axis_bb_intersection(centroid_coordinates, axis, bb_min, bb_max);
			const auto          bottom_point   = maybe_range.value().first;
			const auto          top_point      = maybe_range.value().second;
			const auto          range_distance = (top_point - bottom_point).norm();
			const auto          num_sample     = static_cast<size_t>(std::ceil(range_distance / sample_step));
			PointCloud3<real_t> axis_point_cloud(num_sample, 3);
			// get the upward pointing vector
			const auto axis_sample_axis = axis(2) < 0 ? -axis : axis;
			for (Eigen::Index point_id = 0; point_id < num_sample; ++point_id)
			{
				axis_point_cloud.row(point_id) = bottom_point + axis_sample_axis * (static_cast<real_t>(point_id) * sample_step);
			}
			return axis_point_cloud;
		}

		Eigen::Index tree_id{0};
		real_t       height_difference{0}; // z - z0
		Vec3<real_t> centroid_coordinates{0., 0., 0.};
		Vec3<real_t> axis{0., 0., 0.}; // most significant eigen vector?
		real_t       axis_vertical_deviation{0.};
		bool         valid{false}; // under max deviation threshold
		Vec3<real_t> heighest_point{0.0, 0.0, 0.0};
		real_t       heighest_z0{0};
	};

	template <typename real_t>
	struct AxesData
	{
		std::vector<TreeDescriptor<real_t>> tree_descriptors;
		Eigen::VectorX<real_t>              axis_distance;
		ArrayClusterIndicator               axis_cluster_indicator; // TODO: refactor tree cluster indicator

		void updateIndicator(const ArrayClusterIndicator& mask_indicator)
		{
			axis_cluster_indicator = (mask_indicator > -1).select(axis_cluster_indicator, -1);
		}
	};

} // namespace lib3dfin
