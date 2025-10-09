#pragma once

// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright 2023-2025 Carlos Cabo <carloscabo@uniovi.es>

#include <Eigen/Dense>

namespace lib3dfin
{

	// template variables, hopefully not violating the ODR
	constexpr double RAD_TO_DEG = 180.0 / M_PI;

	constexpr double DEG_TO_RAD = M_PI / 180.0;

	using PointCloud3 = Eigen::Matrix<double, Eigen::Dynamic, 3, Eigen::RowMajor>;

	using PointCloud2 = Eigen::Matrix<double, Eigen::Dynamic, 2, Eigen::RowMajor>;

	using Vec3 = Eigen::Vector<double, 3>;

	using Vec2 = Eigen::Vector<double, 2>;

	using Plane3 = Eigen::Hyperplane<double, 3>;

	using ArrayMask = Eigen::Array<bool, Eigen::Dynamic, 1>;

	using ArrayClusterIndicator = Eigen::Array<int32_t, Eigen::Dynamic, 1>;

	template <typename int_t>
	using VecIndex = Eigen::Vector<int_t, Eigen::Dynamic>;

	inline constexpr int32_t NO_CLUSTER_ID = -1;

	struct Circle
	{
		Vec2   center{0.0, 0.0};
		double radius{0.0};
	};

	struct Stripe
	{
		Stripe(double lower_limit_, double upper_limit_)
		    : lower_limit(lower_limit_)
		    , upper_limit(upper_limit_)
		{
			// todo check upper_limit > lower_limit
		}
		ArrayClusterIndicator cluster_indicator;
		double                lower_limit{0.0};
		double                upper_limit{0.0};
	};

	struct TreeDescriptor
	{
		TreeDescriptor(Eigen::Index tree_id_)
		    : tree_id(tree_id_)
		{
		}

		static std::optional<Vec3> vector_plane_intersection(
		    const Vec3&   axis_pos,
		    const Vec3&   axis_dir,
		    const Plane3& plane)
		{
			const double denom = plane.normal().dot(axis_dir);
			if (std::abs(denom) < 1e-8)
			{
				return std::nullopt;
			}
			const double t = -(plane.normal().dot(axis_pos) + plane.offset()) / denom;
			return axis_pos + t * axis_dir;
		}

		static std::optional<std::pair<Vec3, Vec3>> axis_bb_intersection(
		    const Vec3& axis_pos,
		    const Vec3& axis_dir,
		    const Vec3& bottom_pos,
		    const Vec3& top_pos)
		{
			Plane3     bottom_plane(Vec3(0, 0, 1), bottom_pos);
			const auto bottom_inter = vector_plane_intersection(axis_pos, axis_dir, bottom_plane);
			if (bottom_inter == std::nullopt)
				return std::nullopt;

			Plane3     top_plane(Vec3(0, 0, -1), top_pos);
			const auto top_inter = vector_plane_intersection(axis_pos, axis_dir, top_plane);
			if (top_inter == std::nullopt)
				return std::nullopt;
			return std::make_pair(bottom_inter.value(), top_inter.value());
		}

		void setAxis(const Vec3& axis_, const double max_deviation)
		{
			// axis always points upwards
			axis                    = (axis_(2) < 0) ? -axis_ : axis_;
			axis_vertical_deviation = std::atan2(std::hypot(axis(0), axis(1)), axis(2)) * RAD_TO_DEG;
			valid                   = axis_vertical_deviation < max_deviation;
		}

		void setHeighestPoint(const Vec3& heighest_point_)
		{
			highest_point = heighest_point_;
			highest_z0    = highest_point(2) - height_difference;
		}

		PointCloud3 computeAxisSampling(const Vec3& bb_min, const Vec3& bb_max, double sample_step) const
		{
			const auto  maybe_range    = axis_bb_intersection(centroid_coordinates, axis, bb_min, bb_max);
			const auto  bottom_point   = maybe_range.value().first;
			const auto  top_point      = maybe_range.value().second;
			const auto  range_distance = (top_point - bottom_point).norm();
			const auto  num_sample     = static_cast<size_t>(std::ceil(range_distance / sample_step));
			PointCloud3 axis_point_cloud(num_sample, 3);
			// get the upward pointing vector
			const auto axis_sample_axis = axis(2) < 0 ? -axis : axis;
			for (Eigen::Index point_id = 0; point_id < num_sample; ++point_id)
			{
				axis_point_cloud.row(point_id) = bottom_point + axis_sample_axis * (static_cast<double>(point_id) * sample_step);
			}
			return axis_point_cloud;
		}

		Eigen::Index tree_id{0};
		double       height_difference{0}; // z - z0
		Vec3         centroid_coordinates{0., 0., 0.};
		Vec3         axis{0., 0., 0.}; // most significant eigen vector?
		double       axis_vertical_deviation{0.};
		bool         valid{false}; // under max deviation threshold
		Vec3         highest_point{0.0, 0.0, 0.0};
		double       highest_z0{0};
	};

	struct AxesData
	{
		std::vector<TreeDescriptor> tree_descriptors;
		Eigen::VectorX<double>      axis_distance;
		ArrayClusterIndicator       axis_cluster_indicator; // TODO: refactor tree cluster indicator

		void updateIndicator(const ArrayClusterIndicator& mask_indicator)
		{
			axis_cluster_indicator = (mask_indicator > -1).select(axis_cluster_indicator, -1);
		}
	};

	struct TreeLocatorResult
	{
		double dbh{0.0};
		Vec3   location{0.0, 0.0, 0.0};
	};

	struct CircleData
	{
		enum class Status
		{
			NOT_COMPUTED      = -2,
			NOT_ENOUGH_POINTS = -1,
			SUCCESS           = 0,
			TILT_OUTLIER,
			DIAMETER_TOO_SMALL,
			DIAMETER_TOO_LARGE,
			TOO_MANY_POINTS_INNER,
			NOT_ENOUGH_SECTOR_COVERAGE,
		};

		Circle   circle{};
		double   z0{0};
		Status   status{Status::NOT_COMPUTED};
		double   sector_percentage{0.0};
		double   outlier_probability{0.0};
		uint32_t number_points_inner{0};
	};

	using CircleSections = std::vector<CircleData<real_t>>;

} // namespace lib3dfin
