#pragma once

// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright 2023-2026 Carlos Cabo <carloscabo@uniovi.es>

#include "geom.hpp"

#include <Eigen/Dense>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace lib3dfin
{

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

	struct DTMData
	{
		PointCloud3         dtm;
		std::vector<size_t> tri_ids;
		std::vector<bool>   dtm_mask;
	};

	struct ProjectMeta
	{
		uint32_t num_points{0};
		uint32_t area_m2{0};
		Vec3     shift{0.0, 0.0, 0.0};
		double   scale{1.0};
	};

	struct Circle
	{
		Vec2   center{0.0, 0.0};
		double radius{0.0};
	};

	enum class DBHSource
	{
		NOT_COMPUTED    = -2,
		NOT_RELIABLE    = -1,
		BHSECTION_OQ_OK = 0,          // Overall Quality (OQ) of the BHSection is OK
		BHSECTION_NEIGHBOUR_SUPPORT,  // OQ of the BH Section is not OK, but one of the 4 neighbouring sections has a diameter that differ by less than a threshold
		NEIGHBOURING_CONSISTENT_PAIR, // OQ of the BH Section is not OK, threshold test fail, but two neighboring non-zero sections differ by less than 8% (use the section from that consistent pair that is closest to the BHSection / 1.3 m)
		CLOSEST_OQ_OK_NEIGHBOUR,      // All previous check failed, we take the closest neighbour with OQ Ok
	};

	struct TreeLocatorResult
	{
		double    dbh{0.0};
		Vec3      location{0.0, 0.0, 0.0};
		DBHSource dbh_source{DBHSource::NOT_COMPUTED};
	};

	struct Stripe
	{
		Stripe() = default;
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

	using CircleSections = std::vector<CircleData>;

	struct TreeDescriptor
	{
		TreeDescriptor(Eigen::Index tree_id_)
		    : tree_id(tree_id_)
		{
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

		void setLocation(const TreeLocatorResult& location_)
		{
			location = location_.location;
			dbh      = location_.dbh;
		}

		PointCloud3 computeAxisSampling(const Vec3& bb_min, const Vec3& bb_max, double sample_step)
		{
			const auto result = lib3dfin::computeAxisSampling(centroid_coordinates, axis, bb_min, bb_max, sample_step);
			if (result.rows() >= 2)
			{
				bottom_point = result.row(0);
				top_point    = result.row(result.rows() - 1);
			}
			return result;
		}

		Eigen::Index            tree_id{0};
		double                  height_difference{0}; // z - z0
		Vec3                    centroid_coordinates{0., 0., 0.};
		Vec3                    axis{0., 0., 0.}; // most significant eigen vector
		Vec3                    top_point{0., 0., 0.};
		Vec3                    bottom_point{0., 0., 0.};
		double                  axis_vertical_deviation{0.};
		bool                    valid{false}; // under max deviation threshold
		Vec3                    highest_point{0.0, 0.0, 0.0};
		double                  highest_z0{0.0};
		double                  dbh{0.0};
		Vec3                    location{0.0, 0.0, 0.0};
		std::vector<CircleData> circle_data{};
	};

	struct TreeData
	{
		std::vector<TreeDescriptor> tree_descriptors;
		Eigen::VectorXd             axis_distance;
		ArrayClusterIndicator       tree_cluster_indicator;
	};

} // namespace lib3dfin
