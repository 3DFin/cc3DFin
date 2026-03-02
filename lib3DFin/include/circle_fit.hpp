#pragma once

// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright 2023-2026 Carlos Cabo <carloscabo@uniovi.es

#include "types.hpp"

#include <Eigen/Dense>
#include <Eigen/QR>
#include <unsupported/Eigen/NonLinearOptimization>

namespace lib3dfin
{

	struct HuberLoss
	{
		static double weight(double residual, double threshold)
		{
			const double abs_r = std::abs(residual);
			return (abs_r <= threshold) ? double(1.0) : threshold / abs_r;
		}
	};

	struct HuberEigenCircleFitFunctor
	{
		const PointCloud2& data;
		double             huber_threshold;

		HuberEigenCircleFitFunctor(const PointCloud2& xy, double threshold = 0.1)
		    : n_values(xy.rows())
		    , data(xy)
		    , huber_threshold(threshold)
		{
		}

		// Evaluate weighted residual
		int operator()(const Eigen::VectorXd& x, Eigen::VectorXd& fvec) const
		{
			const double a = x(0);
			const double b = x(1);
			const double r = x(2);

			for (Eigen::Index i = 0; i < data.rows(); ++i)
			{
				const double residual = std::hypot(data(i, 0) - a, data(i, 1) - b) - r;
				const double weight   = HuberLoss::weight(residual, huber_threshold);
				fvec(i)               = std::sqrt(weight) * residual;
			}
			return 0;
		}

		// Compute weighted jacobian
		int df(const Eigen::VectorXd& x, Eigen::MatrixXd& fjac) const
		{
			const double a = x(0);
			const double b = x(1);
			const double r = x(2);

			for (Eigen::Index i = 0; i < data.rows(); ++i)
			{
				const double dx       = data(i, 0) - a;
				const double dy       = data(i, 1) - b;
				const double d        = std::hypot(dx, dy);
				const double residual = d - r;

				const double weight      = HuberLoss::weight(residual, huber_threshold);
				const double sqrt_weight = std::sqrt(weight);

				if (d < 1e-10) // avoid division by zero
				{
					fjac(i, 0) = fjac(i, 1) = 0.0;
				}
				else
				{
					fjac(i, 0) = -sqrt_weight * dx / d;
					fjac(i, 1) = -sqrt_weight * dy / d;
				}
				fjac(i, 2) = -sqrt_weight;
			}
			return 0;
		}

		int n_values = 0;
		int inputs() const
		{
			return 3;
		}
		int values() const
		{
			return n_values;
		}
	};

	Circle algebraicTaubinCircleFit(const PointCloud2& xy)
	{
		const size_t num_points = xy.rows();

		// Linear system
		Eigen::MatrixXd ZXY(num_points, 3);

		// Compute centroid
		const Eigen::Vector2<double> centroid = xy.colwise().mean();

		// Center the data
		ZXY.col(1) = xy.col(0).array() - centroid(0);
		ZXY.col(2) = xy.col(1).array() - centroid(1);

		// Compute Z = X^2 + Y^2
		const Eigen::VectorXd Z      = ZXY.col(1).array().square() + ZXY.col(2).array().square();
		const double          Z_mean = Z.mean();

		// Normalize Z
		ZXY.col(0) = (Z.array() - Z_mean) / (2.0 * sqrt(Z_mean));

		// Solve by SVD
		Eigen::JacobiSVD<Eigen::MatrixXd> svd(ZXY, Eigen::ComputeFullV);
		const Eigen::MatrixXd             V = svd.matrixV();

		Eigen::Vector3d A = V.col(2);
		A(0) /= (2.0 * sqrt(Z_mean));
		Eigen::Vector4<double> A_mat;
		A_mat << A, -Z_mean * A(0);

		// Compute parameter
		Circle result;
		result.center(0) = -A_mat(1) / (2.0 * A_mat(0)) + centroid(0);
		result.center(1) = -A_mat(2) / (2.0 * A_mat(0)) + centroid(1);
		result.radius    = sqrt(A_mat(1) * A_mat(1) + A_mat(2) * A_mat(2) - 4.0 * A_mat(0) * A_mat(3)) / std::abs(A_mat(0)) / 2.0;

		return result;
	}

	Circle initializeByCentroid(const PointCloud2& xy)
	{
		Circle result;

		// Use centroid as initial center
		result.center = xy.colwise().mean();

		// Calculate initial radius as mean distance from centroid to all points
		double             sum_distances = 0.0;
		const Eigen::Index num_points    = xy.rows();

		for (Eigen::Index i = 0; i < num_points; ++i)
		{
			const Vec2   point    = xy.row(i);
			const double distance = (point - result.center).norm();
			sum_distances += distance;
		}

		result.radius = sum_distances / static_cast<double>(num_points);

		return result;
	}

	Circle LMCircleFit(const PointCloud2& xy, double huber_threshold = 0.1)
	{
		if (xy.rows() < 3)
			throw std::invalid_argument("Circle fit need at least 3 points");

		// Initialization by Taubin method
		const auto init_circle = algebraicTaubinCircleFit(xy);

		// Fitting by LM method with Huber M-estimator. We use the intial circle.
		HuberEigenCircleFitFunctor                            functor(xy, huber_threshold);
		Eigen::LevenbergMarquardt<HuberEigenCircleFitFunctor> lm(functor);

		// 30 iters should be enough.
		// See H. Abdul-Rahman and N. Chernov, 2013. : "The GN and LM normally converge in 5–10 iterations."
		lm.parameters.maxfev = 30;
		lm.parameters.xtol   = 1.4e-8;

		Eigen::VectorXd x0(3);
		x0 << init_circle.center.x(), init_circle.center.y(), init_circle.radius;

		int status = lm.minimize(x0); // Status code is not a Eigen::Status

		Circle result;
		result.center(0) = x0(0);
		result.center(1) = x0(1);
		result.radius    = x0(2);
		return result;
	}

} // namespace lib3dfin
