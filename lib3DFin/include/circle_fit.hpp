#pragma once

// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright 2023-2025 Carlos Cabo <carloscabo@uniovi.es

#include "types.hpp"

#include <Eigen/Dense>
#include <Eigen/QR>
#include <unsupported/Eigen/NonLinearOptimization>

namespace lib3dfin
{

	template <typename real_t>
	struct EigenCircleFitFunctor
	{
		const PointCloud2<real_t>& data;

		EigenCircleFitFunctor(const PointCloud2<real_t>& xy)
		    : n_values(xy.rows())
		    , data(xy)
		{
		}

		// Evaluate residual
		int operator()(const Eigen::VectorX<real_t>& x, Eigen::VectorX<real_t>& fvec) const
		{
			const real_t a = x(0);
			const real_t b = x(1);
			const real_t r = x(2);
			for (Eigen::Index i = 0; i < data.rows(); ++i)
			{
				fvec(i) = std::hypot(data(i, 0) - a, data(i, 1) - b) - r;
			}
			return 0;
		}

		// Compute jacobian
		int df(const Eigen::VectorX<real_t> x, Eigen::MatrixX<real_t>& fjac) const
		{
			const real_t a = x(0);
			const real_t b = x(1);
			const real_t r = x(2);
			for (Eigen::Index i = 0; i < data.rows(); ++i)
			{
				const real_t dx = data(i, 0) - a;
				const real_t dy = data(i, 1) - b;
				const real_t d  = std::hypot(dx, dy);

				if (d < real_t(1e-10)) // add robustness (avoid division by zero)
				{
					fjac(i, 0) = fjac(i, 1) = 0;
				}
				else
				{
					fjac(i, 0) = -dx / d;
					fjac(i, 1) = -dy / d;
				}
				fjac(i, 2) = real_t(-1.0);
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

	template <typename real_t>
	Eigen::Vector3<real_t> algebraicTaubinCircleFit(const PointCloud2<real_t>& xy)
	{
		const size_t num_points = xy.rows();

		// Linear system
		Eigen::MatrixX<real_t> ZXY(num_points, 3);

		// Compute centroid
		const Eigen::Vector2<real_t> centroid = xy.colwise().mean();

		// Center the data
		ZXY.col(1) = xy.col(0).array() - centroid(0);
		ZXY.col(2) = xy.col(1).array() - centroid(1);

		// Compute Z = X^2 + Y^2
		const Eigen::VectorX<real_t> Z      = ZXY.col(1).array().square() + ZXY.col(2).array().square();
		const real_t                 Z_mean = Z.mean();

		// Normalize Z
		ZXY.col(0) = (Z.array() - Z_mean) / (2.0 * sqrt(Z_mean));

		// Solve by SVD
		Eigen::JacobiSVD<Eigen::MatrixX<real_t>> svd(ZXY, Eigen::ComputeFullV);
		const Eigen::MatrixX<real_t>             V = svd.matrixV();

		Eigen::Vector3<real_t> A = V.col(2);
		A(0) /= (2.0 * sqrt(Z_mean));
		Eigen::Vector4<real_t> A_mat;
		A_mat << A, -Z_mean * A(0);

		// Compute parameters
		const real_t a = -A_mat(1) / (real_t(2.0) * A_mat(0)) + centroid(0);
		const real_t b = -A_mat(2) / (real_t(2.0) * A_mat(0)) + centroid(1);
		const real_t r = sqrt(A_mat(1) * A_mat(1) + A_mat(2) * A_mat(2) - real_t(4.0) * A_mat(0) * A_mat(3)) / std::abs(A_mat(0)) / real_t(2.0);

		return Eigen::Vector3<real_t>(a, b, r);
	}

	template <typename real_t>
	Eigen::Vector3<real_t> LMCircleFit(const PointCloud2<real_t>& xy)
	{
		if (xy.rows() < 3)
			throw std::invalid_argument("Circle fit need at least 3 points");

		// Initialization by Taubin method
		Eigen::VectorX<real_t> x0 = algebraicTaubinCircleFit(xy);

		EigenCircleFitFunctor                                    functor(xy);
		Eigen::LevenbergMarquardt<EigenCircleFitFunctor<real_t>> lm(functor);

		// 40 iters should be enough.
		// See H. Abdul-Rahman and N. Chernov, 2013. : "The GN and LM normally converge in 5–10 iterations."
		lm.parameters.maxfev = 40;
		lm.parameters.xtol   = 1.4e-8;

		int status = lm.minimize(x0); // TODO: check status
		return Eigen::Vector3<real_t>(x0(0), x0(1), x0(2));
	}

} // namespace lib3dfin
