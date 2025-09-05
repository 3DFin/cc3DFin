#pragma once

// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright 2023-2025 Carlos Cabo <carloscabo@uniovi.es>

// local
#include "types.hpp"

// nanoflann
#include <nanoflann.hpp>

// taskflow
#include <taskflow/algorithm/for_each.hpp>
#include <taskflow/taskflow.hpp>

namespace lib3dfin
{
	template <typename real_t>
	real_t adhoc_verticality(const PointCloud3<real_t>& cloud)
	{
		// Compute the (3, 3) covariance matrix
		const PointCloud3<real_t>    centered_cloud = cloud.rowwise() - cloud.colwise().mean();
		const Eigen::Matrix3<real_t> cov            = (centered_cloud.transpose() * centered_cloud) / real_t(cloud.rows());

		// Compute the eigenvalues and eigenvectors of the covariance
		Eigen::SelfAdjointEigenSolver<Eigen::Matrix3<real_t>> es(cov);

		// eigenvalues are sorted by increasing order so
		// first eigen vector is the normal vector. its third component is the z component
		const real_t normal_z_component = es.eigenvectors()(2, 0);
		return real_t(1.0) - std::abs(normal_z_component);
	}

	template <typename real_t>
	Eigen::VectorX<real_t> compute_verticality_feature(const PointCloud3<real_t>& stripe, real_t scale)
	{
		using kd_tree_t            = nanoflann::KDTreeEigenMatrixAdaptor<const PointCloud3<real_t>, 3, nanoflann::metric_L2_Simple>;
		const size_t       max_knn = 50000;
		kd_tree_t          kd_tree(3, stripe, 10, 0);
		const Eigen::Index n_points         = stripe.rows();
		const real_t       sq_search_radius = scale * scale;

		Eigen::VectorX<real_t> verticality(n_points);
		verticality.setZero();

		tf::Executor executor;
		tf::Taskflow taskflow;

		taskflow.for_each_index(
		    Eigen::Index(0), n_points, Eigen::Index(1), [&](Eigen::Index point_id)
		    {
           std::vector<nanoflann::ResultItem<Eigen::Index, real_t>> result_set;

           nanoflann::RadiusResultSet<real_t, Eigen::Index> radius_result_set(sq_search_radius, result_set);
           const auto                                       num_found =
               kd_tree.index_->radiusSearchCustomCallback(stripe.row(point_id).data(), radius_result_set);

           // not enough point, no feature computation
           if (num_found < 2) return;

           // partial sort for max_knn
           if (num_found > max_knn)
           {
               std::partial_sort(
                   result_set.begin(), result_set.begin() + max_knn, result_set.end(), nanoflann::IndexDist_Sorter());
           }

           const size_t num_nn = std::min(num_found, max_knn);

           PointCloud3<real_t> cloud(num_nn, 3);
           for (size_t id = 0; id < num_nn; ++id) { cloud.row(id) = stripe.row(result_set[id].first); }
           verticality(point_id) = adhoc_verticality(cloud); });
		executor.run(taskflow).get();

		return verticality;
	}
} // namespace lib3dfin
