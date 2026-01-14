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
	double adhoc_verticality(const PointCloud3& cloud)
	{
		// Compute the (3, 3) covariance matrix
		const PointCloud3     centered_cloud = cloud.rowwise() - cloud.colwise().mean();
		const Eigen::Matrix3d cov            = (centered_cloud.transpose() * centered_cloud) / cloud.rows();

		// Compute the eigenvalues and eigenvectors of the covariance
		Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es(cov);

		// eigenvalues are sorted by increasing order so
		// first eigen vector is the normal vector. Its third component is the z component.
		const double normal_z_component = es.eigenvectors()(2, 0);
		return 1.0 - std::abs(normal_z_component);
	}

	Eigen::VectorXd compute_verticality_feature(const PointCloud3& stripe, double scale)
	{
		using kd_tree_t            = nanoflann::KDTreeEigenMatrixAdaptor<const PointCloud3, 3, nanoflann::metric_L2_Simple>;
		const size_t       max_knn = 50000;
		kd_tree_t          kd_tree(3, stripe, 10, 0);
		const Eigen::Index n_points         = stripe.rows();
		const double       sq_search_radius = scale * scale;

		Eigen::VectorXd verticality(n_points);
		verticality.setZero();

		tf::Executor executor;
		tf::Taskflow taskflow;

		taskflow.for_each_index(
		    Eigen::Index(0), n_points, Eigen::Index(1), [&](Eigen::Index point_id)
		    {
           std::vector<nanoflann::ResultItem<Eigen::Index, double>> result_set;

           nanoflann::RadiusResultSet<double, Eigen::Index> radius_result_set(sq_search_radius, result_set);
           const auto                                       num_found =
               kd_tree.index_->radiusSearchCustomCallback(stripe.row(point_id).data(), radius_result_set);

           // not enough point, no feature computation
           if (num_found < 2) return;

           // partial sort for max_knn
           if (num_found > max_knn)
           {
               std::partial_sort(
                   std::begin(result_set), std::begin(result_set) + max_knn, std::end(result_set), nanoflann::IndexDist_Sorter());
           }

           const size_t num_nn = std::min(num_found, max_knn);

           PointCloud3 cloud(num_nn, 3);
           for (size_t id = 0; id < num_nn; ++id) { cloud.row(id) = stripe.row(result_set[id].first); }
           verticality(point_id) = adhoc_verticality(cloud); });
		executor.run(taskflow).get();

		return verticality;
	}
} // namespace lib3dfin
