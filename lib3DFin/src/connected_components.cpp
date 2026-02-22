// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright 2023-2025 Carlos Cabo <carloscabo@uniovi.es>

#include "connected_components.hpp"

#include "../third_party/dset/dset.h"

#include <nanoflann.hpp>
#include <taskflow/algorithm/for_each.hpp>
#include <taskflow/taskflow.hpp>

namespace lib3dfin
{

	std::vector<int32_t> connected_components(const PointCloud3& xyz, const double eps, const uint32_t min_samples, tf::Executor& executor)
	{
		using kd_tree_t = nanoflann::KDTreeEigenMatrixAdaptor<PointCloud3, 3, nanoflann::metric_L2_Simple>;

		// Parallel construction of kdtree index is enabled by default, but maybe we have to adapt this
		// for small point clouds
		kd_tree_t    kd_tree(3, xyz, 10, 0);
		const double sq_search_radius = eps * eps;

		const Eigen::Index n_points = xyz.rows();

		tf::Taskflow taskflow;

		std::vector<std::vector<Eigen::Index>> nn_cells(n_points);
		std::vector<bool>                      is_core(n_points, false);
		std::vector<int32_t>                   cluster_id(n_points, NO_CLUSTER_ID);

		taskflow.for_each_index(
		    Eigen::Index(0), n_points, Eigen::Index(1), [&](Eigen::Index point_id)
		    {
            std::vector<nanoflann::ResultItem<Eigen::Index, double>> result_set;

            nanoflann::RadiusResultSet<double, Eigen::Index> radius_result_set(sq_search_radius, result_set);
            const auto                                       num_found =
                kd_tree.index_->radiusSearchCustomCallback(xyz.row(point_id).data(), radius_result_set);

            is_core[point_id] = num_found >= min_samples;  // we include the core sample itself
            std::vector<Eigen::Index> nn_ids;
            nn_ids.reserve(num_found - 1);
            for (const auto& result : result_set)
            {
                if (result.first != point_id) { nn_ids.push_back(result.first); }
            }

            nn_cells[point_id] = std::move(nn_ids); },
		    tf::StaticPartitioner());

		executor.run(taskflow).get();

		// Link core with disjoint set
		// no parallel since it does not seems to lower the runtime
		DisjointSets uf(n_points);
		for (size_t curr_id = 0; curr_id < n_points; ++curr_id)
		{
			if (!is_core[curr_id])
				continue;
			for (const auto nn_id : nn_cells[curr_id])
			{
				if (is_core[nn_id] && curr_id > nn_id)
				{
					uf.unite(curr_id, nn_id);
				}
			}
		};

		// label core points in //
		auto label_core = taskflow.for_each_index(
		    size_t(0), size_t(n_points), size_t(1), [&](size_t curr_id)
		    {
            if (!is_core[curr_id]) return;
            cluster_id[curr_id] = uf.find(curr_id); });

		// label other nodes as borders in //
		// borders are attributed to their nearest cluster
		auto label_border = taskflow.for_each_index(
		    size_t(0), size_t(n_points), size_t(1), [&](size_t curr_id)
		    {
            if (!is_core[curr_id])
            {
                double min_dist = std::numeric_limits<double>::max();
                for (const auto nn_id : nn_cells[curr_id])
                {
                    if (is_core[nn_id])
                    {
                        double dist = (xyz.row(nn_id) - xyz.row(curr_id)).squaredNorm();
                        if (dist < min_dist)
                        {
                            min_dist            = dist;
                            cluster_id[curr_id] = cluster_id[nn_id];
                        }
                    }
                }
            } });

		label_border.succeed(label_core);

		executor.run(taskflow).get();

		return cluster_id;
	}

} // namespace lib3dfin
