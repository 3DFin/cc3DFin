#pragma once

// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright 2023-2025 Carlos Cabo <carloscabo@uniovi.es>

// local
#include "../third_party/dset/dset.h"
#include "types.hpp"

#include <cstddef>
#include <cstdint>
#include <unordered_map>

namespace lib3dfin
{

	inline PointCloud2 extract_largest_cluster(const PointCloud2& xy, const std::vector<uint32_t>& labels)
	{
		std::unordered_map<uint32_t, uint32_t> cluster_count;
		uint32_t                               best_cl_id   = 0;
		uint32_t                               max_cl_count = 0;

		for (const auto cl_id : labels)
		{
			cluster_count[cl_id]++;
		}
		// Find cluster with maximum count of vertices
		for (const auto& [cl_id, count] : cluster_count)
		{
			if (count > max_cl_count)
			{
				best_cl_id   = cl_id;
				max_cl_count = count;
			}
		}

		// Extract only points from the largest cluster
		PointCloud2  cl_points(max_cl_count, 2);
		Eigen::Index new_id = 0;
		for (size_t i = 0; i < labels.size(); ++i)
		{
			if (labels[i] == best_cl_id)
			{
				cl_points.row(new_id++) = xy.row(i);
			}
		}
		return cl_points;
	}

	PointCloud2 fcluster_naive(const PointCloud2& xy, double threshold)
	{
		const double sq_threshold = threshold * threshold;
		const size_t num_points   = xy.rows();
		DisjointSets uf(num_points);

		for (size_t i = 0; i < num_points; ++i)
		{
			for (size_t j = i + 1; j < num_points; ++j)
			{
				const double deltax = xy(i, 0) - xy(j, 0);
				const double deltay = xy(i, 1) - xy(j, 1);

				if (deltax * deltax + deltay * deltay <= sq_threshold)
				{
					uf.unite(i, j);
				}
			}
		}

		// Assign cluster labels based on root parent
		std::vector<uint32_t>              labels(num_points);
		std::unordered_map<size_t, size_t> cluster_id;
		size_t                             current_label = 0;

		for (uint32_t i = 0; i < num_points; ++i)
		{
			const uint32_t root = uf.find(i);
			if (cluster_id.count(root) == 0)
				cluster_id[root] = current_label++;
			labels[i] = cluster_id[root];
		}

		return extract_largest_cluster(xy, labels);
	}

} // namespace lib3dfin
