// local
#include "../third_party/dset/dset.h"
#include "types.hpp"

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <numeric>
#include <unordered_map>

namespace lib3dfin
{

	template <typename real_t>
	PointCloud2<real_t> extract_largest_cluster(const PointCloud2<real_t>& xy, const std::vector<uint32_t>& labels)
	{
		std::unordered_map<uint32_t, uint32_t> cluster_count;
		uint32_t                               best_cl_id   = 0;
		uint32_t                               max_cl_count = 0;

		for (const auto cl_id : labels)
		{
			cluster_count[cl_id]++;
		}
		// Find cluster with maximum count
		for (const auto& [cl_id, count] : cluster_count)
		{
			if (count > max_cl_count)
			{
				best_cl_id   = cl_id;
				max_cl_count = count;
			}
		}

		// extract only points in the largest cluster
		PointCloud2<real_t> cl_points(max_cl_count, 2);
		uint32_t            new_id = 0;
		for (size_t i = 0; i < labels.size(); ++i)
		{
			if (labels[i] == best_cl_id)
			{
				cl_points.row(new_id++) = xy.row(i);
			}
		}
		return cl_points;
	}

	template <typename real_t>
	std::pair<std::vector<size_t>, std::vector<real_t>> slink_euclidean_2D(const PointCloud2<real_t>& xy)
	{
		const size_t num_points = xy.rows();
		// Best candidate index for point j’s cluster parent (best id)
		std::vector<size_t> pi(num_points);
		// each point begin in its own cluster
		std::iota(std::begin(pi), std::end(pi), 0);

		// The distance at which i merges into the tree (best distance)
		std::vector<real_t> lambda(num_points, std::numeric_limits<real_t>::max());
		std::vector<real_t> dist(num_points, 0);

		for (size_t i = 1; i < num_points; ++i)
		{
			// Compute euclidean distances d(i, j) for any j < i
			for (size_t j = 0; j < i; ++j)
			{
				dist[j] = std::hypot(xy(i, 0) - xy(j, 0), xy(i, 1) - xy(j, 1)); // TODO: sq_dist
			}

			// Update stage
			for (size_t j = 0; j < i; ++j)
			{
				// if the d(i, j) is < to the current merge distance
				if (dist[j] < lambda[j])
				{
					// we try to update the distance of the cluster of j with the current merge distance
					dist[pi[j]] = std::min(dist[pi[j]], lambda[j]);
					// we update the merge distance of j
					lambda[j] = dist[j];
					// we update the id of the cluster
					pi[j] = i;
				}
				else
				{
					// else we try to update the merge distance of the cluster with the d(i,j)
					dist[pi[j]] = std::min(dist[pi[j]], dist[j]);
				}
			}

			// Finalize, reoganize cluster
			for (size_t j = 0; j < i; ++j)
			{
				if (lambda[pi[j]] < lambda[j])
				{
					pi[j] = i;
				}
			}
		}

		// Output pi and lambda
		return {pi, lambda};
	}

	// Extract flat clusters from slink results
	template <typename real_t>
	std::vector<uint32_t> extract_clusters_slink(
	    const std::vector<size_t>& pi,
	    const std::vector<real_t>& lambda,
	    real_t                     threshold)
	{
		const size_t num_points = pi.size();
		DisjointSets uf(num_points);

		for (size_t i = 0; i < num_points; ++i)
		{
			if (lambda[i] <= threshold && i != pi[i])
			{
				uf.unite(i, pi[i]);
			}
		}

		// Assign cluster labels based on root parents
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
		return labels;
	}

	template <typename real_t>
	PointCloud2<real_t> fcluster_slink(const PointCloud2<real_t>& xy, real_t threshold)
	{
		const auto [pi, lambda] = slink_euclidean_2D(xy);
		const auto labels       = extract_clusters_slink(pi, lambda, threshold);

		return extract_largest_cluster(xy, labels);
	}

	template <typename real_t>
	PointCloud2<real_t> fcluster_naive(const PointCloud2<real_t>& xy, real_t threshold)
	{
		const size_t          num_points = xy.rows();
		std::vector<uint32_t> labels(num_points);
		// initialize each point in its own cluster
		std::iota(std::begin(labels), std::end(labels), 0);

		for (size_t i = 0; i < num_points; ++i)
		{
			for (size_t j = i + 1; j < num_points; ++j)
			{
				if (std::hypot(xy(i, 0) - xy(j, 0), xy(i, 1) - xy(j, 1)) <= threshold) //TODO: sq_threshold
				{
					const uint32_t ci = labels[i];
					const uint32_t cj = labels[j];
					if (ci != cj)
					{
						// look for cj cluster and remap them to ci
						for (size_t k = 0; k < num_points; ++k)
							if (labels[k] == cj)
								labels[k] = ci;
					}
				}
			}
		}

		return extract_largest_cluster(xy, labels);
	}

} // namespace lib3dfin
