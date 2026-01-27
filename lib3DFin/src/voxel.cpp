// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright 2023-2025 Carlos Cabo <carloscabo@uniovi.es>

#include "voxel.hpp"

#include <spdlog/spdlog.h>
#include <taskflow/algorithm/for_each.hpp>
#include <taskflow/algorithm/scan.hpp>
#include <taskflow/algorithm/sort.hpp>
#include <taskflow/algorithm/transform.hpp>
#include <taskflow/taskflow.hpp>

namespace lib3dfin
{

	std::tuple<PointCloud3, VecIndex<uint32_t>> voxelize(
	    const PointCloud3& xyz,
	    const double       res_xy,
	    const double       res_z,
	    const bool         verbose)
	{
		// number of bit used to encode one dimension
		constexpr uint64_t voxel_bits     = 21;
		constexpr uint64_t two_voxel_bits = 42;
		constexpr uint64_t num_cells      = 2097151; // 2^21 -1 TODO: make this an exception if we need more

		// The coordinate minima
		const auto start_total = std::chrono::high_resolution_clock::now();

		if (verbose)
			spdlog::info("[Voxelization] Grid size: {} x {} x {} m", res_xy, res_xy, res_z);

		tf::Executor executor;
		tf::Taskflow tf;

		const Eigen::Index num_points = xyz.rows();

		// Lambda to compute min in one dimension
		const auto min_one_dim = [&xyz, num_points](const Eigen::Index id_dim, double& min_dim)
		{
			min_dim = xyz(0, id_dim);
			for (Eigen::Index point_id = 1; point_id < num_points; ++point_id)
			{
				if (xyz(point_id, id_dim) < min_dim)
					min_dim = xyz(point_id, id_dim);
			};
		};

		// Parallel min coeff
		double min_x, min_y, min_z;
		tf.emplace([&]()
		           { min_one_dim(0, min_x); });
		tf.emplace([&]()
		           { min_one_dim(1, min_y); });
		tf.emplace([&]()
		           { min_one_dim(2, min_z); });
		executor.run(tf).wait();

		const Vec3 min_vec(min_x, min_y, min_z);

		// Lambda to compute voxel hashing
		const auto create_hash = [&](const Vec3& point) -> uint64_t
		{
			return ((static_cast<uint64_t>((point(2) - min_vec(2)) / res_z) << two_voxel_bits) | (static_cast<uint64_t>((point(1) - min_vec(1)) / res_xy) << voxel_bits)) | static_cast<uint64_t>((point(0) - min_vec(0)) / res_xy);
		};

		std::vector<uint64_t> hashes(num_points);
		VecIndex<uint32_t>    cloud_to_vox_ind(num_points);
		PointCloud3           vox_pc;
		VecIndex<uint32_t>    vox_to_cloud_ind;

		std::vector<uint32_t> first_point_in_vox(num_points, 0);
		first_point_in_vox[0] = 1;

		std::vector<Eigen::Index>           sorted_indices(num_points);
		std::vector<Eigen::Index>::iterator first_it_indices = sorted_indices.begin();
		std::vector<Eigen::Index>::iterator end_it_indices   = sorted_indices.end();
		std::iota(first_it_indices, end_it_indices, 0);

		// Create hashes
		auto hashing = tf.for_each(
		                     std::cref(first_it_indices), std::cref(end_it_indices), [&](const Eigen::Index point_id)
		                     { hashes[point_id] = create_hash(xyz.row(point_id)); })
		                   .name("hashing");

		// second order point by dimensions
		auto sort_indices = tf.sort(
		                          std::cref(first_it_indices), std::cref(end_it_indices), [&](const Eigen::Index a, Eigen::Index b)
		                          { return hashes[a] < hashes[b]; })
		                        .name("sort_indices"); // note this is not a stable sort

		// In the sorted index find first representent one voxel cell
		auto unique = tf.for_each_index(
		                    Eigen::Index(1), Eigen::Index(num_points), Eigen::Index(1), [&](const Eigen::Index point_id)
		                    {
                                  if (hashes[sorted_indices[point_id]] != hashes[sorted_indices[point_id - 1]])
                                  {
                                      first_point_in_vox[point_id] = 1;
                                  } })
		                  .name("unique");

		// count and generate voxel id with a parallel scan
		auto count_voxels =
		    tf.inclusive_scan(
		          first_point_in_vox.begin(), first_point_in_vox.end(), first_point_in_vox.begin(), std::plus<int>{})
		        .name("count_voxels");

		// allocate voxel point cloud and vox_to_cloud
		auto allocate = tf.emplace(
		    [&]()
		    {
			    vox_pc = PointCloud3(first_point_in_vox.back(), 3);
		    });

		// Precomputed shifts for each dimensional composant of a full hashed code
		const double centroid_shift_x = min_vec(0) + res_xy / 2.0;
		const double centroid_shift_y = min_vec(1) + res_xy / 2.0;
		const double centroid_shift_z = min_vec(2) + res_z / 2.0;

		auto fill_vox_pc = tf.for_each_index(
		                         Eigen::Index(0), Eigen::Index(num_points), Eigen::Index(1), [&](const Eigen::Index point_id)
		                         {
                                       const auto voxel_id             = first_point_in_vox[point_id] - 1;  // it starts at 1
                                       const auto real_point_id        = sorted_indices[point_id];
                                       cloud_to_vox_ind(real_point_id) = voxel_id;
                                       //  we account for the first point here
                                       //  maybe it could be better to init. it in the allocation tasks
                                       if (point_id == 0 || voxel_id != first_point_in_vox[point_id - 1] - 1)
                                       {
                                           const uint64_t hash_val    = hashes[real_point_id];

                                           const uint64_t z_code_val = hash_val >> two_voxel_bits;
                                           const uint64_t y_code_val = (hash_val >> voxel_bits) & num_cells;
                                           const uint64_t x_code_val = hash_val & num_cells;

                                           vox_pc(voxel_id, 0) = x_code_val * res_xy + centroid_shift_x;
                                           vox_pc(voxel_id, 1) = y_code_val * res_xy + centroid_shift_y;
                                           vox_pc(voxel_id, 2) = z_code_val * res_z + centroid_shift_z;
                                       } })
		                       .name("fill_vox_pc");

		// Taskflow workflow
		hashing.precede(sort_indices);
		sort_indices.precede(unique);
		unique.precede(count_voxels);
		count_voxels.precede(allocate);
		allocate.precede(fill_vox_pc);

		// Launch tasks
		executor.run(tf).wait();

		if (verbose)
		{
			std::stringstream log;

			log << "[Voxelization] Total time: "
			    << std::chrono::duration_cast<std::chrono::milliseconds>(
			           std::chrono::high_resolution_clock::now() - start_total)
			           .count()
			    << " ms / "
			    << std::setprecision(3) << std::fixed << " " << num_points / 1.0e6 << " M points -> "
			    << vox_pc.rows() / 1.0e6 << " M voxels ("
			    << std::setprecision(1)
			    << vox_pc.rows() * 100 / static_cast<double>(num_points) << "%)";

			spdlog::info(log.str());
		}

		return {vox_pc, cloud_to_vox_ind};
	}

} // namespace lib3dfin
