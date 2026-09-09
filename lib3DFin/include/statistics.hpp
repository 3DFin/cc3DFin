#pragma once

// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright 2023-2026 Carlos Cabo <carloscabo@uniovi.es>

#include <Eigen/Dense>

namespace lib3dfin
{

	namespace internal
	{
		inline std::array<double, 2> compute_quartiles(const Eigen::VectorXd& data_vector)
		{

			constexpr std::array<double, 2> bounds = {0.25, 0.75};
			if (data_vector.size() == 0)
			{
				return {0., 0.};
			}
			const Eigen::Index max_id       = std::ceil(static_cast<double>(data_vector.size()) * bounds[1]);
			const Eigen::Index num_elements = max_id + 1;

			// Create a deep copy to sort the data
			Eigen::VectorXd partial_data_sorted(num_elements);
			std::partial_sort_copy(std::begin(data_vector), std::begin(data_vector) + num_elements, std::begin(partial_data_sorted), std::end(partial_data_sorted));

			std::array<double, 2> result{0., 0.};

			// compute with linear interpolation (it's the default in numpy)
			for (Eigen::Index id_bound = 0; id_bound < 2; ++id_bound)
			{
				const double id_pos   = bounds[id_bound] * (data_vector.size() - 1);
				const Eigen::Index id_left  = static_cast<Eigen::Index>(std::floor(id_pos));
				const Eigen::Index id_right = static_cast<Eigen::Index>(std::ceil(id_pos));

				if (id_left == id_right)
				{
					result[id_bound] = partial_data_sorted(id_left);
					continue;
				}

				const double weight = id_pos - id_left;
				result[id_bound]    = (partial_data_sorted(id_left) * (1.0 - weight)) + (partial_data_sorted(id_right) * weight);
			}
			return result;
		}
	} // namespace internal

	inline Eigen::VectorX<bool> interquartile_range(const Eigen::VectorXd& data_vector,
	                                                double                 n_range = 1.5)
	{

		const std::array<double, 2> quartiles = internal::compute_quartiles(data_vector);

		const double iqr = quartiles[1] - quartiles[0];

		const double lower_bound = quartiles[0] - (iqr * n_range);
		const double upper_bound = quartiles[1] + (iqr * n_range);

		return (data_vector.array() < lower_bound || data_vector.array() > upper_bound);
	}

} // namespace lib3dfin
