#pragma once

// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright 2023-2025 Carlos Cabo <carloscabo@uniovi.es>

// local
#include "types.hpp"
#include "verticality.hpp"

namespace lib3dfin
{
	template <typename real_t>
	class SectionExtractor
	{
	  public: // struct
		struct Params
		{
			real_t stem_minimum_height{0.3};
			real_t stem_maximum_height{25.0};
			real_t section_lenght{0.2};
			real_t section_width{0.05};
			uint32_t inner_circle_point_threshold{5};
			real_t stem_diameter_proportion{0.5};
			real_t stem_minimum_diameter{0.09};
			real_t stem_maximum_diameter{1.0};
			real_t circle_point_distance{0.02};
			uint32_t number_points_section{80};
			uint32_t total_number_sectors{16};
			uint32_t minimum_number_sectors{9};
			real_t circle_width{0.02};
		};

		struct circle {
			real_t radius{0};
			Vec3<real_t> center{0, 0, 0};
			bool is_valid{false}; // check
			bool second_time{false}; // second time //TODO not convinced, eliminate this indicator
			real_t sector_percentage{0};
			uint32_t number_points_inner{0};
		};

	  public:
		SectionExtractor(const PointCloud3<real_t>& cloud, const AxesData<real_t>& trees, const Params& params):
			cloud_(cloud), trees_(trees), params_(std::move(params))
		{
		}

		void extract()
		{
			// iterate over the trees
			for(const auto& tree : trees_.tree_descriptors)
			{
			    tree.i

			}
		}

	  private: // members
		const PointCloud3<real_t>& cloud_;
		const AxesData<real_t>& trees_;
		const Params& params_;
	};

} // namespace lib3dfin
