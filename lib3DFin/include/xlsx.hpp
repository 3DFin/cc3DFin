#pragma once

// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright 2023-2026 Carlos Cabo <carloscabo@uniovi.es>
#include "types.hpp"

namespace lib3dfin
{
#ifdef TDFIN_USES_OPENXLSX
	void export_xlsx(const TreeData& tree_data, const std::string& filename, const ProjectMeta& meta);
#endif
} // namespace lib3dfin
