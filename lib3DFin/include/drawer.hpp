#pragma once

// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright 2023-2026 Carlos Cabo <carloscabo@uniovi.es>

#include "config.hpp"

namespace lib3dfin
{

	class TDFProcessing;

	class TDFDrawer
	{
	  public:
		virtual ~TDFDrawer() = default;

		virtual void drawAll(const TDFProcessing& result, const DrawResultParams& param) = 0;
	};

} // namespace lib3dfin
