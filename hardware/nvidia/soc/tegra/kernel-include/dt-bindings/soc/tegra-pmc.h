/*
 * Copyright (c) 2017-2020, NVIDIA CORPORATION.  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/*
 * This header provides constants for the binding nvidia,tegra20-pmc
 */

#ifndef _DT_BINDINGS_TEGRA_PMC_H_
#define _DT_BINDINGS_TEGRA_PMC_H_

#define PMC_WAKE_TYPE_GPIO	0
#define PMC_WAKE_TYPE_EVENT	1

#define PMC_TRIGGER_TYPE_NONE		0
#define PMC_TRIGGER_TYPE_RISING		1
#define PMC_TRIGGER_TYPE_FALLING	2
#define PMC_TRIGGER_TYPE_HIGH		4
#define PMC_TRIGGER_TYPE_LOW		8

#endif
