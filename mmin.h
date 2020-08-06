/**
 * @file mave.h
 * @brief Implements a moving minimum.
 * @note Copyright (C) 2019 Andy Spencer <aspencer@spacex.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */
#ifndef HAVE_MMIN_H
#define HAVE_MMIN_H

#include "filter.h"

/**
 * The lower index of the range of values to sum.
 */
extern int configured_mmin_start;

/**
 * The upper index of the range of values to sum.
 */
extern int configured_mmin_stop;

struct filter *mmin_create(int length);

#endif
