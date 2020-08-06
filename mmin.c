/**
 * @file mmin.c
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
#include <stdlib.h>
#include <string.h>

#include "mmin.h"
#include "filter_private.h"
#include "print.h"

/**
 * This implements a moving-minimum filter that averages the lowest values
 * passed in. This is based on the fact that most of the noise seen by PTP is
 * non-Gaussian and non-zero-mean. Instead we typically see a normal low-noise
 * signal with occasional high latency spikes caused by buffering in the
 * network switches. Since the additional latency is always positive, we can
 * simply select the minimum of the input values.
 *
 * To improve robustness we support average several input samples instead of
 * just taking the absolute minimum. We also support throwing away the very low
 * end to avoid ingesting occasional bad timestamps.
 *
 * Filter length is a trade off between latency and noise rejection. For
 * example, with a 10 sample filter and a 1-3 range, we can reject up to ~60%
 * of network noise at the cost of up to ~7 seconds of latency.
 *
 *      0     1     2     3     4     5     6     7     8     9
 *   [ -10 | 0.3 | 0.4 | 0.5 | 0.6 | 2.0 | 3.0 | 6.0 | 9.0 | 100 ]
 *      |     '-----.------'
 *      |           '-- Next three samples averaged.
 *      '- Lowest sample is thrown away.
 *
 * Note, we incur the most latency when the input signal is *increasing*. When
 * the input signal is decreasing, the most recent sample will likely also be
 * one of the minimum samples and be included in the average sooner.
 */
struct mmin {
	struct filter filter;
	int cnt;
	int len;
	int index;
	/* Start index for averaging. */
	int start;
	/* Stop index for averaging. */
	int stop;
	/* Indices sorted by value. */
	int *order;
	/* Values stored in circular buffer. */
	tmv_t *samples;
};

int configured_mmin_start = 0;
int configured_mmin_stop = 0;

static void mmin_destroy(struct filter *filter)
{
	struct mmin *m = container_of(filter, struct mmin, filter);
	free(m->order);
	free(m->samples);
	free(m);
}

static tmv_t mmin_sample(struct filter *filter, tmv_t sample)
{
	struct mmin *m = container_of(filter, struct mmin, filter);
	int i;

	m->samples[m->index] = sample;
	if (m->cnt < m->len) {
		m->cnt++;
	} else {
		/* Remove index of the replaced value from order. */
		for (i = 0; i < m->cnt; i++)
			if (m->order[i] == m->index)
				break;
		for (; i + 1 < m->cnt; i++)
			m->order[i] = m->order[i + 1];
	}

	/* Insert index of the new value to order. */
	for (i = m->cnt - 1; i > 0; i--) {
		if (m->samples[m->order[i - 1]] <= m->samples[m->index])
			break;
		m->order[i] = m->order[i - 1];
	}
	m->order[i] = m->index;

	m->index = (1 + m->index) % m->len;

	/* Sum the low range */
	tmv_t avg = tmv_zero();
	int cnt = 0;
	for (i = m->start; i <= m->stop && i < m->cnt; i++) {
		avg = tmv_add(avg, m->samples[m->order[i]]);
		cnt = cnt + 1;
	}
	if (cnt == 0)
		return sample;
	return tmv_div(avg, cnt);
}

static void mmin_reset(struct filter *filter)
{
	struct mmin *m = container_of(filter, struct mmin, filter);
	m->cnt = 0;
	m->index = 0;
}

struct filter *mmin_create(int length)
{
	struct mmin *m;

	if (length < 1)
		return NULL;
	if (length <= configured_mmin_start)
		return NULL;
	if (length <= configured_mmin_stop)
		return NULL;
	if (configured_mmin_start >= configured_mmin_stop)
		return NULL;
	m = calloc(1, sizeof(*m));
	if (!m)
		return NULL;
	m->filter.destroy = mmin_destroy;
	m->filter.sample = mmin_sample;
	m->filter.reset = mmin_reset;
	m->order = calloc(1, length * sizeof(*m->order));
	if (!m->order) {
		free(m);
		return NULL;
	}
	m->samples = calloc(1, length * sizeof(*m->samples));
	if (!m->samples) {
		free(m->order);
		free(m);
		return NULL;
	}
	m->len = length;
	m->start = configured_mmin_start;
	m->stop = configured_mmin_stop;
	return &m->filter;
}
