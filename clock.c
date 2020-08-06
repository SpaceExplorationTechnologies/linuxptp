/**
 * @file clock.c
 * @note Copyright (C) 2011 Richard Cochran <richardcochran@gmail.com>
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
#include <errno.h>
#include <poll.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <sys/queue.h>
#include "address.h"
#include "bmc.h"
#include "clock.h"
#include "clockadj.h"
#include "clockcheck.h"
#include "foreign.h"
#include "filter.h"
#include "gpstime.h"
#include "missing.h"
#include "msg.h"
#include "phc.h"
#include "port.h"
#include "servo.h"
#include "stats.h"
#include "stats_file.h"
#include "print.h"
#include "tlv.h"
#include "tmv.h"
#include "uds.h"
#include "util.h"

#define N_CLOCK_PFD (N_POLLFD + 1) /* one extra per port, for the fault timer */
#define POW2_41 ((double)(1ULL << 41))

/* This takes its value from the configuration file. (see ptp4l.c) */
enum filter_type configured_offset_filter = FILTER_MOVING_MEDIAN;

/* This takes its value from the configuration file. (see ptp4l.c) */
int configured_offset_filter_length = 1;

struct port {
	LIST_ENTRY(port) list;
};

struct freq_estimator {
	tmv_t origin1;
	tmv_t ingress1;
	unsigned int max_count;
	unsigned int count;
};

struct clock_stats {
	struct stats *offset;
	struct stats *freq;
	struct stats *delay;
	unsigned int max_count;
};

struct clock_subscriber {
	LIST_ENTRY(clock_subscriber) list;
	uint8_t events[EVENT_BITMASK_CNT];
	struct PortIdentity targetPortIdentity;
	struct address addr;
	UInteger16 sequenceId;
	time_t expiration;
};

struct clock {
	clockid_t clkid;
	struct servo *servo;
	enum servo_type servo_type;
	struct defaultDS dds;
	struct dataset default_dataset;
	struct currentDS cur;
	struct parent_ds dad;
	struct timePropertiesDS tds;
	struct ClockIdentity ptl[PATH_TRACE_MAX];
	struct foreign_clock *best;
	struct ClockIdentity best_id;
	LIST_HEAD(ports_head, port) ports;
	struct port *uds_port;
	struct pollfd *pollfd;
	int pollfd_valid;
	int nports; /* does not include the UDS port, unless UDS port has clock associated */
	int last_port_number;
	int free_running;
	int freq_est_interval;
	int grand_master_capable; /* for 802.1AS only */
	int utc_timescale;
	int utc_offset_set;
	int leap_set;
	int kernel_leap;
	int utc_offset;  /* grand master role */
	int time_flags;  /* grand master role */
	int time_source; /* grand master role */
	enum servo_state servo_state;
	tmv_t master_offset_raw;  /* telemetered as master_offset */
	tmv_t master_offset;	  /* telemetered as master_offset_filtered */
	tmv_t path_delay;
	tmv_t path_delay_raw;
	int path_delay_set; /* 0 until 'path_delay' is set */
	struct filter *delay_filter;
	struct filter *offset_filter;
	struct freq_estimator fest;
	struct time_status_np status;
	double nrr;
	tmv_t c1;
	tmv_t c2;
	tmv_t t1;
	tmv_t t2;
	struct clock_description desc;
	struct clock_stats stats;
	int stats_interval;
	/*
	 * The file to output stats to.
	 */
	const char *stats_filename;
	/*
	 * Last time we printed output for telemetry.
	 */
	int64_t last_stats_file_update;
	/*
	 * The leap second table filename.
	 */
	const char *leap_table_filename;
	/*
	 * Last time we re-read the leap second table.
	 */
	int64_t last_leap_table_update;
	/*
	 * Leap table provided by libgpstime.
	 */
	struct gpstime_leap_table_t *leap_table;
	struct clockcheck *sanity_check;
	struct interface uds_interface;
	LIST_HEAD(clock_subscribers_head, clock_subscriber) subscribers;
	const char* uds_clock_filename;
};

struct clock the_clock;

/*
 * Lookup table for the clockAccuracy attribute.
 * See IEEE Std 1588-2008, Table 6 -- clockAccuracy enumeration
 * These values are in nanoseconds (for consistency with other output).
 */
static const int64_t CLOCK_ACCURACIES[0x100] = {
	[0 ... 0x1F]	= -1,
	[0x20]		= 25,
	[0x21]		= 100,
	[0x22]		= 250,
	[0x23]		= 1000,
	[0x24]		= 2500,
	[0x25]		= 10000,
	[0x26]		= 25000,
	[0x27]		= 100000,
	[0x28]		= 250000,
	[0x29]		= 1000000,
	[0x2A]		= 2500000,
	[0x2B]		= 10000000,
	[0x2C]		= 25000000,
	[0x2D]		= 100000000,
	[0x2E]		= 250000000,
	[0x2F]		= NS_PER_SEC,
	[0x30]		= 10 * NS_PER_SEC,
	[0x31]		= INT64_MAX,
	[0x32 ... 0xFF]	= -1
};

static void handle_state_decision_event(struct clock *c);
static int clock_resize_pollfd(struct clock *c, int new_nports);
static void clock_remove_port(struct clock *c, struct port *p);

static int cid_eq(struct ClockIdentity *a, struct ClockIdentity *b)
{
	return 0 == memcmp(a, b, sizeof(*a));
}

#ifndef LIST_FOREACH_SAFE
#define	LIST_FOREACH_SAFE(var, head, field, tvar)			\
	for ((var) = LIST_FIRST((head));				\
	    (var) && ((tvar) = LIST_NEXT((var), field), 1);		\
	    (var) = (tvar))
#endif

static void remove_subscriber(struct clock_subscriber *s)
{
	LIST_REMOVE(s, list);
	free(s);
}

static void clock_update_subscription(struct clock *c, struct ptp_message *req,
				      uint8_t *bitmask, uint16_t duration)
{
	struct clock_subscriber *s;
	int i, remove = 1;
	struct timespec now;

	for (i = 0; i < EVENT_BITMASK_CNT; i++) {
		if (bitmask[i]) {
			remove = 0;
			break;
		}
	}

	LIST_FOREACH(s, &c->subscribers, list) {
		if (!memcmp(&s->targetPortIdentity, &req->header.sourcePortIdentity,
		            sizeof(struct PortIdentity))) {
			/* Found, update the transport address and event
			 * mask. */
			if (!remove) {
				s->addr = req->address;
				memcpy(s->events, bitmask, EVENT_BITMASK_CNT);
				clock_gettime(CLOCK_MONOTONIC, &now);
				s->expiration = now.tv_sec + duration;
			} else {
				remove_subscriber(s);
			}
			return;
		}
	}
	if (remove)
		return;
	/* Not present yet, add the subscriber. */
	s = malloc(sizeof(*s));
	if (!s) {
		pr_err("failed to allocate memory for a subscriber");
		return;
	}
	s->targetPortIdentity = req->header.sourcePortIdentity;
	s->addr = req->address;
	memcpy(s->events, bitmask, EVENT_BITMASK_CNT);
	clock_gettime(CLOCK_MONOTONIC, &now);
	s->expiration = now.tv_sec + duration;
	s->sequenceId = 0;
	LIST_INSERT_HEAD(&c->subscribers, s, list);
}

static void clock_get_subscription(struct clock *c, struct ptp_message *req,
				   uint8_t *bitmask, uint16_t *duration)
{
	struct clock_subscriber *s;
	struct timespec now;

	LIST_FOREACH(s, &c->subscribers, list) {
		if (!memcmp(&s->targetPortIdentity, &req->header.sourcePortIdentity,
			    sizeof(struct PortIdentity))) {
			memcpy(bitmask, s->events, EVENT_BITMASK_CNT);
			clock_gettime(CLOCK_MONOTONIC, &now);
			if (s->expiration < now.tv_sec)
				*duration = 0;
			else
				*duration = s->expiration - now.tv_sec;
			return;
		}
	}
	/* A client without entry means the client has no subscriptions. */
	memset(bitmask, 0, EVENT_BITMASK_CNT);
	*duration = 0;
}

static void clock_flush_subscriptions(struct clock *c)
{
	struct clock_subscriber *s, *tmp;

	LIST_FOREACH_SAFE(s, &c->subscribers, list, tmp) {
		remove_subscriber(s);
	}
}

static void clock_prune_subscriptions(struct clock *c)
{
	struct clock_subscriber *s, *tmp;
	struct timespec now;

	clock_gettime(CLOCK_MONOTONIC, &now);
	LIST_FOREACH_SAFE(s, &c->subscribers, list, tmp) {
		if (s->expiration <= now.tv_sec) {
			pr_info("subscriber %s timed out",
				pid2str(&s->targetPortIdentity));
			remove_subscriber(s);
		}
	}
}

void clock_send_notification(struct clock *c, struct ptp_message *msg,
			     int msglen, enum notification event)
{
	unsigned int event_pos = event / 8;
	uint8_t mask = 1 << (event % 8);
	struct port *uds = c->uds_port;
	struct clock_subscriber *s;

	LIST_FOREACH(s, &c->subscribers, list) {
		if (!(s->events[event_pos] & mask))
			continue;
		/* send event */
		msg->header.sequenceId = htons(s->sequenceId);
		s->sequenceId++;
		msg->management.targetPortIdentity.clockIdentity =
			s->targetPortIdentity.clockIdentity;
		msg->management.targetPortIdentity.portNumber =
			htons(s->targetPortIdentity.portNumber);
		msg->address = s->addr;
		port_forward_to(uds, msg);
	}
}

void clock_destroy(struct clock *c)
{
	struct port *p, *tmp;

	clock_flush_subscriptions(c);
	LIST_FOREACH_SAFE(p, &c->ports, list, tmp) {
		clock_remove_port(c, p);
	}
	port_close(c->uds_port);
	free(c->pollfd);
	if (c->clkid != CLOCK_REALTIME) {
		phc_close(c->clkid);
	}
	servo_destroy(c->servo);
	filter_destroy(c->delay_filter);
	filter_destroy(c->offset_filter);
	stats_destroy(c->stats.offset);
	stats_destroy(c->stats.freq);
	stats_destroy(c->stats.delay);

	if (c->sanity_check)
		clockcheck_destroy(c->sanity_check);
	memset(c, 0, sizeof(*c));
	msg_cleanup();
}

static int clock_fault_timeout(struct port *port, int set)
{
	struct fault_interval i;

	if (!set) {
		pr_debug("clearing fault on port %d", port_number(port));
		return port_set_fault_timer_lin(port, 0);
	}

	fault_interval(port, last_fault_type(port), &i);

	if (i.type == FTMO_LINEAR_SECONDS) {
		pr_debug("waiting %d seconds to clear fault on port %d",
			 i.val, port_number(port));
		return port_set_fault_timer_lin(port, i.val);
	} else if (i.type == FTMO_LOG2_SECONDS) {
		pr_debug("waiting 2^{%d} seconds to clear fault on port %d",
			 i.val, port_number(port));
		return port_set_fault_timer_log(port, 1, i.val);
	}

	pr_err("Unsupported fault interval type %d", i.type);
	return -1;
}

static void clock_freq_est_reset(struct clock *c)
{
	c->fest.origin1 = tmv_zero();
	c->fest.ingress1 = tmv_zero();
	c->fest.count = 0;
};

static void clock_management_send_error(struct port *p,
					struct ptp_message *msg, int error_id)
{
	if (port_management_error(port_identity(p), p, msg, error_id))
		pr_err("failed to send management error status");
}

/* The 'p' and 'req' paremeters are needed for the GET actions that operate
 * on per-client datasets. If such actions do not apply to the caller, it is
 * allowed to pass both of them as NULL.
 */
static int clock_management_fill_response(struct clock *c, struct port *p,
					  struct ptp_message *req,
					  struct ptp_message *rsp, int id)
{
	int datalen = 0, respond = 0;
	struct management_tlv *tlv;
	struct management_tlv_datum *mtd;
	struct time_status_np *tsn;
	struct grandmaster_settings_np *gsn;
	struct subscribe_events_np *sen;
	struct PTPText *text;

	tlv = (struct management_tlv *) rsp->management.suffix;
	tlv->type = TLV_MANAGEMENT;
	tlv->id = id;

	switch (id) {
	case TLV_USER_DESCRIPTION:
		text = (struct PTPText *) tlv->data;
		text->length = c->desc.userDescription.length;
		memcpy(text->text, c->desc.userDescription.text, text->length);
		datalen = 1 + text->length;
		respond = 1;
		break;
	case TLV_DEFAULT_DATA_SET:
		memcpy(tlv->data, &c->dds, sizeof(c->dds));
		datalen = sizeof(c->dds);
		respond = 1;
		break;
	case TLV_CURRENT_DATA_SET:
		memcpy(tlv->data, &c->cur, sizeof(c->cur));
		datalen = sizeof(c->cur);
		respond = 1;
		break;
	case TLV_PARENT_DATA_SET:
		memcpy(tlv->data, &c->dad.pds, sizeof(c->dad.pds));
		datalen = sizeof(c->dad.pds);
		respond = 1;
		break;
	case TLV_TIME_PROPERTIES_DATA_SET:
		memcpy(tlv->data, &c->tds, sizeof(c->tds));
		datalen = sizeof(c->tds);
		respond = 1;
		break;
	case TLV_PRIORITY1:
		mtd = (struct management_tlv_datum *) tlv->data;
		mtd->val = c->dds.priority1;
		datalen = sizeof(*mtd);
		respond = 1;
		break;
	case TLV_PRIORITY2:
		mtd = (struct management_tlv_datum *) tlv->data;
		mtd->val = c->dds.priority2;
		datalen = sizeof(*mtd);
		respond = 1;
		break;
	case TLV_DOMAIN:
		mtd = (struct management_tlv_datum *) tlv->data;
		mtd->val = c->dds.domainNumber;
		datalen = sizeof(*mtd);
		respond = 1;
		break;
	case TLV_SLAVE_ONLY:
		mtd = (struct management_tlv_datum *) tlv->data;
		mtd->val = c->dds.flags & DDS_SLAVE_ONLY;
		datalen = sizeof(*mtd);
		respond = 1;
		break;
	case TLV_CLOCK_ACCURACY:
		mtd = (struct management_tlv_datum *) tlv->data;
		mtd->val = c->dds.clockQuality.clockAccuracy;
		datalen = sizeof(*mtd);
		respond = 1;
		break;
	case TLV_TRACEABILITY_PROPERTIES:
		mtd = (struct management_tlv_datum *) tlv->data;
		mtd->val = c->tds.flags & (TIME_TRACEABLE|FREQ_TRACEABLE);
		datalen = sizeof(*mtd);
		respond = 1;
		break;
	case TLV_TIMESCALE_PROPERTIES:
		mtd = (struct management_tlv_datum *) tlv->data;
		mtd->val = c->tds.flags & PTP_TIMESCALE;
		datalen = sizeof(*mtd);
		respond = 1;
		break;
	case TLV_TIME_STATUS_NP:
		tsn = (struct time_status_np *) tlv->data;
		tsn->master_offset = c->master_offset;
		tsn->ingress_time = tmv_to_nanoseconds(c->t2);
		tsn->cumulativeScaledRateOffset =
			(Integer32) (c->status.cumulativeScaledRateOffset +
				      c->nrr * POW2_41 - POW2_41);
		tsn->scaledLastGmPhaseChange = c->status.scaledLastGmPhaseChange;
		tsn->gmTimeBaseIndicator = c->status.gmTimeBaseIndicator;
		tsn->lastGmPhaseChange = c->status.lastGmPhaseChange;
		if (cid_eq(&c->dad.pds.grandmasterIdentity, &c->dds.clockIdentity))
			tsn->gmPresent = 0;
		else
			tsn->gmPresent = 1;
		tsn->gmIdentity = c->dad.pds.grandmasterIdentity;
		datalen = sizeof(*tsn);
		respond = 1;
		break;
	case TLV_GRANDMASTER_SETTINGS_NP:
		gsn = (struct grandmaster_settings_np *) tlv->data;
		gsn->clockQuality = c->dds.clockQuality;
		gsn->utc_offset = c->utc_offset;
		gsn->time_flags = c->time_flags;
		gsn->time_source = c->time_source;
		datalen = sizeof(*gsn);
		respond = 1;
		break;
	case TLV_SUBSCRIBE_EVENTS_NP:
		if (p != c->uds_port) {
			/* Only the UDS port allowed. */
			break;
		}
		sen = (struct subscribe_events_np *)tlv->data;
		clock_get_subscription(c, req, sen->bitmask, &sen->duration);
		respond = 1;
		break;
	}
	if (respond) {
		if (datalen % 2) {
			tlv->data[datalen] = 0;
			datalen++;
		}
		tlv->length = sizeof(tlv->id) + datalen;
		rsp->header.messageLength += sizeof(*tlv) + datalen;
		rsp->tlv_count = 1;
	}
	return respond;
}

static int clock_management_get_response(struct clock *c, struct port *p,
					 int id, struct ptp_message *req)
{
	struct PortIdentity pid = port_identity(p);
	struct ptp_message *rsp;
	int respond;

	rsp = port_management_reply(pid, p, req);
	if (!rsp) {
		return 0;
	}
	respond = clock_management_fill_response(c, p, req, rsp, id);
	if (respond)
		port_prepare_and_send(p, rsp, 0);
	msg_put(rsp);
	return respond;
}

static int clock_management_set(struct clock *c, struct port *p,
				int id, struct ptp_message *req, int *changed)
{
	int respond = 0;
	struct management_tlv *tlv;
	struct grandmaster_settings_np *gsn;
	struct subscribe_events_np *sen;

	tlv = (struct management_tlv *) req->management.suffix;

	switch (id) {
	case TLV_GRANDMASTER_SETTINGS_NP:
		gsn = (struct grandmaster_settings_np *) tlv->data;
		c->dds.clockQuality = gsn->clockQuality;
		c->utc_offset = gsn->utc_offset;
		c->time_flags = gsn->time_flags;
		c->time_source = gsn->time_source;
		*changed = 1;
		respond = 1;
		break;
	case TLV_SUBSCRIBE_EVENTS_NP:
		sen = (struct subscribe_events_np *)tlv->data;
		clock_update_subscription(c, req, sen->bitmask,
					  sen->duration);
		respond = 1;
		break;
	}
	if (respond && !clock_management_get_response(c, p, id, req))
		pr_err("failed to send management set response");
	return respond ? 1 : 0;
}

static void clock_stats_update(struct clock_stats *s,
			       int64_t offset, double freq)
{
	struct stats_result offset_stats, freq_stats, delay_stats;

	stats_add_value(s->offset, offset);
	stats_add_value(s->freq, freq);

	if (stats_get_num_values(s->offset) < s->max_count)
		return;

	stats_get_result(s->offset, &offset_stats);
	stats_get_result(s->freq, &freq_stats);

	/* Path delay stats are updated separately, they may be empty. */
	if (!stats_get_result(s->delay, &delay_stats)) {
		pr_info("rms %4.0f max %4.0f "
			"freq %+6.0f +/- %3.0f "
			"delay %5.0f +/- %3.0f",
			offset_stats.rms, offset_stats.max_abs,
			freq_stats.mean, freq_stats.stddev,
			delay_stats.mean, delay_stats.stddev);
	} else {
		pr_info("rms %4.0f max %4.0f "
			"freq %+6.0f +/- %3.0f",
			offset_stats.rms, offset_stats.max_abs,
			freq_stats.mean, freq_stats.stddev);
	}

	stats_reset(s->offset);
	stats_reset(s->freq);
	stats_reset(s->delay);
}

static void clock_print_to_stats_file(struct clock *c)
{
	int64_t best_id;
	int64_t master_id;
	int64_t my_id;
	struct timespec now;
	int64_t now_nanoseconds;
	struct port *p;
	char port_state_output[1024] = { '\0' };
	size_t port_state_bytes_left = sizeof(port_state_output);

	clock_gettime(CLOCK_MONOTONIC, &now);
	now_nanoseconds = tmv_to_nanoseconds(timespec_to_tmv(now));

	/*
	 * Update at 2Hz max.
	 */
	if ((now_nanoseconds - c->last_stats_file_update) < (NS_PER_SEC / 2))
		return;

	c->last_stats_file_update = now_nanoseconds;

	/* Clock identifiers are stored in network byte order. */
	memcpy(&best_id, c->best_id.id, sizeof(best_id));
	best_id = net2host64(best_id);
	memcpy(&master_id, c->dad.pds.parentPortIdentity.clockIdentity.id,
	       sizeof(master_id));
	master_id = net2host64(master_id);
	memcpy(&my_id, c->dds.clockIdentity.id, sizeof(my_id));
	my_id = net2host64(my_id);

	LIST_FOREACH(p, &c->ports, list) {
		int bytes_consumed = snprintf(
			&port_state_output[sizeof(port_state_output) -
					   port_state_bytes_left],
			port_state_bytes_left,
			"port%d_state: %d\n",
			port_number(p), port_state(p));
		if (bytes_consumed < 0)
			break;
		else if (bytes_consumed > port_state_bytes_left)
			bytes_consumed = port_state_bytes_left;
		port_state_bytes_left -= bytes_consumed;
	}
	port_state_output[sizeof(port_state_output) - 1] = '\0';

	if (stats_file_print(c->stats_filename,
			     "monotonic_timestamp: %" PRId64 "\n"
			     "master_offset: %" PRId64 "\n"
			     "servo_state: %d\n"
			     "freq: %f\n"
			     "path_delay: %" PRId64 "\n"
			     "best_id: 0x%016" PRIx64 "\n"
			     "steps_removed: %" PRId16 "\n"
			     "master_id: 0x%016" PRIx64 "\n"
			     "master_port_number: %" PRId16 "\n"
			     "my_id: 0x%016" PRIx64 "\n"
			     "nrr: %f\n"
			     "c1: %" PRId64 "\n"
			     "c2: %" PRId64 "\n"
			     "t1: %" PRId64 "\n"
			     "t2: %" PRId64 "\n"
			     "free_running: %d\n"
			     "freq_est_interval: %d\n"
			     "grand_master_capable: %d\n"
			     "utc_timescale: %d\n"
			     "utc_offset_set: %d\n"
			     "leap_set: %d\n"
			     "kernel_leap: %d\n"
			     "utc_offset: %d\n"
			     "time_flags: 0x%08x\n"
			     "time_source: 0x%02x\n"
			     "gm_clock_class: %u\n"
			     "gm_clock_accuracy: %" PRId64 "\n"
			     "my_clock_class: %u\n"
			     "my_clock_accuracy: %" PRId64 "\n"
			     "gm_priority1: %u\n"
			     "gm_priority2: %u\n"
			     "my_priority1: %u\n"
			     "my_priority2: %u\n"
			     "%s"
			     "master_offset_filtered: %" PRId64 "\n"
			     "path_delay_raw: %" PRId64 "\n",
			     now_nanoseconds,
			     tmv_to_nanoseconds(c->master_offset_raw),
			     c->servo_state,
			     clockadj_get_freq(c->clkid),
			     tmv_to_nanoseconds(c->path_delay),
			     best_id,
			     c->cur.stepsRemoved,
			     master_id,
			     c->dad.pds.parentPortIdentity.portNumber,
			     my_id,
			     c->nrr,
			     tmv_to_nanoseconds(c->c1),
			     tmv_to_nanoseconds(c->c2),
			     tmv_to_nanoseconds(c->t1),
			     tmv_to_nanoseconds(c->t2),
			     c->free_running,
			     c->freq_est_interval,
			     c->grand_master_capable,
			     c->utc_timescale,
			     c->utc_offset_set,
			     c->leap_set,
			     c->kernel_leap,
			     clock_current_utc_offset(c),
			     c->time_flags,
			     c->time_source,
			     c->dad.pds.grandmasterClockQuality.clockClass,
			     CLOCK_ACCURACIES[c->dad.pds.grandmasterClockQuality.
				clockAccuracy],
			     c->dds.clockQuality.clockClass,
			     CLOCK_ACCURACIES[c->dds.clockQuality.clockAccuracy],
			     c->dad.pds.grandmasterPriority1,
			     c->dad.pds.grandmasterPriority2,
			     c->dds.priority1,
			     c->dds.priority2,
			     port_state_output,
			     tmv_to_nanoseconds(c->master_offset),
			     tmv_to_nanoseconds(c->path_delay_raw)))
	{
		/*
		 * Clear 'stats_filename' on error so that we do not spam.
		 */
		c->stats_filename = NULL;
	}
}

/*
 * Update the clock's grandmaster settings based on libgpstime. This sets the
 * UTC offset and leap second flags based on a leap second table.
 *
 * @return 1 if the grandmaster settings were changed. This means a state
 *         determination event (sde) is required to copy the grandmaster
 *         settings into the appropriate datasets.
 */
static int clock_update_libgpstime_leap_second(struct clock *c)
{
	struct timespec now;
	int64_t now_ns;
	int64_t time_since_update;
	gpstime_error_code_t gpstime_error;
	gpstime_nano_t current_gps_time;
	int current_gps_leaps;
	int future_gps_leaps;
	int new_utc_offset;
	int new_time_flags;

	/*
	 * Skip update if we're not configured to use a leap second table.
	 */
	if (!c->leap_table_filename)
	{
		return 0;
	}

	/*
	 * Re-read the table if we don't have one, or if it is over an hour
	 * old. This allows us to update the leap second table without having to
	 * restart ptp4l.
	 */
	clock_gettime(CLOCK_MONOTONIC, &now);
	now_ns = tmv_to_nanoseconds(timespec_to_tmv(now));
	time_since_update = now_ns - c->last_leap_table_update;
	if (!c->leap_table || time_since_update > NS_PER_HOUR)
	{
		struct gpstime_leap_table_t *new_leap_table;

		c->last_leap_table_update = now_ns;
		new_leap_table = gpstime_leap_table_parse_any(
			c->leap_table_filename,
			&gpstime_error);
		if (new_leap_table)
		{
			pr_debug("libgpstime: read leap table");
			gpstime_leap_table_free(c->leap_table);
			c->leap_table = new_leap_table;
		}
		else
		{
			pr_err("libgpstime: failed to read leap table '%s' error %d",
			       c->leap_table_filename,
			       (int)gpstime_error);
		}
	}

	/*
	 * Don't proceed without a leap table.
	 */
	if (!c->leap_table)
	{
		return 0;
	}

	/*
	 * Get the UTC offset.
	 */
	current_gps_time = gpstime_get_gps_time(c->leap_table);
	current_gps_leaps = gpstime_get_leaps_since_gps_epoch(c->leap_table,
							      current_gps_time);
	new_utc_offset = gpstime_leaps_between_ptp_and_gps + current_gps_leaps;

	/*
	 * Get the time flags. Always assert UTC_OFF_VALID, which indicates that
	 * we know the current UTC offset. Always assert TIME_TRACEABLE and
	 * FREQ_TRACEABLE, which indicates that our time is traceable to an
	 * official source of time. We always use the leap second table in
	 * conjunction with NTP, but ideally this would be configurable.
	 */
	new_time_flags = c->time_flags;
	new_time_flags |= UTC_OFF_VALID | TIME_TRACEABLE | FREQ_TRACEABLE;

	/*
	 * To determine if a leap second will occur, look half a day into the
	 * future and see if there is a different UTC offset.
	 *
	 * IEEE1588-2008 section 9.4 states that these should be true no earlier
	 * than 12 hours before midnight, and that they should be cleared within
	 * +/- 2 announcement messages of midnight.
	 */
	future_gps_leaps = gpstime_get_leaps_since_gps_epoch(
		c->leap_table,
		current_gps_time + (NS_PER_DAY / 2));
	new_time_flags &= ~(LEAP_59 | LEAP_61);
	if (future_gps_leaps > current_gps_leaps)
	{
		new_time_flags |= LEAP_61;
	}
	else if (future_gps_leaps < current_gps_leaps)
	{
		new_time_flags |= LEAP_59;
	}

	/*
	 * If nothing has changed, then stop here.
	 */
	if (c->utc_offset == new_utc_offset && c->time_flags == new_time_flags)
	{
	    return 0;
	}

	/*
	 * Otherwise, update the grandmaster settings, and return 1 to trigger a
	 * state determination event. This will call
	 * handle_state_decision_for_port() for each port, which will invoke
	 * clock_update_grandmaster() on all ports every time we are currently
	 * the grandmaster. This function is responsible for copying these
	 * parameters into the timedataset.
	 */
	c->utc_offset = new_utc_offset;
	c->time_flags = new_time_flags;

	pr_notice("libgpstime: updated grandmaster settings: utc_offset %d time_flags 0x%x",
		  c->utc_offset,
		  c->time_flags);

	return 1;
}

static enum servo_state clock_no_adjust(struct clock *c)
{
	double fui;
	double ratio, freq;
	tmv_t origin2;
	struct freq_estimator *f = &c->fest;
	enum servo_state state = SERVO_UNLOCKED;
	/*
	 * We have clock.t1 as the origin time stamp, and clock.t2 as
	 * the ingress. According to the master's clock, the time at
	 * which the sync arrived is:
	 *
	 *    origin = origin_ts + path_delay + correction
	 *
	 * The ratio of the local clock freqency to the master clock
	 * is estimated by:
	 *
	 *    (ingress_2 - ingress_1) / (origin_2 - origin_1)
	 *
	 * Both of the origin time estimates include the path delay,
	 * but we assume that the path delay is in fact constant.
	 * By leaving out the path delay altogther, we can avoid the
	 * error caused by our imperfect path delay measurement.
	 */
	if (!f->ingress1) {
		f->ingress1 = c->t2;
		f->origin1 = tmv_add(c->t1, tmv_add(c->c1, c->c2));
		return state;
	}
	f->count++;
	if (f->count < f->max_count) {
		return state;
	}
	if (tmv_eq(c->t2, f->ingress1)) {
		pr_warning("bad timestamps in rate ratio calculation");
		return state;
	}
	/*
	 * origin2 = c->t1 (+c->path_delay) + c->c1 + c->c2;
	 */
	origin2 = tmv_add(c->t1, tmv_add(c->c1, c->c2));

	ratio = tmv_dbl(tmv_sub(origin2, f->origin1)) /
		tmv_dbl(tmv_sub(c->t2, f->ingress1));
	freq = (1.0 - ratio) * 1e9;

	if (c->stats.max_count > 1) {
		clock_stats_update(&c->stats,
				   tmv_to_nanoseconds(c->master_offset), freq);
	} else {
		pr_info("master offset %10" PRId64 " s%d freq %+7.0f "
			"path delay %9" PRId64,
			tmv_to_nanoseconds(c->master_offset), state, freq,
			tmv_to_nanoseconds(c->path_delay));
	}

	fui = 1.0 + (c->status.cumulativeScaledRateOffset + 0.0) / POW2_41;

	pr_debug("peer/local    %.9f", c->nrr);
	pr_debug("fup_info      %.9f", fui);
	pr_debug("product       %.9f", fui * c->nrr);
	pr_debug("sum-1         %.9f", fui + c->nrr - 1.0);
	pr_debug("master/local  %.9f", ratio);
	pr_debug("diff         %+.9f", ratio - (fui + c->nrr - 1.0));

	f->ingress1 = c->t2;
	f->origin1 = origin2;
	f->count = 0;

	return state;
}

static void clock_update_grandmaster(struct clock *c)
{
	struct parentDS *pds = &c->dad.pds;
	memset(&c->cur, 0, sizeof(c->cur));
	memset(c->ptl, 0, sizeof(c->ptl));
	pds->parentPortIdentity.clockIdentity   = c->dds.clockIdentity;
	pds->parentPortIdentity.portNumber      = 0;
	pds->grandmasterIdentity                = c->dds.clockIdentity;
	pds->grandmasterClockQuality            = c->dds.clockQuality;
	pds->grandmasterPriority1               = c->dds.priority1;
	pds->grandmasterPriority2               = c->dds.priority2;
	c->dad.path_length                      = 0;
	c->tds.currentUtcOffset                 = c->utc_offset;
	c->tds.flags                            = c->time_flags;
	c->tds.timeSource                       = c->time_source;
}

static void clock_update_slave(struct clock *c)
{
	struct parentDS *pds = &c->dad.pds;
	struct ptp_message *msg        = TAILQ_FIRST(&c->best->messages);
	c->cur.stepsRemoved            = 1 + c->best->dataset.stepsRemoved;
	pds->parentPortIdentity        = c->best->dataset.sender;
	pds->grandmasterIdentity       = msg->announce.grandmasterIdentity;
	pds->grandmasterClockQuality   = msg->announce.grandmasterClockQuality;
	pds->grandmasterPriority1      = msg->announce.grandmasterPriority1;
	pds->grandmasterPriority2      = msg->announce.grandmasterPriority2;
	c->tds.currentUtcOffset        = msg->announce.currentUtcOffset;
	c->tds.flags                   = msg->header.flagField[1];
	c->tds.timeSource              = msg->announce.timeSource;
	if (!(c->tds.flags & PTP_TIMESCALE)) {
		pr_warning("foreign master not using PTP timescale");
	}
	if (c->tds.currentUtcOffset < CURRENT_UTC_OFFSET) {
		pr_warning("running in a temporal vortex");
	}
}

int clock_current_utc_offset(struct clock *c)
{
	int utc_offset;

	if (c->tds.flags & UTC_OFF_VALID && c->tds.flags & TIME_TRACEABLE) {
		utc_offset = c->tds.currentUtcOffset;
	} else if (c->tds.currentUtcOffset > CURRENT_UTC_OFFSET) {
		utc_offset = c->tds.currentUtcOffset;
	} else {
		utc_offset = CURRENT_UTC_OFFSET;
	}

	return utc_offset;
}

static int clock_utc_correct(struct clock *c, tmv_t ingress)
{
	struct timespec offset;
	int utc_offset, leap, clock_leap;
	uint64_t ts;

	if (!c->utc_timescale)
		return 0;

	utc_offset = clock_current_utc_offset(c);

	if (c->tds.flags & LEAP_61) {
		leap = 1;
	} else if (c->tds.flags & LEAP_59) {
		leap = -1;
	} else {
		leap = 0;
	}

	/* Handle leap seconds. */
	if ((leap || c->leap_set) && c->clkid == CLOCK_REALTIME) {
		/* If the clock will be stepped, the time stamp has to be the
		   target time. Ignore possible 1 second error in utc_offset. */
		if (c->servo_state == SERVO_UNLOCKED) {
			ts = tmv_to_nanoseconds(tmv_sub(ingress,
							c->master_offset));
			if (c->tds.flags & PTP_TIMESCALE)
				ts -= utc_offset * NS_PER_SEC;
		} else {
			ts = tmv_to_nanoseconds(ingress);
		}

		/* Suspend clock updates in the last second before midnight. */
		if (is_utc_ambiguous(ts)) {
			pr_info("clock update suspended due to leap second");
			return -1;
		}

		clock_leap = leap_second_status(ts, c->leap_set,
						&leap, &utc_offset);
		if (c->leap_set != clock_leap) {
			if (c->kernel_leap)
				sysclk_set_leap(clock_leap);
			else
				servo_leap(c->servo, clock_leap);
			c->leap_set = clock_leap;
		}
	}

	/* Update TAI-UTC offset of the system clock if valid and traceable. */
	if (c->tds.flags & UTC_OFF_VALID && c->tds.flags & TIME_TRACEABLE &&
	    c->utc_offset_set != utc_offset && c->clkid == CLOCK_REALTIME) {
		sysclk_set_tai_offset(utc_offset);
		c->utc_offset_set = utc_offset;
	}

	if (!(c->tds.flags & PTP_TIMESCALE))
		return 0;

	offset.tv_sec = utc_offset;
	offset.tv_nsec = 0;
	/* Local clock is UTC, but master is TAI. */
	c->master_offset = tmv_add(c->master_offset, timespec_to_tmv(offset));
	return 0;
}

static int forwarding(struct clock *c, struct port *p)
{
	enum port_state ps = port_state(p);
	switch (ps) {
	case PS_MASTER:
	case PS_GRAND_MASTER:
	case PS_SLAVE:
	case PS_UNCALIBRATED:
	case PS_PRE_MASTER:
		return 1;
	default:
		break;
	}
	if (p == c->uds_port && ps != PS_FAULTY) {
		return 1;
	}
	return 0;
}

/* public methods */

UInteger8 clock_class(struct clock *c)
{
	return c->dds.clockQuality.clockClass;
}

static int clock_add_port(struct clock *c, int phc_index,
			  enum timestamp_type timestamping,
			  struct interface *iface)
{
	struct port *p, *piter, *lastp = NULL;

	if (clock_resize_pollfd(c, c->nports + 1))
		return -1;

	/* This differs from the implementation in master,
	 * last_port_number is now post-fix incremented to correctly
	 * allow the registration of port 0.
	 */
	p = port_open(phc_index, timestamping, c->last_port_number++,
		      iface, c);
	if (!p) {
		/* No need to shrink pollfd */
		return -1;
	}

	/* If this is the UDS port, initialize the clock structure here instead. */
	if (iface->ts_info.phc_index == PHC_INDEX_UDS)
		c->uds_port = p;

	LIST_FOREACH(piter, &c->ports, list)
		lastp = piter;
	if (lastp)
		LIST_INSERT_AFTER(lastp, p, list);
	else
		LIST_INSERT_HEAD(&c->ports, p, list);
	c->nports++;
	clock_fda_changed(c);
	return 0;
}

static void clock_remove_port(struct clock *c, struct port *p)
{
	/* Do not call clock_resize_pollfd, it's pointless to shrink
	 * the allocated memory at this point, clock_destroy will free
	 * it all anyway. This function is usable from other parts of
	 * the code, but even then we don't mind if pollfd is larger
	 * than necessary. */
	LIST_REMOVE(p, list);
	c->nports--;
	clock_fda_changed(c);
	port_close(p);
}

struct clock *clock_create(int phc_index, struct interfaces_head *ifaces,
			   enum timestamp_type timestamping, struct default_ds *dds,
			   enum servo_type servo, const char *stats_filename,
			   const char *leap_table_filename, const char *uds_clock_filename)
{
	int fadj = 0, max_adj = 0, sw_ts = timestamping_is_software(timestamping) ? 1 : 0;
	struct clock *c = &the_clock;
	struct port *p;
	char phc[32];
	struct interface *udsif = &c->uds_interface;
	struct interface *iface;
	struct timespec ts;

	clock_gettime(CLOCK_REALTIME, &ts);
	srandom(ts.tv_sec ^ ts.tv_nsec);

	if (c->nports)
		clock_destroy(c);

	snprintf(udsif->name, sizeof(udsif->name), "%s", uds_path);
	udsif->transport = TRANS_UDS;
	udsif->delay_filter_length = 1;

	c->free_running = dds->free_running;
	c->freq_est_interval = dds->freq_est_interval;
	c->grand_master_capable = dds->grand_master_capable;
	c->kernel_leap = dds->kernel_leap;

	/*
	 * Save the UDS clock device name, if any.
	 */
	c->uds_clock_filename = uds_clock_filename;

	/*
	 * If we have libgpstime, we will overwrite utc_offset in
	 * clock_update_libgpstime_leap_second.
	 */
	c->utc_offset = CURRENT_UTC_OFFSET;
	c->time_source = dds->time_source;
	c->desc = dds->clock_desc;

	if (c->free_running) {
		c->clkid = CLOCK_INVALID;
		if (timestamping == TS_SOFTWARE || timestamping == TS_LEGACY_HW) {
			c->utc_timescale = 1;
		}
	} else if (phc_index >= 0) {
		snprintf(phc, 31, "/dev/ptp%d", phc_index);
		c->clkid = phc_open(phc);
		if (c->clkid == CLOCK_INVALID) {
			pr_err("Failed to open %s: %m", phc);
			return NULL;
		}
		max_adj = phc_max_adj(c->clkid);
		if (!max_adj) {
			pr_err("clock is not adjustable");
			return NULL;
		}
		clockadj_init(c->clkid);
	} else {
		c->clkid = CLOCK_REALTIME;
		c->utc_timescale = 1;
		clockadj_init(c->clkid);
		max_adj = sysclk_max_freq();
		sysclk_set_leap(0);
	}
	c->utc_offset_set = 0;
	c->leap_set = 0;
	/* Always use PTP timescale */
	c->time_flags = PTP_TIMESCALE;

	if (c->clkid != CLOCK_INVALID) {
		fadj = (int) clockadj_get_freq(c->clkid);
		/* Due to a bug in older kernels, the reading may silently fail
		   and return 0. Set the frequency back to make sure fadj is
		   the actual frequency of the clock. */
		clockadj_set_freq(c->clkid, fadj);
	}
	c->servo = servo_create(servo, -fadj, max_adj, sw_ts);
	if (!c->servo) {
		pr_err("Failed to create clock servo");
		return NULL;
	}
	c->servo_state = SERVO_UNLOCKED;
	c->servo_type = servo;
	c->delay_filter = filter_create(dds->delay_filter,
					dds->delay_filter_length);
	if (!c->delay_filter) {
		pr_err("Failed to create delay filter");
		return NULL;
	}
	c->offset_filter = filter_create(configured_offset_filter,
					 configured_offset_filter_length);
	if (!c->offset_filter) {
		pr_err("Failed to create offset filter");
		return NULL;
	}
	c->nrr = 1.0;
	c->stats_interval = dds->stats_interval;
	c->stats.offset = stats_create();
	c->stats.freq = stats_create();
	c->stats.delay = stats_create();
	if (!c->stats.offset || !c->stats.freq || !c->stats.delay) {
		pr_err("failed to create stats");
		return NULL;
	}
	c->stats_filename = stats_filename;

	c->leap_table_filename = leap_table_filename;
	c->last_leap_table_update = 0;
	c->leap_table = NULL;
	if (c->leap_table_filename)
	{
		/*
		 * If we're configured to use a leap table, perform a leap
		 * second update now to attempt to read in the table. If we
		 * don't have a table, error out so we can discover leap table
		 * problems at initialization.
		 */
		clock_update_libgpstime_leap_second(c);
		if (!c->leap_table) {
			pr_err("Failed to initialize leap second table");
			return NULL;
		}
	}

	if (dds->sanity_freq_limit) {
		c->sanity_check = clockcheck_create(dds->sanity_freq_limit);
		if (!c->sanity_check) {
			pr_err("Failed to create clock sanity check");
			return NULL;
		}
	}

	c->dds = dds->dds;

	/* Initialize the parentDS. */
	clock_update_grandmaster(c);
	c->dad.pds.parentStats                           = 0;
	c->dad.pds.observedParentOffsetScaledLogVariance = 0xffff;
	c->dad.pds.observedParentClockPhaseChangeRate    = 0x7fffffff;
	c->dad.ptl = c->ptl;

	clock_sync_interval(c, 0);

	LIST_INIT(&c->subscribers);
	LIST_INIT(&c->ports);
	c->last_port_number = 0;

	/*
	 * Create the UDS interface.
	 */
	if (c->uds_clock_filename) {
		int rc;

		/* Create a "valid" ts_info structure, with an special phc_index. */
		udsif->ts_info.valid = 1;
		udsif->ts_info.phc_index = PHC_INDEX_UDS;
		udsif->boundary_clock_jbod = dds->boundary_clock_jbod;

		/* And register it normally as a clock port. */
		rc = clock_add_port(c, phc_index, timestamping, udsif);
		if (rc != 0) {
			pr_err("failed to add UDS port with explicit PHC: %d",
			       rc);
			return NULL;
		}
	} else {
		if (clock_resize_pollfd(c, 0)) {
			pr_err("failed to allocate pollfd");
			return NULL;
		}
		c->uds_port = port_open(phc_index, timestamping, 0, udsif, c);
		if (!c->uds_port) {
			pr_err("failed to open the UDS port");
			return NULL;
		}
		clock_fda_changed(c);

		/* Increment the port number, since clock_add_port now uses post-fix
		 * incrementing.
		 */
		c->last_port_number++;
	}

	/* Create the ports. */
	STAILQ_FOREACH(iface, ifaces, list) {
		if (clock_add_port(c, phc_index, timestamping, iface)) {
			pr_err("failed to open port %s", iface->name);
			return NULL;
		}
	}

	c->dds.numberPorts = c->nports;

	LIST_FOREACH(p, &c->ports, list) {
		port_dispatch(p, EV_INITIALIZE, 0);
	}

	/* If there's no UDS clock, explicitly dispatch UDS. */
	if (!c->uds_clock_filename)
		port_dispatch(c->uds_port, EV_INITIALIZE, 0);

	return c;
}

struct dataset *clock_best_foreign(struct clock *c)
{
	return c->best ? &c->best->dataset : NULL;
}

struct port *clock_best_port(struct clock *c)
{
	return c->best ? c->best->port : NULL;
}

struct dataset *clock_default_ds(struct clock *c)
{
	struct dataset *out = &c->default_dataset;
	struct defaultDS *in = &c->dds;

	out->priority1              = in->priority1;
	out->identity               = in->clockIdentity;
	out->quality                = in->clockQuality;
	out->priority2              = in->priority2;
	out->stepsRemoved           = 0;
	out->sender.clockIdentity   = in->clockIdentity;
	out->sender.portNumber      = 0;
	out->receiver.clockIdentity = in->clockIdentity;
	out->receiver.portNumber    = 0;

	return out;
}

UInteger8 clock_domain_number(struct clock *c)
{
	return c->dds.domainNumber;
}

void clock_follow_up_info(struct clock *c, struct follow_up_info_tlv *f)
{
	c->status.cumulativeScaledRateOffset = f->cumulativeScaledRateOffset;
	c->status.scaledLastGmPhaseChange = f->scaledLastGmPhaseChange;
	c->status.gmTimeBaseIndicator = f->gmTimeBaseIndicator;
	memcpy(&c->status.lastGmPhaseChange, &f->lastGmPhaseChange,
	       sizeof(c->status.lastGmPhaseChange));
}

int clock_gm_capable(struct clock *c)
{
	return c->grand_master_capable;
}

struct ClockIdentity clock_identity(struct clock *c)
{
	return c->dds.clockIdentity;
}

static int clock_resize_pollfd(struct clock *c, int new_nports)
{
	struct pollfd *new_pollfd;

	/* Need to allocate one extra block of fds for uds if there is not a
	 * clock associated with the port */
	if (c->uds_clock_filename)
		new_pollfd = realloc(c->pollfd, new_nports * N_CLOCK_PFD *
						    sizeof(struct pollfd));
	else
		new_pollfd = realloc(c->pollfd, (new_nports + 1) * N_CLOCK_PFD *
						    sizeof(struct pollfd));

	if (!new_pollfd)
		return -1;
	c->pollfd = new_pollfd;
	return 0;
}

static void clock_fill_pollfd(struct pollfd *dest, struct port *p)
{
	struct fdarray *fda;
	int i;

	fda = port_fda(p);
	for (i = 0; i < N_POLLFD; i++) {
		dest[i].fd = fda->fd[i];
		dest[i].events = POLLIN|POLLPRI;
	}
	dest[i].fd = port_fault_fd(p);
	dest[i].events = POLLIN|POLLPRI;
}

static void clock_check_pollfd(struct clock *c)
{
	struct port *p;
	struct pollfd *dest = c->pollfd;

	if (c->pollfd_valid)
		return;
	LIST_FOREACH(p, &c->ports, list) {
		clock_fill_pollfd(dest, p);
		dest += N_CLOCK_PFD;
	}

	/* If there's no UDS clock, explicitly check UDS. */
	if (!c->uds_clock_filename)
		clock_fill_pollfd(dest, c->uds_port);

	c->pollfd_valid = 1;
}

void clock_fda_changed(struct clock *c)
{
	c->pollfd_valid = 0;
}

static int clock_do_forward_mgmt(struct clock *c,
				 struct port *in, struct port *out,
				 struct ptp_message *msg, int *pre_sent)
{
	if (in == out || !forwarding(c, out))
		return 0;
	if (!*pre_sent) {
		/* delay calling msg_pre_send until
		 * actually forwarding */
		msg_pre_send(msg);
		*pre_sent = 1;
	}
	return port_forward(out, msg);
}

static void clock_forward_mgmt_msg(struct clock *c, struct port *p, struct ptp_message *msg)
{
	struct port *piter;
	int pdulen = 0, msg_ready = 0;

	if (forwarding(c, p) && msg->management.boundaryHops) {
		pdulen = msg->header.messageLength;
		msg->management.boundaryHops--;
		LIST_FOREACH(piter, &c->ports, list) {
			if (clock_do_forward_mgmt(c, p, piter, msg, &msg_ready))
				pr_err("port %d: management forward failed",
				       port_number(piter));
		}

		/* If there's no UDS clock, explicitly handle UDS. */
		if (!c->uds_clock_filename) {
			if (clock_do_forward_mgmt(c, p, c->uds_port, msg, &msg_ready))
				pr_err("uds port: management forward failed");
		}

		if (msg_ready) {
			msg_post_recv(msg, pdulen);
			msg->management.boundaryHops++;
		}
	}
}

int clock_manage(struct clock *c, struct port *p, struct ptp_message *msg)
{
	int changed = 0, res, answers;
	struct port *piter;
	struct management_tlv *mgt;
	struct ClockIdentity *tcid, wildcard = {
		{0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff}
	};

	/* Forward this message out all eligible ports. */
	clock_forward_mgmt_msg(c, p, msg);

	/* Apply this message to the local clock and ports. */
	tcid = &msg->management.targetPortIdentity.clockIdentity;
	if (!cid_eq(tcid, &wildcard) && !cid_eq(tcid, &c->dds.clockIdentity)) {
		return changed;
	}
	if (msg->tlv_count != 1) {
		return changed;
	}
	mgt = (struct management_tlv *) msg->management.suffix;

	/*
	  The correct length according to the management ID is checked
	  in tlv.c, but management TLVs with empty bodies are also
	  received successfully to support GETs and CMDs. At this
	  point the TLV either has the correct length or length 2.
	*/
	switch (management_action(msg)) {
	case GET:
		if (clock_management_get_response(c, p, mgt->id, msg))
			return changed;
		break;
	case SET:
		if (mgt->length == 2 && mgt->id != TLV_NULL_MANAGEMENT) {
			clock_management_send_error(p, msg, TLV_WRONG_LENGTH);
			return changed;
		}
		if (p != c->uds_port) {
			/* Sorry, only allowed on the UDS port. */
			clock_management_send_error(p, msg, TLV_NOT_SUPPORTED);
			return changed;
		}
		if (clock_management_set(c, p, mgt->id, msg, &changed))
			return changed;
		break;
	case COMMAND:
		break;
	default:
		return changed;
	}

	switch (mgt->id) {
	case TLV_PORT_PROPERTIES_NP:
		if (p != c->uds_port) {
			/* Only the UDS port allowed. */
			clock_management_send_error(p, msg, TLV_NOT_SUPPORTED);
			return 0;
		}
	}

	switch (mgt->id) {
	case TLV_USER_DESCRIPTION:
	case TLV_SAVE_IN_NON_VOLATILE_STORAGE:
	case TLV_RESET_NON_VOLATILE_STORAGE:
	case TLV_INITIALIZE:
	case TLV_FAULT_LOG:
	case TLV_FAULT_LOG_RESET:
	case TLV_DEFAULT_DATA_SET:
	case TLV_CURRENT_DATA_SET:
	case TLV_PARENT_DATA_SET:
	case TLV_TIME_PROPERTIES_DATA_SET:
	case TLV_PRIORITY1:
	case TLV_PRIORITY2:
	case TLV_DOMAIN:
	case TLV_SLAVE_ONLY:
	case TLV_TIME:
	case TLV_CLOCK_ACCURACY:
	case TLV_UTC_PROPERTIES:
	case TLV_TRACEABILITY_PROPERTIES:
	case TLV_TIMESCALE_PROPERTIES:
	case TLV_PATH_TRACE_LIST:
	case TLV_PATH_TRACE_ENABLE:
	case TLV_GRANDMASTER_CLUSTER_TABLE:
	case TLV_ACCEPTABLE_MASTER_TABLE:
	case TLV_ACCEPTABLE_MASTER_MAX_TABLE_SIZE:
	case TLV_ALTERNATE_TIME_OFFSET_ENABLE:
	case TLV_ALTERNATE_TIME_OFFSET_NAME:
	case TLV_ALTERNATE_TIME_OFFSET_MAX_KEY:
	case TLV_ALTERNATE_TIME_OFFSET_PROPERTIES:
	case TLV_TRANSPARENT_CLOCK_DEFAULT_DATA_SET:
	case TLV_PRIMARY_DOMAIN:
	case TLV_TIME_STATUS_NP:
	case TLV_GRANDMASTER_SETTINGS_NP:
	case TLV_SUBSCRIBE_EVENTS_NP:
		clock_management_send_error(p, msg, TLV_NOT_SUPPORTED);
		break;
	default:
		answers = 0;
		LIST_FOREACH(piter, &c->ports, list) {
			res = port_manage(piter, p, msg);
			if (res < 0)
				return changed;
			if (res > 0)
				answers++;
		}
		if (!answers) {
			/* IEEE 1588 Interpretation #21 suggests to use
			 * TLV_WRONG_VALUE for ports that do not exist */
			clock_management_send_error(p, msg, TLV_WRONG_VALUE);
		}
		break;
	}
	return changed;
}

void clock_notify_event(struct clock *c, enum notification event)
{
	struct port *uds = c->uds_port;
	struct PortIdentity pid = port_identity(uds);
	struct ptp_message *msg;
	UInteger16 msg_len;
	int id;

	switch (event) {
	/* set id */
	default:
		return;
	}
	/* targetPortIdentity and sequenceId will be filled by
	 * clock_send_notification */
	msg = port_management_notify(pid, uds);
	if (!msg)
		return;
	if (!clock_management_fill_response(c, NULL, NULL, msg, id))
		goto err;
	msg_len = msg->header.messageLength;
	if (msg_pre_send(msg))
		goto err;
	clock_send_notification(c, msg, msg_len, event);
err:
	msg_put(msg);
}

struct parent_ds *clock_parent_ds(struct clock *c)
{
	return &c->dad;
}

struct PortIdentity clock_parent_identity(struct clock *c)
{
	return c->dad.pds.parentPortIdentity;
}

int clock_poll(struct clock *c)
{
	int cnt, err, i, sde = 0;
	enum fsm_event event;
	struct pollfd *cur;
	struct port *p;

	clock_check_pollfd(c);

	/* If we're using the UDS clock, don't wait on an extra port. */
	if (c->uds_clock_filename)
		cnt = poll(c->pollfd, (c->nports) * N_CLOCK_PFD, -1);
	else
		cnt = poll(c->pollfd, (c->nports + 1) * N_CLOCK_PFD, -1);

	if (cnt < 0) {
		if (EINTR == errno) {
			return 0;
		} else {
			pr_emerg("poll failed");
			return -1;
		}
	} else if (!cnt) {
		return 0;
	}

	cur = c->pollfd;
	LIST_FOREACH(p, &c->ports, list) {
		/* Let the ports handle their events. */
		for (i = err = 0; i < N_POLLFD && !err; i++) {
			if (cur[i].revents & (POLLIN|POLLPRI)) {
				event = port_event(p, i);
				if (EV_STATE_DECISION_EVENT == event)
					sde = 1;
				if (EV_ANNOUNCE_RECEIPT_TIMEOUT_EXPIRES == event)
					sde = 1;
				err = port_dispatch(p, event, 0);
				/* Clear any fault after a little while. */
				if (PS_FAULTY == port_state(p)) {
					clock_fault_timeout(p, 1);
					break;
				}
			}
		}

		/* Check the fault timer. */
		if (cur[N_POLLFD].revents & (POLLIN|POLLPRI)) {
			clock_fault_timeout(p, 0);
			port_dispatch(p, EV_FAULT_CLEARED, 0);
			/* Did that fail to clear the fault? Try again. */
			if (PS_FAULTY == port_state(p))
				clock_fault_timeout(p, 1);
		}

		cur += N_CLOCK_PFD;
	}

	/* If there's no UDS clock, explicitly handle UDS. */
	if (!c->uds_clock_filename) {
		/* Check the UDS port. */
		for (i = 0; i < N_POLLFD; i++) {
			if (cur[i].revents & (POLLIN|POLLPRI)) {
				event = port_event(c->uds_port, i);
				if (EV_STATE_DECISION_EVENT == event)
					sde = 1;
				/* If the UDS has a master, this can happen here
				 * too.
				 * In other words, if we do not receive a timely
				 * ANNOUNCE message, we want to be able to choose
				 * a new master clock. */
				if (EV_ANNOUNCE_RECEIPT_TIMEOUT_EXPIRES == event)
					sde = 1;
			}
		}
	}

	if (clock_update_libgpstime_leap_second(c))
		sde = 1;

	if (sde)
		handle_state_decision_event(c);

	clock_print_to_stats_file(c);

	clock_prune_subscriptions(c);
	return 0;
}

void clock_path_delay(struct clock *c, struct timespec req, struct timestamp rx,
		      Integer64 correction)
{
	tmv_t c1, c2, c3, pd, t1, t2, t3, t4;
	double rr;

	if (tmv_is_zero(c->t1))
		return;

	c1 = c->c1;
	c2 = c->c2;
	c3 = correction_to_tmv(correction);
	t1 = c->t1;
	t2 = c->t2;
	t3 = timespec_to_tmv(req);
	t4 = timestamp_to_tmv(rx);
	rr = clock_rate_ratio(c);

	/*
	 * c->path_delay = (t2 - t3) * rr + (t4 - t1);
	 * c->path_delay -= c_sync + c_fup + c_delay_resp;
	 * c->path_delay /= 2.0;
	 */

	pd = tmv_sub(t2, t3);
	if (rr != 1.0)
		pd = dbl_tmv(tmv_dbl(pd) * rr);
	pd = tmv_add(pd, tmv_sub(t4, t1));
	pd = tmv_sub(pd, tmv_add(c1, tmv_add(c2, c3)));
	pd = tmv_div(pd, 2);

	if (pd < 0) {
		pr_debug("negative path delay %10" PRId64, pd);
		pr_debug("path_delay = (t2 - t3) * rr + (t4 - t1) - (c1 + c2 + c3)");
		pr_debug("t2 - t3 = %+10" PRId64, t2 - t3);
		pr_debug("t4 - t1 = %+10" PRId64, t4 - t1);
		pr_debug("rr = %.9f", rr);
		pr_debug("c1 %10" PRId64, c1);
		pr_debug("c2 %10" PRId64, c2);
		pr_debug("c3 %10" PRId64, c3);
	}

	c->path_delay_set = 1;
	c->path_delay = filter_sample(c->delay_filter, pd);
	c->path_delay_raw = pd;

	c->cur.meanPathDelay = tmv_to_TimeInterval(c->path_delay);

	pr_debug("path delay    %10" PRId64 " %10" PRId64, c->path_delay, pd);

	if (c->stats.delay)
		stats_add_value(c->stats.delay, tmv_to_nanoseconds(pd));
}

void clock_peer_delay(struct clock *c, tmv_t ppd, double nrr)
{
	c->path_delay_set = 1;
	c->path_delay = ppd;
	c->nrr = nrr;

	if (c->stats.delay)
		stats_add_value(c->stats.delay, tmv_to_nanoseconds(ppd));
}

int clock_slave_only(struct clock *c)
{
	return c->dds.flags & DDS_SLAVE_ONLY;
}

UInteger16 clock_steps_removed(struct clock *c)
{
	return c->cur.stepsRemoved;
}

int clock_switch_phc(struct clock *c, int phc_index)
{
	struct servo *servo;
	int fadj, max_adj;
	clockid_t clkid;
	char phc[32];

	if (phc_index == PHC_INDEX_UDS && c->uds_clock_filename) {
		strncpy(phc, c->uds_clock_filename, sizeof(phc) - 1);
		phc[sizeof(phc) - 1] = '\0';
	} else {
		int ret = snprintf(phc, sizeof(phc), "/dev/ptp%d", phc_index);
		if (ret < 0) {
			pr_err("failed to format the ptp device: %m");
			return -1;

		} else if (ret >= sizeof(phc)) {
			pr_err("incompletely formatted ptp device");
			return -1;
		}
	}

	clkid = phc_open(phc);
	if (clkid == CLOCK_INVALID) {
		pr_err("Switching PHC, failed to open %s: %m", phc);
		return -1;
	}
	max_adj = phc_max_adj(clkid);
	if (!max_adj) {
		pr_err("Switching PHC, clock is not adjustable");
		phc_close(clkid);
		return -1;
	}
	fadj = (int) clockadj_get_freq(clkid);
	clockadj_set_freq(clkid, fadj);
	servo = servo_create(c->servo_type, -fadj, max_adj, 0);
	if (!servo) {
		pr_err("Switching PHC, failed to create clock servo");
		phc_close(clkid);
		return -1;
	}
	phc_close(c->clkid);
	servo_destroy(c->servo);
	c->clkid = clkid;
	c->servo = servo;
	c->servo_state = SERVO_UNLOCKED;
	return 0;
}

enum servo_state clock_synchronize(struct clock *c,
				   struct timespec ingress_ts,
				   struct timestamp origin_ts,
				   Integer64 correction1,
				   Integer64 correction2)
{
	double adj;
	tmv_t ingress, origin;
	enum servo_state state = SERVO_UNLOCKED;
	int clock_leap_second_ambiguous;

	ingress = timespec_to_tmv(ingress_ts);
	origin  = timestamp_to_tmv(origin_ts);

	c->t1 = origin;
	c->t2 = ingress;

	c->c1 = correction_to_tmv(correction1);
	c->c2 = correction_to_tmv(correction2);

	/*
	 * c->master_offset = ingress - origin - c->path_delay - c->c1 - c->c2;
	 */
	c->master_offset_raw = tmv_sub(ingress,
		tmv_add(origin, tmv_add(c->path_delay, tmv_add(c->c1, c->c2))));

	/*
	 * filter out network jitter
	 */
	c->master_offset = filter_sample(c->offset_filter,
					 c->master_offset_raw);

	clock_leap_second_ambiguous = clock_utc_correct(c, ingress);

	if (!c->path_delay_set)
		return state;

	/*
	 * "Coast" (keep current servo state, don't run servo) if we are
	 * crossing a leap second, and therefore the clock's time is ambiguous
	 * (only happens if 'c' is in UTC timescale).
	 */
	if (clock_leap_second_ambiguous)
		return c->servo_state;

	c->cur.offsetFromMaster = tmv_to_TimeInterval(c->master_offset);

	if (c->free_running)
		return clock_no_adjust(c);

	adj = servo_sample(c->servo, tmv_to_nanoseconds(c->master_offset),
			   tmv_to_nanoseconds(ingress), &state);
	c->servo_state = state;

	if (c->stats.max_count > 1) {
		clock_stats_update(&c->stats,
				   tmv_to_nanoseconds(c->master_offset), adj);
	} else {
		pr_info("master offset %10" PRId64 " s%d freq %+7.0f "
			"path delay %9" PRId64,
			tmv_to_nanoseconds(c->master_offset), state, adj,
			tmv_to_nanoseconds(c->path_delay));
	}

	switch (state) {
	case SERVO_UNLOCKED:
		break;
	case SERVO_JUMP:
		clockadj_set_freq(c->clkid, -adj);
		clockadj_step(c->clkid, -tmv_to_nanoseconds(c->master_offset));
		c->t1 = tmv_zero();
		c->t2 = tmv_zero();
		if (c->sanity_check) {
			clockcheck_set_freq(c->sanity_check, -adj);
			clockcheck_step(c->sanity_check,
					-tmv_to_nanoseconds(c->master_offset));
		}
		filter_reset(c->offset_filter);
		break;
	case SERVO_LOCKED:
		clockadj_set_freq(c->clkid, -adj);
		if (c->clkid == CLOCK_REALTIME)
			sysclk_set_sync();
		if (c->sanity_check)
			clockcheck_set_freq(c->sanity_check, -adj);
		break;
	}
	return state;
}

void clock_sync_interval(struct clock *c, int n)
{
	int shift;

	shift = c->freq_est_interval - n;
	if (shift < 0)
		shift = 0;
	else if (shift >= sizeof(int) * 8) {
		shift = sizeof(int) * 8 - 1;
		pr_warning("freq_est_interval is too long");
	}
	c->fest.max_count = (1 << shift);

	shift = c->stats_interval - n;
	if (shift < 0)
		shift = 0;
	else if (shift >= sizeof(int) * 8) {
		shift = sizeof(int) * 8 - 1;
		pr_warning("summary_interval is too long");
	}
	c->stats.max_count = (1 << shift);

	servo_sync_interval(c->servo, n < 0 ? 1.0 / (1 << -n) : 1 << n);
}

struct timePropertiesDS *clock_time_properties(struct clock *c)
{
	return &c->tds;
}

void clock_update_time_properties(struct clock *c, struct timePropertiesDS tds)
{
	c->tds = tds;
}

static void handle_state_decision_for_port(struct clock *c, struct port *p,
					   int fresh_best)
{
	enum fsm_event event;
	enum port_state ps = bmc_state_decision(c, p);

	switch (ps) {
	case PS_LISTENING:
		event = EV_NONE;
		break;
	case PS_GRAND_MASTER:
		pr_notice("assuming the grand master role");
		clock_update_grandmaster(c);
		event = EV_RS_GRAND_MASTER;
		break;
	case PS_MASTER:
		event = EV_RS_MASTER;
		break;
	case PS_PASSIVE:
		event = EV_RS_PASSIVE;
		break;
	case PS_SLAVE:
		clock_update_slave(c);
		event = EV_RS_SLAVE;
		break;
	default:
		event = EV_FAULT_DETECTED;
		break;
	}
	port_dispatch(p, event, fresh_best);
}

static void handle_state_decision_event(struct clock *c)
{
	struct foreign_clock *best = NULL, *fc;
	struct ClockIdentity best_id;
	struct port *piter;
	int fresh_best = 0;

	LIST_FOREACH(piter, &c->ports, list) {
		fc = port_compute_best(piter);
		if (!fc)
			continue;
		if (!best || dscmp(&fc->dataset, &best->dataset) > 0)
			best = fc;
	}

	/* If there's no UDS clock registered, explicitly handle UDS. */
	if (!c->uds_clock_filename) {
		/* Try the UDS as well -- the best master clock might be there! */
		fc = port_compute_best(c->uds_port);
		if (fc) {
			if (!best || dscmp(&fc->dataset, &best->dataset) > 0)
				best = fc;
		}
	}

	if (best) {
		best_id = best->dataset.identity;
	} else {
		best_id = c->dds.clockIdentity;
	}

	pr_notice("selected best master clock %s",
		  cid2str(&best_id));

	if (!cid_eq(&best_id, &c->best_id)) {
		clock_freq_est_reset(c);
		filter_reset(c->delay_filter);
		filter_reset(c->offset_filter);
		c->t1 = tmv_zero();
		c->t2 = tmv_zero();
		c->path_delay_set = 0;
		c->path_delay = 0;
		c->nrr = 1.0;
		/* The n_messages check should never be true (see
		 * port_compute_best()), but we keep it here for paranoia. */
		if (!best || (best->n_messages < 1)) {
			/* We are the grandmaster; clear the path trace. */
			struct parent_ds *dad = clock_parent_ds(c);

			dad->path_length = 0;
		} else {
			port_update_bmc_path_trace(
				best->port, TAILQ_FIRST(&best->messages));
		}
		fresh_best = 1;
	}

	c->best = best;
	c->best_id = best_id;

	LIST_FOREACH(piter, &c->ports, list) {
		handle_state_decision_for_port(c, piter, fresh_best);
	}
	/* The UDS can now have a master; run its state machine as well. */
	handle_state_decision_for_port(c, c->uds_port, fresh_best);
}

struct clock_description *clock_description(struct clock *c)
{
	return &c->desc;
}

int clock_num_ports(struct clock *c)
{
	return c->nports;
}

void clock_check_ts(struct clock *c, struct timespec ts)
{
	if (c->sanity_check &&
	    clockcheck_sample(c->sanity_check,
			      ts.tv_sec * NS_PER_SEC + ts.tv_nsec)) {
		filter_reset(c->offset_filter);
		servo_reset(c->servo);
	}
}

double clock_rate_ratio(struct clock *c)
{
	return servo_rate_ratio(c->servo);
}
