/**
 * @file clock.h
 * @brief Implements a PTP clock.
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
#ifndef HAVE_CLOCK_H
#define HAVE_CLOCK_H

#include "dm.h"
#include "ds.h"
#include "config.h"
#include "notification.h"
#include "servo.h"
#include "tlv.h"
#include "tmv.h"
#include "transport.h"

/**
 * Special PHC index values. Index 0 - n are managed by the kernel's PTP
 * driver stack.
 */
#define PHC_INDEX_CLOCK_REALTIME -1
#define PHC_INDEX_UDS -2

/**
 * This variable determines the filter type for the input offsets.
 * Defaults to moving median fitler.
 */
extern enum filter_type configured_offset_filter;

/**
 * This variable determines the length of the filter on the input offsets.
 * Defaults to 1 (i.e., no filtering).
 */
extern int configured_offset_filter_length;

struct ptp_message; /*forward declaration*/

/** Opaque type. */
struct clock;

/**
 * Obtains a reference to the best foreign master of a clock.
 * @param c  The clock instance.
 * @return   A pointer to the data set of the foreign master,
 *           or NULL if none has been yet discovered.
 */
struct dataset *clock_best_foreign(struct clock *c);

/**
 * Obtains a reference to the port with the best foreign master.
 * @param c  The clock instance.
 * @return   A pointer to the port with the best foreign master,
 *           or NULL if none has been yet discovered.
 */
struct port *clock_best_port(struct clock *c);

/**
 * Obtain the clockClass attribute from a clock.
 * @param c  The clock instance.
 * @return   The value of the clock's class.
 */
UInteger8 clock_class(struct clock *c);

/**
 * Create a clock instance. There can only be one clock in any system,
 * so subsequent calls will destroy the previous clock instance.
 *
 * @param phc_index           PTP hardware clock device to use.
 *                            Pass -1 to select CLOCK_REALTIME.
 * @param ifaces              A queue of network interfaces.
 * @param timestamping        The timestamping mode for this clock.
 * @param dds                 A pointer to a default data set for the clock.
 * @param servo               The servo that this clock will use.
 * @param stats_filename      The file to output clock statistics to (can be NULL).
 * @param leap_table_filename The filename which contains the leap second table.
 *                            If NULL, don't use a table.
 * @param uds_clock_filename  The file to a posix clock device to associate a clock
 *                            with the UDS port (can be NULL).
 * @return             A pointer to the single global clock instance.
 */
struct clock *clock_create(int phc_index, struct interfaces_head *ifaces,
			   enum timestamp_type timestamping, struct default_ds *dds,
			   enum servo_type servo, const char *stats_filename,
			   const char *leap_table_filename, const char *uds_clock_filename);
/**
 * Obtains a clock's default data set.
 * @param c  The clock instance.
 * @return   A pointer to the data set of the clock.
 */
struct dataset *clock_default_ds(struct clock *c);

/**
 * Free all of the resources associated with a clock.
 * @param c  The clock instance.
 */
void clock_destroy(struct clock *c);

/**
 * Obtain the domain number from a clock's default data set.
 * @param c  The clock instance.
 * @return   The PTP domain number.
 */
UInteger8 clock_domain_number(struct clock *c);

/**
 * Provide the follow_up info TLV from a slave port.
 * @param c  The clock instance.
 * @param f  Pointer to the TLV.
 */
void clock_follow_up_info(struct clock *c, struct follow_up_info_tlv *f);

/**
 * Obtain the gmCapable flag from a clock's default data set.
 * This function is specific to the 802.1AS standard.
 * @param c  The clock instance.
 * @return One if the clock is capable of becoming grand master, zero otherwise.
 */
int clock_gm_capable(struct clock *c);

/**
 * Obtain a clock's identity from its default data set.
 * @param c  The clock instance.
 * @return   The clock's identity.
 */
struct ClockIdentity clock_identity(struct clock *c);

/**
 * Informs clock that a file descriptor of one of its ports changed. The
 * clock will rebuild its array of file descriptors to poll.
 * @param c    The clock instance.
 */
void clock_fda_changed(struct clock *c);

/**
 * Manage the clock according to a given message.
 * @param c    The clock instance.
 * @param p    The port on which the message arrived.
 * @param msg  A management message.
 * @return     One if the management action caused a change that
 *             implies a state decision event, zero otherwise.
 */
int clock_manage(struct clock *c, struct port *p, struct ptp_message *msg);

/**
 * Send notification about an event to all subscribers.
 * @param c      The clock instance.
 * @param msg    The PTP message to send, in network byte order.
 * @param msglen The length of the message in bytes.
 * @param event  The event that occured.
 */
void clock_send_notification(struct clock *c, struct ptp_message *msg,
			     int msglen, enum notification event);

/**
 * Construct and send notification to subscribers about an event that
 * occured on the clock.
 * @param c      The clock instance.
 * @param event  The identification of the event.
 */
void clock_notify_event(struct clock *c, enum notification event);

/**
 * Obtain a clock's parent data set.
 * @param c  The clock instance.
 * @return   A pointer to the parent data set of the clock.
 */
struct parent_ds *clock_parent_ds(struct clock *c);

/**
 * Obtain the parent port identity from a clock's parent data set.
 * @param c  The clock instance.
 * @return   The parent port identity.
 */
struct PortIdentity clock_parent_identity(struct clock *c);

/**
 * Provide a data point to estimate the path delay.
 * @param c           The clock instance.
 * @param req         The transmission time of the delay request message.
 * @param rx          The reception time of the delay request message,
 *                    as reported in the delay response message.
 * @param correction  The correction field from the delay response message.
 */
void clock_path_delay(struct clock *c, struct timespec req, struct timestamp rx,
		      Integer64 correction);

/**
 * Provide the estimated peer delay from a slave port.
 * @param c           The clock instance.
 * @param ppd         The peer delay as measured on a slave port.
 * @param nrr         The neighbor rate ratio as measured on a slave port.
 */
void clock_peer_delay(struct clock *c, tmv_t ppd, double nrr);

/**
 * Poll for events and dispatch them.
 * @param c A pointer to a clock instance obtained with clock_create().
 * @return  Zero on success, non-zero otherwise.
 */
int clock_poll(struct clock *c);

/**
 * Obtain the slave-only flag from a clock's default data set.
 * @param c  The clock instance.
 * @return   The value of the clock's slave-only flag.
 */
int clock_slave_only(struct clock *c);

/**
 * Obtain the steps removed field from a clock's current data set.
 * @param c  The clock instance.
 * @return   The value of the clock's steps removed field.
 */
UInteger16 clock_steps_removed(struct clock *c);

/**
 * Switch to a new PTP Hardware Clock, for use with the "jbod" mode.
 * @param c          The clock instance.
 * @param phc_index  The index of the PHC device to use.
 * @return           Zero on success, non-zero otherwise.
 */
int clock_switch_phc(struct clock *c, int phc_index);

/**
 * Provide a data point to synchronize the clock.
 * @param c            The clock instance to synchronize.
 * @param ingress_ts   The ingress time stamp on the sync message.
 * @param origin_ts    The reported transmission time of the sync message.
 * @param correction1  The correction field of the sync message.
 * @param correction2  The correction field of the follow up message.
 *                     Pass zero in the case of one step operation.
 * @return             The state of the clock's servo.
 */
enum servo_state clock_synchronize(struct clock *c,
				   struct timespec ingress_ts,
				   struct timestamp origin_ts,
				   Integer64 correction1,
				   Integer64 correction2);

/**
 * Inform a slaved clock about the master's sync interval.
 * @param c  The clock instance.
 * @param n  The logarithm base two of the sync interval.
 */
void clock_sync_interval(struct clock *c, int n);

/**
 * Obtain a clock's time properties data set.
 * @param c  The clock instance.
 * @return   A pointer to the time properties data set of the clock.
 */
struct timePropertiesDS *clock_time_properties(struct clock *c);

/**
 * Update a clock's time properties data set.
 * @param c   The clock instance.
 * @param tds The new time properties data set for the clock.
 */
void clock_update_time_properties(struct clock *c, struct timePropertiesDS tds);

/**
 * Obtain a clock's description.
 * @param c  The clock instance.
 * @return   A pointer to the clock_description of the clock.
 */
struct clock_description *clock_description(struct clock *c);

/**
 * Obtain the number of ports a clock has, excluding the UDS port.
 * @param c  The clock instance.
 * @return   The number of ports.
 */
int clock_num_ports(struct clock *c);

/**
 * Perform a sanity check on a time stamp made by a clock.
 * @param c  The clock instance.
 * @param ts The time stamp.
 */
void clock_check_ts(struct clock *c, struct timespec ts);

/**
 * Obtain ratio between master's frequency and current clock frequency.
 * @param c  The clock instance.
 * @return   The rate ratio, 1.0 is returned when not known.
 */
double clock_rate_ratio(struct clock *c);

/**
 * Obtain the current UTC offset.
 * @param c  The clock instance.
 * @return   The current UTC offset.
 */
int clock_current_utc_offset(struct clock *c);

#endif
