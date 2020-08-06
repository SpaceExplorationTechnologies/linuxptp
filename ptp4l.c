/**
 * @file ptp4l.c
 * @brief PTP Boundary Clock main program
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
#include <limits.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <unistd.h>
#include <linux/net_tstamp.h>

#include "clock.h"
#include "config.h"
#include "mmin.h"
#include "ntpshm.h"
#include "pi.h"
#include "print.h"
#include "raw.h"
#include "sk.h"
#include "stats_file.h"
#include "transport.h"
#include "udp6.h"
#include "uds.h"
#include "util.h"
#include "version.h"

int assume_two_step = 0;

static struct config cfg_settings = {
	.interfaces = STAILQ_HEAD_INITIALIZER(cfg_settings.interfaces),

	.dds = {
		.dds = {
			.flags = DDS_TWO_STEP_FLAG,
			.priority1 = 128,
			.clockQuality.clockClass = 248,
			.clockQuality.clockAccuracy = 0xfe,
			.clockQuality.offsetScaledLogVariance = 0xffff,
			.priority2 = 128,
			.domainNumber = 0,
		},
		.free_running = 0,
		.freq_est_interval = 1,
		.grand_master_capable = 1,
		.stats_interval = 0,
		.kernel_leap = 1,
		.sanity_freq_limit = 200000000,
		.time_source = INTERNAL_OSCILLATOR,
		.clock_desc = {
			.productDescription = {
				.max_symbols = 64,
				.text = ";;",
				.length = 2,
			},
			.revisionData = {
				.max_symbols = 32,
				.text = ";;",
				.length = 2,
			},
			.userDescription      = { .max_symbols = 128 },
			.manufacturerIdentity = { 0, 0, 0 },
		},
		.delay_filter = FILTER_MOVING_MEDIAN,
		.delay_filter_length = 10,
		.boundary_clock_jbod = 0,
	},

	.pod = {
		.logAnnounceInterval = 1,
		.logSyncInterval = 0,
		.logMinDelayReqInterval = 0,
		.logMinPdelayReqInterval = 0,
		.announceReceiptTimeout = 3,
		.syncReceiptTimeout = 0,
		.transportSpecific = 0,
		.announce_span = 1,
		.sync_interval_addend_ns = 0,
		.path_trace_enabled = 0,
		.follow_up_info = 0,
		.freq_est_interval = 1,
		/* Default to very a large neighborPropDelay threshold */
		.neighborPropDelayThresh = 20000000,
		.min_neighbor_prop_delay = -20000000,
		.tx_timestamp_offset = 0,
		.rx_timestamp_offset = 0,
	},

	.timestamping = TS_HARDWARE,
	.dm = DM_E2E,
	.transport = TRANS_UDP_IPV4,

	.assume_two_step = &assume_two_step,
	.tx_timestamp_timeout = &sk_tx_timeout,
	.check_fup_sync = &sk_check_fupsync,

	.clock_servo = CLOCK_SERVO_PI,

	.step_threshold = &servo_step_threshold,
	.first_step_threshold = &servo_first_step_threshold,
	.max_frequency = &servo_max_frequency,
	.offset_filter = &configured_offset_filter,
	.offset_filter_length = &configured_offset_filter_length,
	.min_filter_start = &configured_mmin_start,
	.min_filter_stop = &configured_mmin_stop,

	.pi_proportional_const = &configured_pi_kp,
	.pi_integral_const = &configured_pi_ki,
	.pi_proportional_scale = &configured_pi_kp_scale,
	.pi_proportional_exponent = &configured_pi_kp_exponent,
	.pi_proportional_norm_max = &configured_pi_kp_norm_max,
	.pi_integral_scale = &configured_pi_ki_scale,
	.pi_integral_exponent = &configured_pi_ki_exponent,
	.pi_integral_norm_max = &configured_pi_ki_norm_max,
	.pi_init_freq_est_interval = &configured_pi_init_freq_est_interval,
	.ntpshm_segment = &ntpshm_segment,

	.ptp_dst_mac = ptp_dst_mac,
	.p2p_dst_mac = p2p_dst_mac,
	.udp6_scope = &udp6_scope,
	.uds_address = uds_path,

	.print_level = LOG_INFO,
	.use_syslog = 1,
	.verbose = 0,

	.cfg_ignore = 0,
};

static void usage(char *progname)
{
	fprintf(stderr,
		"\nusage: %s [options]\n\n"
		" Delay Mechanism\n\n"
		" -A        Auto, starting with E2E\n"
		" -E        E2E, delay request-response (default)\n"
		" -P        P2P, peer delay mechanism\n\n"
		" Network Transport\n\n"
		" -2        IEEE 802.3\n"
		" -4        UDP IPV4 (default)\n"
		" -6        UDP IPV6\n\n"
		" Time Stamping\n\n"
		" -H        HARDWARE (default)\n"
		" -S        SOFTWARE\n"
		" -L        LEGACY HW\n\n"
		" Other Options\n\n"
		" -C [mac]  generate clock id from specified MAC address\n"
		"           instead of using hardware address.\n"
		" -f [file] read configuration from 'file'\n"
		" -i [dev]  interface device to use, for example 'eth0'\n"
		"           (may be specified multiple times)\n"
		" -p [dev]  PTP hardware clock device to use, default auto\n"
		"           (ignored for SOFTWARE/LEGACY HW time stamping)\n"
		" -s        slave only mode (overrides configuration file)\n"
		" -l [num]  set the logging level to 'num'\n"
		" -m        print messages to stdout\n"
		" -q        do not print messages to the syslog\n"
		" -t [file] name of file for time-setting statistics\n"
		" -T [file] name of *temporary* file for '-t' atomic updates\n"
		" -U [file] name of the clock device to associate with UDS\n"
		" -Z [file] name of leap second table for use while grandmaster\n"
		"           file must be tzdata or USNO-formatted.\n"
		" -v        prints the software version and exits\n"
		" -h        prints this message and exits\n"
		"\n",
		progname);
}

int main(int argc, char *argv[])
{
	char *config = NULL, *req_phc = NULL, *progname;
	int c, i;
	struct interface *iface;
	int *cfg_ignore = &cfg_settings.cfg_ignore;
	enum delay_mechanism *dm = &cfg_settings.dm;
	enum transport_type *transport = &cfg_settings.transport;
	enum timestamp_type *timestamping = &cfg_settings.timestamping;
	struct clock *clock;
	struct defaultDS *ds = &cfg_settings.dds.dds;
	int phc_index = PHC_INDEX_CLOCK_REALTIME, required_modes = 0;
	const char *stats_filename = NULL;
	const char *leap_table_filename = NULL;
	const char *sx_init_gated;
	const char *uds_clock_filename = NULL;
	const char *clock_id = NULL;

	if (handle_term_signals())
		return -1;

	/* Set fault timeouts to a default value */
	for (i = 0; i < FT_CNT; i++) {
		cfg_settings.pod.flt_interval_pertype[i].type = FTMO_LOG2_SECONDS;
		cfg_settings.pod.flt_interval_pertype[i].val = 4;
	}

	/* Process the command line arguments. */
	progname = strrchr(argv[0], '/');
	progname = progname ? 1+progname : argv[0];
	while (EOF != (c = getopt(argc, argv, "AEP246HSLC:f:i:p:sl:mqt:T:U:Z:vh"))) {
		switch (c) {
		case 'A':
			*dm = DM_AUTO;
			*cfg_ignore |= CFG_IGNORE_DM;
			break;
		case 'E':
			*dm = DM_E2E;
			*cfg_ignore |= CFG_IGNORE_DM;
			break;
		case 'P':
			*dm = DM_P2P;
			*cfg_ignore |= CFG_IGNORE_DM;
			break;
		case '2':
			*transport = TRANS_IEEE_802_3;
			*cfg_ignore |= CFG_IGNORE_TRANSPORT;
			break;
		case '4':
			*transport = TRANS_UDP_IPV4;
			*cfg_ignore |= CFG_IGNORE_TRANSPORT;
			break;
		case '6':
			*transport = TRANS_UDP_IPV6;
			*cfg_ignore |= CFG_IGNORE_TRANSPORT;
			break;
		case 'H':
			*timestamping = TS_HARDWARE;
			*cfg_ignore |= CFG_IGNORE_TIMESTAMPING;
			break;
		case 'S':
			*timestamping = TS_SOFTWARE;
			*cfg_ignore |= CFG_IGNORE_TIMESTAMPING;
			break;
		case 'L':
			*timestamping = TS_LEGACY_HW;
			*cfg_ignore |= CFG_IGNORE_TIMESTAMPING;
			break;
		case 'C':
			clock_id = optarg;
			break;
		case 'f':
			config = optarg;
			break;
		case 'i':
			if (!config_create_interface(optarg, &cfg_settings))
				return -1;
			break;
		case 'p':
			req_phc = optarg;
			break;
		case 's':
			ds->flags |= DDS_SLAVE_ONLY;
			*cfg_ignore |= CFG_IGNORE_SLAVEONLY;
			break;
		case 'l':
			if (get_arg_val_i(c, optarg, &cfg_settings.print_level,
					  PRINT_LEVEL_MIN, PRINT_LEVEL_MAX))
				return -1;
			*cfg_ignore |= CFG_IGNORE_PRINT_LEVEL;
			break;
		case 'm':
			cfg_settings.verbose = 1;
			*cfg_ignore |= CFG_IGNORE_VERBOSE;
			break;
		case 'q':
			cfg_settings.use_syslog = 0;
			*cfg_ignore |= CFG_IGNORE_USE_SYSLOG;
			break;
		case 't':
			stats_filename = optarg;
			break;
		case 'T':
			stats_file_set_temp_filename(optarg);
			break;
		case 'U':
			uds_clock_filename = optarg;
			break;
		case 'Z':
			leap_table_filename = optarg;
			break;
		case 'v':
			version_show(stdout);
			return 0;
		case 'h':
			usage(progname);
			return 0;
		case '?':
			usage(progname);
			return -1;
		default:
			usage(progname);
			return -1;
		}
	}

	if (config && (c = config_read(config, &cfg_settings))) {
		return c;
	}
	if (!cfg_settings.dds.grand_master_capable &&
	    ds->flags & DDS_SLAVE_ONLY) {
		fprintf(stderr,
			"Cannot mix 1588 slaveOnly with 802.1AS !gmCapable.\n");
		return -1;
	}
	if (!cfg_settings.dds.grand_master_capable ||
	    ds->flags & DDS_SLAVE_ONLY) {
		ds->clockQuality.clockClass = 255;
	}
	if (cfg_settings.clock_servo == CLOCK_SERVO_NTPSHM) {
		cfg_settings.dds.kernel_leap = 0;
		cfg_settings.dds.sanity_freq_limit = 0;
	}

	print_set_progname(progname);
	print_set_verbose(cfg_settings.verbose);
	print_set_syslog(cfg_settings.use_syslog);
	print_set_level(cfg_settings.print_level);

	if (STAILQ_EMPTY(&cfg_settings.interfaces)) {
		fprintf(stderr, "no interface specified\n");
		usage(progname);
		return -1;
	}

	if (!(ds->flags & DDS_TWO_STEP_FLAG)) {
		switch (*timestamping) {
		case TS_SOFTWARE:
		case TS_LEGACY_HW:
		case TS_LEGACY_SW:
			fprintf(stderr, "one step is only possible "
				"with hardware time stamping\n");
			return -1;
		case TS_HARDWARE:
			*timestamping = TS_ONESTEP;
			break;
		case TS_ONESTEP:
			break;
		}
	}

	switch (*timestamping) {
	case TS_SOFTWARE:
		required_modes |= SOF_TIMESTAMPING_TX_SOFTWARE |
			SOF_TIMESTAMPING_RX_SOFTWARE |
			SOF_TIMESTAMPING_SOFTWARE;
		break;
	case TS_LEGACY_HW:
		required_modes |= SOF_TIMESTAMPING_TX_HARDWARE |
			SOF_TIMESTAMPING_RX_HARDWARE |
			SOF_TIMESTAMPING_SYS_HARDWARE;
		break;
	case TS_HARDWARE:
	case TS_ONESTEP:
		required_modes |= SOF_TIMESTAMPING_TX_HARDWARE |
			SOF_TIMESTAMPING_RX_HARDWARE |
			SOF_TIMESTAMPING_RAW_HARDWARE;
		break;
	case TS_LEGACY_SW:
		/* No required modes for this. */
		break;
	}

	/* Init interface configs and check whether timestamping mode is
	 * supported. */
	STAILQ_FOREACH(iface, &cfg_settings.interfaces, list) {
		config_init_interface(iface, &cfg_settings);
		if (iface->ts_info.valid &&
		    ((iface->ts_info.so_timestamping & required_modes) != required_modes)) {
			fprintf(stderr, "interface '%s' does not support "
				        "requested timestamping mode.\n",
				iface->name);
			return -1;
		}
	}

	/* determine PHC Clock index */
	iface = STAILQ_FIRST(&cfg_settings.interfaces);
	if (cfg_settings.dds.free_running) {
		phc_index = PHC_INDEX_CLOCK_REALTIME;
	} else if (*timestamping == TS_SOFTWARE ||
		   *timestamping == TS_LEGACY_HW ||
		   *timestamping == TS_LEGACY_SW) {
		phc_index = PHC_INDEX_CLOCK_REALTIME;
	} else if (req_phc) {
		if (1 != sscanf(req_phc, "/dev/ptp%d", &phc_index)) {
			fprintf(stderr, "bad ptp device string\n");
			return -1;
		}
	} else if (iface->ts_info.valid) {
		phc_index = iface->ts_info.phc_index;
	} else {
		fprintf(stderr, "ptp device not specified and\n"
			        "automatic determination is not\n"
			        "supported. please specify ptp device\n");
		return -1;
	}

	if (phc_index >= 0) {
		pr_info("selected /dev/ptp%d as PTP clock", phc_index);
	}

	/* If a clock_id string was specified, use it as the basis for
	 * the clock id instead of the ethernet controller MAC. Some
	 * network interfaces don't provide sufficiently system-unique
	 * MAC addresses, so this allows an override.
	 */
	if (clock_id) {
		unsigned char mac[MAC_LEN];
		if (str2mac(clock_id, mac)) {
			fprintf(stderr, "failed to parse clock id\n");
			return -1;
		}

		ds->clockIdentity.id[0] = mac[0];
		ds->clockIdentity.id[1] = mac[1];
		ds->clockIdentity.id[2] = mac[2];
		ds->clockIdentity.id[3] = 0xFF;
		ds->clockIdentity.id[4] = 0xFE;
		ds->clockIdentity.id[5] = mac[3];
		ds->clockIdentity.id[6] = mac[4];
		ds->clockIdentity.id[7] = mac[5];
	}
	else if (generate_clock_identity(&ds->clockIdentity, iface->name)) {
		fprintf(stderr, "failed to generate a clock identity\n");
		return -1;
	}

	clock = clock_create(phc_index, &cfg_settings.interfaces,
			     *timestamping, &cfg_settings.dds,
			     cfg_settings.clock_servo, stats_filename,
			     leap_table_filename, uds_clock_filename);
	if (!clock) {
		fprintf(stderr, "failed to create a clock\n");
		return -1;
	}

	/* Wait at the initialization gate. This allows us to create a
	 * barrier in the runtime file so that processes depending on
	 * ptp4l wait for its initialization to complete. */
	sx_init_gated = getenv("SX_INIT_GATED");
	if (sx_init_gated != NULL && !strcasecmp(sx_init_gated, "true")) {
		printf("Waiting at the initialization gate.\n");
		fflush(stdout);
		/* sxruntime_start will send a SIGCONT to resume the
		 * process when ready. */
		kill(getpid(), SIGSTOP);
		printf("Proceeding past the initialization gate.\n");
		fflush(stdout);
	}

	while (is_running()) {
		if (clock_poll(clock))
			break;
	}

	clock_destroy(clock);
	config_destroy(&cfg_settings);
	return 0;
}
