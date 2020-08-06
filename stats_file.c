/**
 * @file stats_file.c
 */
#include <errno.h>
#include <fcntl.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>

#include "print.h"

#include "stats_file.h"

/*
 * If non-NULL, then output will first be written to this file,
 * then atomically renamed to stats_filename.
 */
static const char *temp_filename = NULL;

void stats_file_set_temp_filename(const char *name)
{
	temp_filename = name;
}

int stats_file_print(const char *stats_filename, char const *format, ...)
{
	va_list ap;
	char buf[4096];
	int fd = -1;
	int len;
	int rc;

	if (!stats_filename)
		return 0;

	va_start(ap, format);
	len = vsnprintf(buf, sizeof(buf), format, ap);
	if (len > sizeof(buf)) {
		pr_info("Stats file output truncated (output len > %d)\n",
			(int)sizeof(buf));
		goto err;
	}
	if (len < 0) {
		pr_info("Error %d outputting stats to buffer\n", len);
		goto err;
	}
	va_end(ap);

	if (temp_filename)
		fd = creat(temp_filename, S_IRUSR | S_IWUSR);
	else
		fd = creat(stats_filename, S_IRUSR | S_IWUSR);
	if (fd < 0) {
		pr_info("Stats file could not be created: %s\n",
			strerror(errno));
		goto err;
	}

	rc = write(fd, buf, len);
	if (rc < 0) {
		pr_info("Error writing to stats file: %s\n", strerror(errno));
		goto err;
	} else if (rc < len) {
		pr_info("On write stats file, only wrote %d of %d bytes\n",
			rc, len);
		goto err;
	}

	rc = close(fd);
	if (rc != 0) {
		pr_info("Failed to close stats file: %s\n", strerror(errno));
		goto err;
	}

	if (temp_filename) {
		rc = rename(temp_filename, stats_filename);
		if (rc != 0) {
			pr_info("Failed to rename stats file on output: %s\n",
				strerror(errno));
			goto err;
		}
	}

	return 0;

err:
	if (fd >= 0) {
		close(fd);
	}
	return -1;
}
