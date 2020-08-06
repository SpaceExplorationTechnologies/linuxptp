/**
 * @file stats_file.h
 * @brief Support functions for writing statistics to a file.
 */
#ifndef HAVE_STATS_FILE_H
#define HAVE_STATS_FILE_H

#ifdef __GNUC__
__attribute__ ((format (printf, 2, 3)))
#endif
int stats_file_print(const char *stats_filename, char const *format, ...);

void stats_file_set_temp_filename(const char *name);

#endif
