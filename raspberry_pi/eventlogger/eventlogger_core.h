#ifndef NEUROKAIROS_EVENTLOGGER_CORE_H
#define NEUROKAIROS_EVENTLOGGER_CORE_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <time.h>

#define EVENTLOGGER_MAX_INPUTS 32
#define EVENTLOGGER_NAME_LEN 32
#define EVENTLOGGER_PATH_LEN 512

typedef struct {
    char name[EVENTLOGGER_NAME_LEN];
    int gpio;
    int enabled;
    int deadtime_ms;
    int64_t last_event_monotonic_ns;
} eventlogger_input_t;

typedef struct {
    char chip[EVENTLOGGER_NAME_LEN];
    char config_path[EVENTLOGGER_PATH_LEN];
    char journal_dir[EVENTLOGGER_PATH_LEN];
    int flush_interval_ms;
    int flush_event_count;
    int status_interval_s;
    int cleanup_check_interval_s;
    double cleanup_threshold_gb;
    double cleanup_target_gb;
    int input_count;
    eventlogger_input_t inputs[EVENTLOGGER_MAX_INPUTS];
} eventlogger_config_t;

typedef struct {
    FILE *event_file;
    FILE *status_file;
    char event_date[11];
    char status_date[11];
    char event_path[EVENTLOGGER_PATH_LEN];
    char status_path[EVENTLOGGER_PATH_LEN];
    unsigned int pending_event_lines;
    time_t last_flush_time;
} eventlogger_journal_t;

typedef struct {
    char timestamp_utc[32];
    char reference_id[64];
    char leap_status[64];
    int stratum;
    double system_time_s;
    double last_offset_s;
    double rms_offset_s;
    double root_delay_s;
    double root_dispersion_s;
    double frequency_ppm;
    double skew_ppm;
} eventlogger_chrony_status_t;

/*
 * Initialize an event logger configuration.
 *
 * Inputs: config points to writable storage for one eventlogger_config_t.
 * Shapes/units: scalar structure; time fields use ms or s as named.
 * Returns: none. The structure is filled with safe defaults for eight GPIO
 * inputs: BCM 5, 6, 10, 11, 12, 13, 16, and 17.
 */
void eventlogger_config_init(eventlogger_config_t *config);

/*
 * Load an INI-like event logger configuration file.
 *
 * Inputs: path is a filesystem path; config points to initialized or
 * uninitialized writable storage.
 * Shapes/units: scalar strings; configured deadtime is milliseconds.
 * Returns: 0 on success, -1 on file/parse errors.
 */
int eventlogger_config_load(const char *path, eventlogger_config_t *config);

/*
 * Decide whether an edge should be accepted after per-input deadtime filtering.
 *
 * Inputs: input points to one configured GPIO input; monotonic_ns is the event
 * timestamp in nanoseconds from a monotonic clock.
 * Shapes/units: scalar input; timestamp in ns; deadtime in ms.
 * Returns: true when the event is accepted and updates input state, false when
 * it is filtered out.
 */
bool eventlogger_should_accept_event(eventlogger_input_t *input, int64_t monotonic_ns);

/*
 * Convert nanoseconds since the Unix epoch to UTC date/time text.
 *
 * Inputs: realtime_ns is nanoseconds since 1970-01-01T00:00:00Z.
 * Shapes/units: date_out has at least 11 bytes; time_out has at least 32 bytes.
 * Returns: 0 on success, -1 on conversion errors.
 */
int eventlogger_format_utc_from_ns(int64_t realtime_ns, char *date_out, size_t date_len,
                                   char *time_out, size_t time_len);

/*
 * Initialize and close daily journal file state.
 *
 * Inputs: journal points to writable state. No external files are opened by init.
 * Shapes/units: scalar structure.
 * Returns: none.
 */
void eventlogger_journal_init(eventlogger_journal_t *journal);
void eventlogger_journal_close(eventlogger_journal_t *journal);

/*
 * Write one edge event to the daily UTC TSV journal.
 *
 * Inputs: config supplies journal_dir; realtime_ns is UTC epoch ns;
 * monotonic_ns is monotonic-clock ns; input_name is the configured input label;
 * edge is "rising" or "falling".
 * Shapes/units: scalar event fields; timestamps in ns.
 * Returns: 0 on success, -1 on filesystem/write errors.
 */
int eventlogger_journal_write_event(eventlogger_journal_t *journal,
                                    const eventlogger_config_t *config,
                                    int64_t realtime_ns,
                                    int64_t monotonic_ns,
                                    const char *input_name,
                                    const char *edge);

/*
 * Flush pending event/status journal writes when count or time thresholds pass.
 *
 * Inputs: now is wall-clock seconds; force flushes regardless of thresholds.
 * Shapes/units: flush_interval_ms from config is milliseconds.
 * Returns: 0 on success, -1 on fflush errors.
 */
int eventlogger_journal_maybe_flush(eventlogger_journal_t *journal,
                                    const eventlogger_config_t *config,
                                    time_t now,
                                    bool force);

/*
 * Write one chrony tracking status row to a daily UTC TSV file.
 *
 * Inputs: status contains scalar chrony tracking fields; timestamp_utc must be UTC.
 * Shapes/units: offsets/delay/dispersion in seconds; frequency/skew in ppm.
 * Returns: 0 on success, -1 on filesystem/write errors.
 */
int eventlogger_journal_write_status(eventlogger_journal_t *journal,
                                     const eventlogger_config_t *config,
                                     const eventlogger_chrony_status_t *status);

/*
 * Query chronyc tracking and parse the clock status fields used by the logger.
 *
 * Inputs: status points to writable storage.
 * Shapes/units: output offsets/delay/dispersion are seconds; frequency/skew ppm.
 * Returns: 0 on success, -1 if chronyc cannot be run or no useful data is parsed.
 */
int eventlogger_read_chrony_status(eventlogger_chrony_status_t *status);

/*
 * Delete old inactive journal/status TSV files if free disk space is low.
 *
 * Inputs: config supplies journal_dir and free-space thresholds in GB;
 * active_event_path and active_status_path are current files to preserve.
 * Shapes/units: thresholds in decimal GB.
 * Returns: 0 on success, -1 on filesystem errors.
 */
int eventlogger_cleanup_if_needed(const eventlogger_config_t *config,
                                  const char *active_event_path,
                                  const char *active_status_path);

#endif
