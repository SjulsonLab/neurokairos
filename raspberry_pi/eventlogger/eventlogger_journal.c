#include "eventlogger_core.h"

#include <dirent.h>
#include <errno.h>
#include <limits.h>
#include <stdarg.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/statvfs.h>
#include <unistd.h>

static int mkdir_p(const char *path)
{
    char tmp[EVENTLOGGER_PATH_LEN];
    char *cursor;

    snprintf(tmp, sizeof(tmp), "%s", path);
    for (cursor = tmp + 1; *cursor; cursor++) {
        if (*cursor == '/') {
            *cursor = '\0';
            if (mkdir(tmp, 0755) != 0 && errno != EEXIST) {
                return -1;
            }
            *cursor = '/';
        }
    }
    if (mkdir(tmp, 0755) != 0 && errno != EEXIST) {
        return -1;
    }
    return 0;
}

static int file_exists_and_nonempty(const char *path)
{
    struct stat st;
    if (stat(path, &st) != 0) {
        return 0;
    }
    return st.st_size > 0;
}

static int64_t timespec_to_ns(const struct timespec *ts)
{
    return (int64_t)ts->tv_sec * 1000000000LL + (int64_t)ts->tv_nsec;
}

int eventlogger_format_utc_from_ns(int64_t realtime_ns, char *date_out, size_t date_len,
                                   char *time_out, size_t time_len)
{
    time_t seconds = (time_t)(realtime_ns / 1000000000LL);
    long nanoseconds = (long)(realtime_ns % 1000000000LL);
    struct tm tm_utc;

    if (nanoseconds < 0) {
        seconds--;
        nanoseconds += 1000000000L;
    }

    if (gmtime_r(&seconds, &tm_utc) == NULL) {
        return -1;
    }

    snprintf(date_out, date_len, "%04d-%02d-%02d",
             tm_utc.tm_year + 1900, tm_utc.tm_mon + 1, tm_utc.tm_mday);
    snprintf(time_out, time_len, "%04d-%02d-%02dT%02d:%02d:%02d.%09ldZ",
             tm_utc.tm_year + 1900, tm_utc.tm_mon + 1, tm_utc.tm_mday,
             tm_utc.tm_hour, tm_utc.tm_min, tm_utc.tm_sec, nanoseconds);
    return 0;
}

void eventlogger_journal_init(eventlogger_journal_t *journal)
{
    memset(journal, 0, sizeof(*journal));
}

void eventlogger_journal_close(eventlogger_journal_t *journal)
{
    if (journal->event_file) {
        fclose(journal->event_file);
        journal->event_file = NULL;
    }
    if (journal->status_file) {
        fclose(journal->status_file);
        journal->status_file = NULL;
    }
}

static int open_daily_file(FILE **file, char *stored_date, size_t stored_date_len,
                           char *stored_path, size_t stored_path_len,
                           const char *journal_dir, const char *prefix,
                           const char *date, const char *header)
{
    int needs_header;

    if (*file && strcmp(stored_date, date) == 0) {
        return 0;
    }

    if (*file) {
        fclose(*file);
        *file = NULL;
    }

    if (mkdir_p(journal_dir) != 0) {
        return -1;
    }

    snprintf(stored_date, stored_date_len, "%s", date);
    snprintf(stored_path, stored_path_len, "%s/%s_%s.tsv", journal_dir, prefix, date);
    needs_header = !file_exists_and_nonempty(stored_path);

    *file = fopen(stored_path, "a");
    if (!*file) {
        return -1;
    }
    if (needs_header && fputs(header, *file) < 0) {
        return -1;
    }
    return 0;
}

int eventlogger_journal_write_event(eventlogger_journal_t *journal,
                                    const eventlogger_config_t *config,
                                    int64_t realtime_ns,
                                    int64_t monotonic_ns,
                                    const char *input_name,
                                    const char *edge)
{
    char date[11];
    char utc_time[32];

    if (eventlogger_format_utc_from_ns(realtime_ns, date, sizeof(date),
                                       utc_time, sizeof(utc_time)) != 0) {
        return -1;
    }
    if (open_daily_file(&journal->event_file, journal->event_date, sizeof(journal->event_date),
                        journal->event_path, sizeof(journal->event_path),
                        config->journal_dir, "events", date,
                        "utc_time\trealtime_ns\tmonotonic_ns\tinput\tedge\n") != 0) {
        return -1;
    }

    if (fprintf(journal->event_file, "%s\t%lld\t%lld\t%s\t%s\n",
                utc_time, (long long)realtime_ns, (long long)monotonic_ns,
                input_name, edge) < 0) {
        return -1;
    }
    journal->pending_event_lines++;
    return 0;
}

int eventlogger_journal_write_status(eventlogger_journal_t *journal,
                                     const eventlogger_config_t *config,
                                     const eventlogger_chrony_status_t *status)
{
    char date[11];
    int64_t now_ns = 0;
    struct timespec now;

    if (clock_gettime(CLOCK_REALTIME, &now) == 0) {
        now_ns = timespec_to_ns(&now);
    }
    if (eventlogger_format_utc_from_ns(now_ns, date, sizeof(date), (char[32]){0}, 32) != 0) {
        return -1;
    }
    if (open_daily_file(&journal->status_file, journal->status_date, sizeof(journal->status_date),
                        journal->status_path, sizeof(journal->status_path),
                        config->journal_dir, "chrony_status", date,
                        "timestamp_utc\treference_id\tstratum\tleap_status\tsystem_time_s\tlast_offset_s\trms_offset_s\troot_delay_s\troot_dispersion_s\tfrequency_ppm\tskew_ppm\n") != 0) {
        return -1;
    }

    if (fprintf(journal->status_file,
                "%s\t%s\t%d\t%s\t%.12g\t%.12g\t%.12g\t%.12g\t%.12g\t%.12g\t%.12g\n",
                status->timestamp_utc, status->reference_id, status->stratum, status->leap_status,
                status->system_time_s, status->last_offset_s, status->rms_offset_s,
                status->root_delay_s, status->root_dispersion_s,
                status->frequency_ppm, status->skew_ppm) < 0) {
        return -1;
    }
    return 0;
}

int eventlogger_journal_maybe_flush(eventlogger_journal_t *journal,
                                    const eventlogger_config_t *config,
                                    time_t now,
                                    bool force)
{
    bool count_due = config->flush_event_count > 0 &&
                     journal->pending_event_lines >= (unsigned int)config->flush_event_count;
    bool time_due = journal->last_flush_time == 0 ||
                    (config->flush_interval_ms > 0 &&
                     (now - journal->last_flush_time) * 1000 >= config->flush_interval_ms);

    if (!force && !count_due && !time_due) {
        return 0;
    }
    if (journal->event_file && fflush(journal->event_file) != 0) {
        return -1;
    }
    if (journal->status_file && fflush(journal->status_file) != 0) {
        return -1;
    }
    journal->pending_event_lines = 0;
    journal->last_flush_time = now;
    return 0;
}

typedef struct {
    char path[EVENTLOGGER_PATH_LEN];
    time_t mtime;
} cleanup_candidate_t;

static int compare_candidates(const void *a, const void *b)
{
    const cleanup_candidate_t *left = (const cleanup_candidate_t *)a;
    const cleanup_candidate_t *right = (const cleanup_candidate_t *)b;
    if (left->mtime < right->mtime) {
        return -1;
    }
    if (left->mtime > right->mtime) {
        return 1;
    }
    return strcmp(left->path, right->path);
}

static double free_space_gb(const char *path)
{
    struct statvfs fs;
    if (statvfs(path, &fs) != 0) {
        return -1.0;
    }
    return ((double)fs.f_bavail * (double)fs.f_frsize) / 1000000000.0;
}

static int append_cleanup_log(const char *journal_dir, const char *message, const char *path)
{
    char log_path[EVENTLOGGER_PATH_LEN];
    FILE *file;
    struct timespec now;
    char date[11];
    char utc_time[32];

    if (clock_gettime(CLOCK_REALTIME, &now) != 0) {
        return -1;
    }
    eventlogger_format_utc_from_ns(timespec_to_ns(&now), date, sizeof(date),
                                   utc_time, sizeof(utc_time));
    snprintf(log_path, sizeof(log_path), "%s/cleanup_audit.tsv", journal_dir);
    file = fopen(log_path, "a");
    if (!file) {
        return -1;
    }
    fprintf(file, "%s\t%s\t%s\n", utc_time, message, path);
    fclose(file);
    return 0;
}

int eventlogger_cleanup_if_needed(const eventlogger_config_t *config,
                                  const char *active_event_path,
                                  const char *active_status_path)
{
    DIR *dir;
    struct dirent *entry;
    cleanup_candidate_t candidates[512];
    size_t count = 0;
    size_t i;
    double free_gb;

    free_gb = free_space_gb(config->journal_dir);
    if (free_gb < 0 || free_gb >= config->cleanup_threshold_gb) {
        return free_gb < 0 ? -1 : 0;
    }

    dir = opendir(config->journal_dir);
    if (!dir) {
        return -1;
    }

    while ((entry = readdir(dir)) != NULL && count < 512) {
        struct stat st;
        char path[EVENTLOGGER_PATH_LEN];
        size_t name_len = strlen(entry->d_name);

        if (name_len < 4 || strcmp(entry->d_name + name_len - 4, ".tsv") != 0) {
            continue;
        }
        snprintf(path, sizeof(path), "%s/%s", config->journal_dir, entry->d_name);
        if ((active_event_path && strcmp(path, active_event_path) == 0) ||
            (active_status_path && strcmp(path, active_status_path) == 0)) {
            continue;
        }
        if (stat(path, &st) != 0 || !S_ISREG(st.st_mode)) {
            continue;
        }
        snprintf(candidates[count].path, sizeof(candidates[count].path), "%s", path);
        candidates[count].mtime = st.st_mtime;
        count++;
    }
    closedir(dir);

    qsort(candidates, count, sizeof(candidates[0]), compare_candidates);
    for (i = 0; i < count && free_space_gb(config->journal_dir) < config->cleanup_target_gb; i++) {
        if (unlink(candidates[i].path) == 0) {
            append_cleanup_log(config->journal_dir, "deleted", candidates[i].path);
        }
    }
    return 0;
}
