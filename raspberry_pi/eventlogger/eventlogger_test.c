#include "eventlogger_core.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <unistd.h>

static int failures = 0;

static void require_true(int condition, const char *message)
{
    if (!condition) {
        fprintf(stderr, "FAIL: %s\n", message);
        failures++;
    }
}

static void test_defaults(void)
{
    eventlogger_config_t config;
    eventlogger_config_init(&config);

    require_true(strcmp(config.journal_dir, "/var/lib/neurokairos/eventlogger/journal") == 0,
                 "default journal directory");
    require_true(config.flush_interval_ms == 1000, "default flush interval");
    require_true(config.flush_event_count == 1000, "default flush count");
    require_true(config.cleanup_threshold_gb == 5.0, "default cleanup threshold");
    require_true(config.cleanup_target_gb == 8.0, "default cleanup target");
    require_true(config.input_count == 8, "default input count");
    require_true(config.inputs[0].gpio == 5, "default DIN1 GPIO");
    require_true(config.inputs[1].gpio == 6, "default DIN2 GPIO");
    require_true(config.inputs[2].gpio == 13, "default DIN3 GPIO");
    require_true(config.inputs[3].gpio == 19, "default DIN4 GPIO");
    require_true(config.inputs[4].gpio == 26, "default DIN5 GPIO");
    require_true(config.inputs[5].gpio == 16, "default DIN6 GPIO");
    require_true(config.inputs[6].gpio == 20, "default DIN7 GPIO");
    require_true(config.inputs[7].gpio == 21, "default DIN8 GPIO");
}

static void test_deadtime(void)
{
    eventlogger_input_t input;
    memset(&input, 0, sizeof(input));
    input.deadtime_ms = 1;

    require_true(eventlogger_should_accept_event(&input, 1000000000LL), "first event accepted");
    require_true(!eventlogger_should_accept_event(&input, 1000500000LL), "0.5 ms event filtered");
    require_true(eventlogger_should_accept_event(&input, 1001000000LL), "1.0 ms event accepted");

    input.deadtime_ms = 0;
    require_true(eventlogger_should_accept_event(&input, 1001000100LL), "zero deadtime accepted");
}

static void test_utc_format(void)
{
    char date[11];
    char utc_time[32];

    require_true(eventlogger_format_utc_from_ns(1735689600123456789LL, date, sizeof(date),
                                                utc_time, sizeof(utc_time)) == 0,
                 "UTC formatter succeeds");
    require_true(strcmp(date, "2025-01-01") == 0, "UTC date");
    require_true(strcmp(utc_time, "2025-01-01T00:00:00.123456789Z") == 0, "UTC timestamp");
}

static void test_config_file(void)
{
    char path[] = "/tmp/neurokairos_eventlogger_test_XXXXXX";
    int fd = mkstemp(path);
    FILE *file;
    eventlogger_config_t config;

    require_true(fd >= 0, "create temp config");
    file = fdopen(fd, "w");
    require_true(file != NULL, "open temp config");
    fprintf(file,
            "[global]\n"
            "journal_dir = /tmp/eventlogger\n"
            "flush_interval_ms = 250\n"
            "cleanup_threshold_gb = 3\n"
            "deadtime_ms = 2\n"
            "\n"
            "[input DIN1]\n"
            "gpio = 21\n"
            "deadtime_ms = 0\n"
            "\n"
            "[input DIN2]\n"
            "enabled = false\n");
    fclose(file);

    require_true(eventlogger_config_load(path, &config) == 0, "load config");
    require_true(strcmp(config.journal_dir, "/tmp/eventlogger") == 0, "configured journal dir");
    require_true(config.flush_interval_ms == 250, "configured flush interval");
    require_true(config.inputs[0].gpio == 21, "configured GPIO");
    require_true(config.inputs[0].deadtime_ms == 0, "configured per-input deadtime");
    require_true(config.inputs[1].enabled == 0, "configured disabled input");
    unlink(path);
}

static void test_journal_header(void)
{
    char dir[] = "/tmp/neurokairos_eventlogger_journal_XXXXXX";
    int dir_fd;
    eventlogger_config_t config;
    eventlogger_journal_t journal;
    char event_path[512];
    FILE *file;
    char header[256];

    dir_fd = mkstemp(dir);
    require_true(dir_fd >= 0, "reserve journal temp path");
    close(dir_fd);
    unlink(dir);
    require_true(mkdir(dir, 0700) == 0, "create journal temp dir");
    eventlogger_config_init(&config);
    snprintf(config.journal_dir, sizeof(config.journal_dir), "%s", dir);
    eventlogger_journal_init(&journal);

    require_true(eventlogger_journal_write_event(&journal, &config,
                                                 1735689600000000000LL,
                                                 123456789LL,
                                                 "DIN1", "rising") == 0,
                 "write event row");
    eventlogger_journal_maybe_flush(&journal, &config, 1, true);
    snprintf(event_path, sizeof(event_path), "%s/2025-01_journal/all_events_2025-01-01.tsv", dir);
    file = fopen(event_path, "r");
    require_true(file != NULL, "open event TSV");
    require_true(fgets(header, sizeof(header), file) != NULL, "read event TSV header");
    require_true(strcmp(header, "utc_time\trealtime_ns\tmonotonic_ns\tinput\tedge\n") == 0,
                 "event TSV header");
    fclose(file);
    eventlogger_journal_close(&journal);
    unlink(event_path);
    rmdir(dir);
}

int main(void)
{
    test_defaults();
    test_deadtime();
    test_utc_format();
    test_config_file();
    test_journal_header();
    return failures == 0 ? 0 : 1;
}
