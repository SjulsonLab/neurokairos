#include "eventlogger_core.h"

#include <errno.h>
#include <gpiod.h>
#include <signal.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

static volatile sig_atomic_t keep_running = 1;

static void handle_signal(int signum)
{
    (void)signum;
    keep_running = 0;
}

static int64_t timespec_to_ns(const struct timespec *ts)
{
    return (int64_t)ts->tv_sec * 1000000000LL + (int64_t)ts->tv_nsec;
}

static const char *edge_name(int event_type)
{
    if (event_type == GPIOD_LINE_EVENT_RISING_EDGE) {
        return "rising";
    }
    if (event_type == GPIOD_LINE_EVENT_FALLING_EDGE) {
        return "falling";
    }
    return "unknown";
}

static int find_input_for_line(struct gpiod_line *line,
                               eventlogger_input_t **line_inputs,
                               struct gpiod_line **lines,
                               int line_count)
{
    int i;
    for (i = 0; i < line_count; i++) {
        if (lines[i] == line) {
            return i;
        }
    }
    (void)line_inputs;
    return -1;
}

static int run_event_loop(const eventlogger_config_t *config)
{
    struct gpiod_chip *chip = NULL;
    struct gpiod_line_bulk requested = GPIOD_LINE_BULK_INITIALIZER;
    struct gpiod_line *lines[EVENTLOGGER_MAX_INPUTS];
    eventlogger_input_t *line_inputs[EVENTLOGGER_MAX_INPUTS];
    int line_count = 0;
    eventlogger_journal_t journal;
    time_t last_status = 0;
    time_t last_cleanup = 0;
    int i;

    eventlogger_journal_init(&journal);

    chip = gpiod_chip_open_by_name(config->chip);
    if (!chip) {
        fprintf(stderr, "Failed to open GPIO chip %s: %s\n", config->chip, strerror(errno));
        return 1;
    }

    for (i = 0; i < config->input_count; i++) {
        struct gpiod_line *line;
        if (!config->inputs[i].enabled) {
            continue;
        }
        if (config->inputs[i].gpio < 0) {
            fprintf(stderr, "Input %s has invalid GPIO %d\n",
                    config->inputs[i].name, config->inputs[i].gpio);
            return 1;
        }
        line = gpiod_chip_get_line(chip, (unsigned int)config->inputs[i].gpio);
        if (!line) {
            fprintf(stderr, "Failed to get GPIO %d for %s\n",
                    config->inputs[i].gpio, config->inputs[i].name);
            return 1;
        }
        gpiod_line_bulk_add(&requested, line);
        lines[line_count] = line;
        line_inputs[line_count] = (eventlogger_input_t *)&config->inputs[i];
        line_count++;
    }

    if (line_count == 0) {
        fprintf(stderr, "No enabled GPIO inputs configured\n");
        return 1;
    }

    {
        struct gpiod_line_request_config request = {
            .consumer = "neurokairos-eventlogger",
            .request_type = GPIOD_LINE_REQUEST_EVENT_BOTH_EDGES,
            .flags = 0,
        };
        if (gpiod_line_request_bulk(&requested, &request, NULL) != 0) {
            fprintf(stderr, "Failed to request GPIO edge events: %s\n", strerror(errno));
            return 1;
        }
    }

    fprintf(stderr, "Logging %d GPIO inputs to %s\n", line_count, config->journal_dir);

    while (keep_running) {
        struct timespec timeout = {.tv_sec = 1, .tv_nsec = 0};
        struct gpiod_line_bulk events = GPIOD_LINE_BULK_INITIALIZER;
        time_t now_s = time(NULL);
        int wait_result;

        wait_result = gpiod_line_event_wait_bulk(&requested, &timeout, &events);
        if (wait_result < 0) {
            if (errno == EINTR) {
                continue;
            }
            fprintf(stderr, "GPIO event wait failed: %s\n", strerror(errno));
            break;
        }

        if (wait_result > 0) {
            unsigned int offset;
            struct gpiod_line *line;
            gpiod_line_bulk_foreach_line_off(&events, line, offset) {
                struct gpiod_line_event event_buffer[32];
                int event_count = gpiod_line_event_read_multiple(line, event_buffer, 32);
                int line_index = find_input_for_line(line, line_inputs, lines, line_count);
                int event_index;

                if (event_count < 0 || line_index < 0) {
                    continue;
                }
                for (event_index = 0; event_index < event_count; event_index++) {
                    struct timespec realtime_now;
                    struct timespec monotonic_now;
                    int64_t event_mono_ns = timespec_to_ns(&event_buffer[event_index].ts);
                    int64_t realtime_ns;
                    int64_t mono_now_ns;
                    int64_t real_now_ns;
                    eventlogger_input_t *input = line_inputs[line_index];

                    if (!eventlogger_should_accept_event(input, event_mono_ns)) {
                        continue;
                    }
                    clock_gettime(CLOCK_REALTIME, &realtime_now);
                    clock_gettime(CLOCK_MONOTONIC, &monotonic_now);
                    real_now_ns = timespec_to_ns(&realtime_now);
                    mono_now_ns = timespec_to_ns(&monotonic_now);
                    realtime_ns = real_now_ns - (mono_now_ns - event_mono_ns);

                    if (eventlogger_journal_write_event(&journal, config,
                                                        realtime_ns, event_mono_ns,
                                                        input->name,
                                                        edge_name(event_buffer[event_index].event_type)) != 0) {
                        fprintf(stderr, "Failed to write event journal row\n");
                    }
                }
            }
        }

        now_s = time(NULL);
        if (config->status_interval_s > 0 && now_s - last_status >= config->status_interval_s) {
            eventlogger_chrony_status_t status;
            if (eventlogger_read_chrony_status(&status) == 0) {
                eventlogger_journal_write_status(&journal, config, &status);
            }
            last_status = now_s;
        }
        if (config->cleanup_check_interval_s > 0 &&
            now_s - last_cleanup >= config->cleanup_check_interval_s) {
            eventlogger_cleanup_if_needed(config, journal.event_path, journal.status_path);
            last_cleanup = now_s;
        }
        eventlogger_journal_maybe_flush(&journal, config, now_s, false);
    }

    eventlogger_journal_maybe_flush(&journal, config, time(NULL), true);
    eventlogger_journal_close(&journal);
    gpiod_line_release_bulk(&requested);
    gpiod_chip_close(chip);
    return 0;
}

int main(int argc, char **argv)
{
    const char *config_path = "/etc/neurokairos/eventlogger.conf";
    eventlogger_config_t config;
    int opt;

    while ((opt = getopt(argc, argv, "c:h")) != -1) {
        switch (opt) {
        case 'c':
            config_path = optarg;
            break;
        case 'h':
            fprintf(stderr, "Usage: %s [-c /path/to/eventlogger.conf]\n", argv[0]);
            return 0;
        default:
            return 1;
        }
    }

    if (eventlogger_config_load(config_path, &config) != 0) {
        fprintf(stderr, "Failed to load config %s\n", config_path);
        return 1;
    }

    signal(SIGINT, handle_signal);
    signal(SIGTERM, handle_signal);
    return run_event_loop(&config);
}
