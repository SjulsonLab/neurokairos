#include "eventlogger_core.h"

#include <ctype.h>
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static void trim(char *text)
{
    char *start = text;
    char *end;

    while (isspace((unsigned char)*start)) {
        start++;
    }
    if (start != text) {
        memmove(text, start, strlen(start) + 1);
    }

    end = text + strlen(text);
    while (end > text && isspace((unsigned char)*(end - 1))) {
        end--;
    }
    *end = '\0';
}

static void copy_string(char *dest, size_t dest_len, const char *src)
{
    size_t copy_len;

    if (dest_len == 0) {
        return;
    }
    copy_len = strlen(src);
    if (copy_len >= dest_len) {
        copy_len = dest_len - 1;
    }
    memcpy(dest, src, copy_len);
    dest[copy_len] = '\0';
}

static eventlogger_input_t *find_or_add_input(eventlogger_config_t *config, const char *name)
{
    int i;

    for (i = 0; i < config->input_count; i++) {
        if (strcmp(config->inputs[i].name, name) == 0) {
            return &config->inputs[i];
        }
    }

    if (config->input_count >= EVENTLOGGER_MAX_INPUTS) {
        return NULL;
    }

    eventlogger_input_t *input = &config->inputs[config->input_count++];
    memset(input, 0, sizeof(*input));
    copy_string(input->name, sizeof(input->name), name);
    input->gpio = -1;
    input->enabled = 1;
    input->deadtime_ms = 1;
    return input;
}

void eventlogger_config_init(eventlogger_config_t *config)
{
    /* Default DIN layout follows the Raspberry Pi header's board layout. */
    static const int default_gpios[] = {5, 6, 13, 19, 26, 16, 20, 21};
    int i;

    memset(config, 0, sizeof(*config));
    copy_string(config->chip, sizeof(config->chip), "gpiochip0");
    copy_string(config->config_path, sizeof(config->config_path), "/etc/neurokairos/eventlogger.conf");
    copy_string(config->journal_dir, sizeof(config->journal_dir), "/var/lib/neurokairos/eventlogger/journal");
    config->flush_interval_ms = 1000;
    config->flush_event_count = 1000;
    config->status_interval_s = 60;
    config->cleanup_check_interval_s = 60;
    config->cleanup_threshold_gb = 5.0;
    config->cleanup_target_gb = 8.0;
    config->input_count = 8;

    for (i = 0; i < config->input_count; i++) {
        snprintf(config->inputs[i].name, sizeof(config->inputs[i].name), "DIN%d", i + 1);
        config->inputs[i].gpio = default_gpios[i];
        config->inputs[i].enabled = 1;
        config->inputs[i].deadtime_ms = 1;
        config->inputs[i].last_event_monotonic_ns = 0;
    }
}

bool eventlogger_should_accept_event(eventlogger_input_t *input, int64_t monotonic_ns)
{
    int64_t deadtime_ns;

    if (input->deadtime_ms <= 0) {
        input->last_event_monotonic_ns = monotonic_ns;
        return true;
    }

    deadtime_ns = (int64_t)input->deadtime_ms * 1000LL * 1000LL;
    if (input->last_event_monotonic_ns != 0 &&
        monotonic_ns - input->last_event_monotonic_ns < deadtime_ns) {
        return false;
    }

    input->last_event_monotonic_ns = monotonic_ns;
    return true;
}

static int parse_bool(const char *value)
{
    if (strcmp(value, "1") == 0 || strcmp(value, "true") == 0 || strcmp(value, "yes") == 0 ||
        strcmp(value, "on") == 0) {
        return 1;
    }
    return 0;
}

static int apply_global(eventlogger_config_t *config, const char *key, const char *value)
{
    if (strcmp(key, "chip") == 0) {
        copy_string(config->chip, sizeof(config->chip), value);
    } else if (strcmp(key, "journal_dir") == 0) {
        copy_string(config->journal_dir, sizeof(config->journal_dir), value);
    } else if (strcmp(key, "flush_interval_ms") == 0) {
        config->flush_interval_ms = atoi(value);
    } else if (strcmp(key, "flush_event_count") == 0) {
        config->flush_event_count = atoi(value);
    } else if (strcmp(key, "status_interval_s") == 0) {
        config->status_interval_s = atoi(value);
    } else if (strcmp(key, "cleanup_check_interval_s") == 0) {
        config->cleanup_check_interval_s = atoi(value);
    } else if (strcmp(key, "cleanup_threshold_gb") == 0) {
        config->cleanup_threshold_gb = atof(value);
    } else if (strcmp(key, "cleanup_target_gb") == 0) {
        config->cleanup_target_gb = atof(value);
    } else if (strcmp(key, "deadtime_ms") == 0) {
        int i;
        int deadtime_ms = atoi(value);
        for (i = 0; i < config->input_count; i++) {
            config->inputs[i].deadtime_ms = deadtime_ms;
        }
    }
    return 0;
}

int eventlogger_config_load(const char *path, eventlogger_config_t *config)
{
    FILE *file;
    char line[512];
    char section[EVENTLOGGER_NAME_LEN] = "global";
    eventlogger_input_t *current_input = NULL;
    unsigned int line_number = 0;

    eventlogger_config_init(config);
    copy_string(config->config_path, sizeof(config->config_path), path);

    file = fopen(path, "r");
    if (!file) {
        return -1;
    }

    while (fgets(line, sizeof(line), file)) {
        char *comment;
        char *equals;
        char key[128];
        char value[384];

        line_number++;
        comment = strchr(line, '#');
        if (comment) {
            *comment = '\0';
        }
        trim(line);
        if (line[0] == '\0') {
            continue;
        }

        if (line[0] == '[') {
            char *end = strchr(line, ']');
            if (!end) {
                fprintf(stderr, "Invalid section at %s:%u\n", path, line_number);
                fclose(file);
                return -1;
            }
            *end = '\0';
            copy_string(section, sizeof(section), line + 1);
            trim(section);
            current_input = NULL;
            if (strncmp(section, "input ", 6) == 0) {
                current_input = find_or_add_input(config, section + 6);
                if (!current_input) {
                    fprintf(stderr, "Too many inputs at %s:%u\n", path, line_number);
                    fclose(file);
                    return -1;
                }
            }
            continue;
        }

        equals = strchr(line, '=');
        if (!equals) {
            fprintf(stderr, "Invalid key/value at %s:%u\n", path, line_number);
            fclose(file);
            return -1;
        }
        *equals = '\0';
        copy_string(key, sizeof(key), line);
        copy_string(value, sizeof(value), equals + 1);
        trim(key);
        trim(value);

        if (current_input) {
            if (strcmp(key, "gpio") == 0) {
                current_input->gpio = atoi(value);
            } else if (strcmp(key, "enabled") == 0) {
                current_input->enabled = parse_bool(value);
            } else if (strcmp(key, "deadtime_ms") == 0) {
                current_input->deadtime_ms = atoi(value);
            }
        } else {
            apply_global(config, key, value);
        }
    }

    fclose(file);
    return 0;
}
