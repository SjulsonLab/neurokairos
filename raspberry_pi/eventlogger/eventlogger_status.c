#include "eventlogger_core.h"

#include <ctype.h>
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

static double parse_first_double(const char *value)
{
    char *end = NULL;
    return strtod(value, &end);
}

static void set_utc_now(char *dest, size_t dest_len)
{
    struct timespec now;
    char date[11];

    if (clock_gettime(CLOCK_REALTIME, &now) != 0 ||
        eventlogger_format_utc_from_ns((int64_t)now.tv_sec * 1000000000LL + now.tv_nsec,
                                       date, sizeof(date), dest, dest_len) != 0) {
        snprintf(dest, dest_len, "1970-01-01T00:00:00.000000000Z");
    }
}

int eventlogger_read_chrony_status(eventlogger_chrony_status_t *status)
{
    FILE *pipe;
    char line[512];
    int parsed_any = 0;

    memset(status, 0, sizeof(*status));
    snprintf(status->reference_id, sizeof(status->reference_id), "unknown");
    snprintf(status->leap_status, sizeof(status->leap_status), "unknown");
    set_utc_now(status->timestamp_utc, sizeof(status->timestamp_utc));

    pipe = popen("chronyc tracking 2>/dev/null", "r");
    if (!pipe) {
        return -1;
    }

    while (fgets(line, sizeof(line), pipe)) {
        char *colon = strchr(line, ':');
        char key[128];
        char value[384];

        if (!colon) {
            continue;
        }
        *colon = '\0';
        snprintf(key, sizeof(key), "%s", line);
        snprintf(value, sizeof(value), "%s", colon + 1);
        trim(key);
        trim(value);

        if (strcmp(key, "Reference ID") == 0) {
            snprintf(status->reference_id, sizeof(status->reference_id), "%s", value);
            parsed_any = 1;
        } else if (strcmp(key, "Stratum") == 0) {
            status->stratum = atoi(value);
            parsed_any = 1;
        } else if (strcmp(key, "Leap status") == 0) {
            snprintf(status->leap_status, sizeof(status->leap_status), "%s", value);
            parsed_any = 1;
        } else if (strcmp(key, "System time") == 0) {
            status->system_time_s = parse_first_double(value);
            parsed_any = 1;
        } else if (strcmp(key, "Last offset") == 0) {
            status->last_offset_s = parse_first_double(value);
            parsed_any = 1;
        } else if (strcmp(key, "RMS offset") == 0) {
            status->rms_offset_s = parse_first_double(value);
            parsed_any = 1;
        } else if (strcmp(key, "Root delay") == 0) {
            status->root_delay_s = parse_first_double(value);
            parsed_any = 1;
        } else if (strcmp(key, "Root dispersion") == 0) {
            status->root_dispersion_s = parse_first_double(value);
            parsed_any = 1;
        } else if (strcmp(key, "Frequency") == 0) {
            status->frequency_ppm = parse_first_double(value);
            parsed_any = 1;
        } else if (strcmp(key, "Skew") == 0) {
            status->skew_ppm = parse_first_double(value);
            parsed_any = 1;
        }
    }

    if (pclose(pipe) == -1) {
        return -1;
    }
    return parsed_any ? 0 : -1;
}
