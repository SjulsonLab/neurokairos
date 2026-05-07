#define _POSIX_C_SOURCE 199309L
#define _GNU_SOURCE
#define _DEFAULT_SOURCE
#define _DARWIN_C_SOURCE

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>
#include <math.h>
#include <pthread.h>
#include <stdbool.h>
#include <signal.h>
#include <strings.h>
#include <sys/stat.h>
#include <sys/mman.h>
#include <fcntl.h>
#include <errno.h>
#include <sched.h>
#include <stdint.h>
#include <getopt.h>

// BCM283x/BCM2711 GPIO register base addresses (Pi 1-4)
#define BCM2708_PERI_BASE_RPI1  0x20000000
#define BCM2708_PERI_BASE_RPI2  0x3F000000
#define BCM2708_PERI_BASE_RPI4  0xFE000000
#define GPIO_BASE_OFFSET        0x200000

#define BLOCK_SIZE (4*1024)

// BCM283x/BCM2711 GPIO register offsets (Pi 1-4)
#define GPFSEL0   0x00
#define GPFSEL1   0x04
#define GPFSEL2   0x08
#define GPSET0    0x1C
#define GPCLR0    0x28
#define GPLEV0    0x34

// RP1 GPIO register layout for Pi 5.
// /dev/gpiomem0 maps three contiguous 64KB blocks:
//   IO_BANK0  (0x00000): per-pin status/ctrl registers, 8 bytes per pin
//   SYS_RIO0  (0x10000): fast register I/O with atomic SET/CLR variants
//   PADS_BANK0(0x20000): pad drive/pull configuration
#define RP1_GPIOMEM_SIZE        0x30000
#define RP1_IO_BANK0_OFFSET     0x00000
#define RP1_SYS_RIO0_OFFSET    0x10000
#define RP1_PADS_BANK0_OFFSET  0x20000

// Per-pin CTRL register in IO_BANK0 (bits 4:0 = FUNCSEL)
#define RP1_GPIO_CTRL(pin)     (RP1_IO_BANK0_OFFSET + (pin) * 8 + 4)
#define RP1_FSEL_SYS_RIO       5
#define RP1_FSEL_NULL          0x1f

// SYS_RIO0 register offsets (from SYS_RIO0 base)
#define RP1_RIO_OUT            0x00
#define RP1_RIO_OE             0x04
#define RP1_RIO_IN             0x08

// Atomic operation offsets (added to SYS_RIO0 base)
#define RP1_SET_OFFSET         0x2000
#define RP1_CLR_OFFSET         0x3000

// Pad control register per pin (bit 7 = OD output disable)
#define RP1_PADS_GPIO(pin)     (RP1_PADS_BANK0_OFFSET + 0x04 + (pin) * 4)

#ifndef MOCK_GPIO
static volatile unsigned *gpio_map = NULL;
static int gpio_mem_fd = -1;
static int pi_model_detected = 0;  // set by detect_pi_model(), used by gpio_* functions
#endif

typedef enum {
    IRIG_ZERO = 0,
    IRIG_ONE = 1,
    IRIG_P = 2
} irig_bit_t;

typedef struct {
    double *data;
    size_t length;
    size_t capacity;
} double_array_t;

#define SENDING_BIT_LENGTH 1.0
// Offset to account for pin toggle latency (tuned via oscilloscope).
// Pi 5 has a faster CPU (Cortex-A76) so less overhead before the register write.
#define OFFSET_NS_RPI4 20000
#define OFFSET_NS_RPI5  9500
// Sleep until this much time before target, then busy wait for precision.
// Must exceed worst-case nanosleep jitter (1-10ms on macOS/Linux without RT;
// <0.5ms on Linux with SCHED_FIFO). 10ms gives safe margin everywhere.
#define BUSY_WAIT_BUFFER_NS 10000000L
// Sleep interval during busy wait (0 = pure busy wait for maximum precision)
#define BUSY_WAIT_SLEEP_NS 0L

static const uint64_t NS_PER_SEC = 1000000000ULL;
static uint64_t bit_length_ns;
static uint64_t offset_ns;  // platform-dependent, set in init_timing_constants()

static const int SECONDS_WEIGHTS[] = {1, 2, 4, 8, 10, 20, 40};
static const int MINUTES_WEIGHTS[] = {1, 2, 4, 8, 10, 20, 40};
static const int HOURS_WEIGHTS[] = {1, 2, 4, 8, 10, 20};
static const int DAY_OF_YEAR_WEIGHTS[] = {1, 2, 4, 8, 10, 20, 40, 80, 100, 200};
static const int DECISECONDS_WEIGHTS[] = {1, 2, 4, 8};
static const int YEARS_WEIGHTS[] = {1, 2, 4, 8, 10, 20, 40, 80};

// --- Mock GPIO support --- //
#ifdef MOCK_GPIO
static uint32_t mock_gpio_regs[2];  // dummy SET and CLR registers
static FILE *mock_log = NULL;
static char mock_log_path[256] = "/tmp/irig_mock.csv";
#endif

static int max_frames = 0;  // 0 = unlimited
static char latency_log_path[256] = "";  // empty = no latency logging

typedef struct {
    int sending_gpio_pin;
    int inverted_gpio_pin;
    pthread_t sender_thread;
    bool running;
    double_array_t encoded_times;
    double_array_t sending_starts;
    char timestamp_filename[256];

    irig_bit_t current_frame[60];
    irig_bit_t next_frame[60];
    double pulse_lengths[60];
    uint64_t bit_start_times[60];
    struct timespec frame_start_time;
    volatile unsigned *gpio_set_reg;
    volatile unsigned *gpio_clr_reg;
    uint32_t gpio_mask;
    uint32_t inverted_gpio_mask;

    // Per-pulse latency logging (populated when --latency-log is set)
    int64_t *pulse_latency_ns;     // actual onset - ideal second boundary
    size_t pulse_count;
    size_t pulse_capacity;

    // Chrony sync status (polled every ~60 seconds)
    int chrony_stratum;            // 0 = not synced, 1-15 = NTP stratum
    double chrony_root_dispersion; // in seconds
    bool chrony_synced;            // true if leap status != "Not synchronised"
    double warn_threshold_ms;      // LED warning threshold (default 1.0)
    char led_original_trigger[64]; // saved LED trigger for cleanup
} irig_h_sender_t;

void init_timing_constants(void);
uint64_t timespec_to_ns(const struct timespec *ts);
void ultra_wait_until_ns(uint64_t target_ns);

volatile sig_atomic_t running = 1;
static int debug_mode = 0;

void signal_handler(int sig) {
    printf("Received signal %d, shutting down gracefully...\n", sig);
    running = 0;
}

// ── Hardware-specific functions ─────────────────────────────────────────────

#ifdef MOCK_GPIO

int detect_pi_model() { return 4; }

int gpio_init() { return 0; }

void gpio_cleanup() {}

void gpio_set_output(int pin) {
    (void)pin;
}

void gpio_write(int pin, int value) {
    (void)pin;
    (void)value;
}

void init_gpio_cache(irig_h_sender_t *sender) {
    sender->gpio_set_reg = (volatile unsigned *)&mock_gpio_regs[0];
    sender->gpio_clr_reg = (volatile unsigned *)&mock_gpio_regs[1];
    sender->gpio_mask = 1;
    sender->inverted_gpio_mask = 0;
}

#else  // !MOCK_GPIO

int detect_pi_model() {
    FILE *fp = fopen("/proc/device-tree/model", "r");
    if (!fp) return 2;

    char model[256];
    if (fgets(model, sizeof(model), fp)) {
        fclose(fp);
        if (strstr(model, "Raspberry Pi 5")) {
            return 5;
        } else if (strstr(model, "Raspberry Pi 4") || strstr(model, "Raspberry Pi 400")) {
            return 4;
        } else if (strstr(model, "Raspberry Pi 2") || strstr(model, "Raspberry Pi 3")) {
            return 2;
        } else {
            return 1;
        }
    }
    fclose(fp);
    return 2;
}

int gpio_init() {
    pi_model_detected = detect_pi_model();

    if (pi_model_detected == 5) {
        // Pi 5: RP1 GPIO via /dev/gpiomem0 (no root needed for mmap)
        printf("Detected Raspberry Pi 5, using RP1 GPIO via /dev/gpiomem0\n");

        gpio_mem_fd = open("/dev/gpiomem0", O_RDWR | O_SYNC);
        if (gpio_mem_fd < 0) {
            printf("Error: Cannot open /dev/gpiomem0: %s\n", strerror(errno));
            return -1;
        }

        gpio_map = (volatile unsigned *)mmap(
            NULL,
            RP1_GPIOMEM_SIZE,
            PROT_READ | PROT_WRITE,
            MAP_SHARED,
            gpio_mem_fd,
            0
        );
    } else {
        // Pi 1-4: BCM GPIO via /dev/mem
        unsigned gpio_base;
        switch (pi_model_detected) {
            case 1:
                gpio_base = BCM2708_PERI_BASE_RPI1 + GPIO_BASE_OFFSET;
                break;
            case 4:
                gpio_base = BCM2708_PERI_BASE_RPI4 + GPIO_BASE_OFFSET;
                break;
            default:
                gpio_base = BCM2708_PERI_BASE_RPI2 + GPIO_BASE_OFFSET;
                break;
        }

        printf("Detected Raspberry Pi model %d, using GPIO base 0x%08X\n",
               pi_model_detected, gpio_base);

        gpio_mem_fd = open("/dev/mem", O_RDWR | O_SYNC);
        if (gpio_mem_fd < 0) {
            printf("Error: Cannot open /dev/mem. Are you running as root?\n");
            return -1;
        }

        gpio_map = (volatile unsigned *)mmap(
            NULL,
            BLOCK_SIZE,
            PROT_READ | PROT_WRITE,
            MAP_SHARED,
            gpio_mem_fd,
            gpio_base
        );
    }

    if (gpio_map == MAP_FAILED) {
        printf("Error: mmap failed: %s\n", strerror(errno));
        close(gpio_mem_fd);
        return -1;
    }

    printf("GPIO memory mapped successfully\n");
    return 0;
}

void gpio_cleanup() {
    if (gpio_map != NULL && gpio_map != MAP_FAILED) {
        size_t map_size = (pi_model_detected == 5) ? RP1_GPIOMEM_SIZE : BLOCK_SIZE;
        munmap((void*)gpio_map, map_size);
        gpio_map = NULL;
    }
    if (gpio_mem_fd >= 0) {
        close(gpio_mem_fd);
        gpio_mem_fd = -1;
    }
}

void gpio_set_output(int pin) {
    if (gpio_map == NULL) return;

    if (pi_model_detected == 5) {
        // RP1: set FUNCSEL to SYS_RIO in the per-pin CTRL register
        volatile unsigned *ctrl = gpio_map + RP1_GPIO_CTRL(pin) / 4;
        *ctrl = (*ctrl & ~0x1f) | RP1_FSEL_SYS_RIO;

        // Ensure output driver is enabled (clear OD bit in pad register)
        volatile unsigned *pad = gpio_map + RP1_PADS_GPIO(pin) / 4;
        *pad &= ~(1 << 7);

        // Set output enable via RIO atomic SET register
        *(gpio_map + (RP1_SYS_RIO0_OFFSET + RP1_SET_OFFSET + RP1_RIO_OE) / 4) = (1 << pin);
    } else {
        // BCM (Pi 1-4): 3-bit function select, 10 pins per register
        int reg = pin / 10;
        int shift = (pin % 10) * 3;
        *(gpio_map + reg) &= ~(7 << shift);
        *(gpio_map + reg) |= (1 << shift);
    }

    printf("GPIO %d set as output\n", pin);
}

void gpio_write(int pin, int value) {
    if (gpio_map == NULL) return;

    if (pi_model_detected == 5) {
        // RP1: use SYS_RIO0 atomic SET/CLR for the OUT register
        if (value) {
            *(gpio_map + (RP1_SYS_RIO0_OFFSET + RP1_SET_OFFSET + RP1_RIO_OUT) / 4) = 1 << pin;
        } else {
            *(gpio_map + (RP1_SYS_RIO0_OFFSET + RP1_CLR_OFFSET + RP1_RIO_OUT) / 4) = 1 << pin;
        }
    } else {
        // BCM (Pi 1-4): GPSET0/GPCLR0
        if (value) {
            *(gpio_map + GPSET0/4) = 1 << pin;
        } else {
            *(gpio_map + GPCLR0/4) = 1 << pin;
        }
    }
}

// Cache GPIO registers for fast access.
// When a pin is -1 (disabled), its mask is set to 0. Writing mask=0 to
// SET/CLR registers is a hardware no-op on both BCM and RP1, so no guards
// are needed in the hot path (ultra_fast_pulse).
void init_gpio_cache(irig_h_sender_t *sender) {
    if (gpio_map == NULL) return;

    if (pi_model_detected == 5) {
        // RP1: SYS_RIO0 atomic SET/CLR registers for OUT
        sender->gpio_set_reg = gpio_map + (RP1_SYS_RIO0_OFFSET + RP1_SET_OFFSET + RP1_RIO_OUT) / 4;
        sender->gpio_clr_reg = gpio_map + (RP1_SYS_RIO0_OFFSET + RP1_CLR_OFFSET + RP1_RIO_OUT) / 4;
    } else {
        // BCM (Pi 1-4): GPSET0/GPCLR0
        sender->gpio_set_reg = gpio_map + GPSET0/4;
        sender->gpio_clr_reg = gpio_map + GPCLR0/4;
    }

    sender->gpio_mask = (sender->sending_gpio_pin >= 0) ? (1 << sender->sending_gpio_pin) : 0;
    sender->inverted_gpio_mask = (sender->inverted_gpio_pin >= 0) ? (1 << sender->inverted_gpio_pin) : 0;
}

#endif  // MOCK_GPIO

double_array_t* create_double_array(size_t initial_capacity) {
    double_array_t *arr = malloc(sizeof(double_array_t));
    arr->data = malloc(sizeof(double) * initial_capacity);
    arr->length = 0;
    arr->capacity = initial_capacity;
    return arr;
}

void append_double(double_array_t *arr, double value) {
    if (arr->length >= arr->capacity) {
        arr->capacity *= 2;
        arr->data = realloc(arr->data, sizeof(double) * arr->capacity);
    }
    arr->data[arr->length++] = value;
}

void free_double_array(double_array_t *arr) {
    if (arr) {
        free(arr->data);
        free(arr);
    }
}

void bcd_encode(int value, const int *weights, int weight_count, int *result) {
    memset(result, 0, weight_count * sizeof(int));
    for (int i = weight_count - 1; i >= 0; i--) {
        if (weights[i] <= value) {
            result[i] = 1;
            value -= weights[i];
        }
    }
}

// --- Chrony status polling --- //

// Encode stratum into 2-bit field: 1->0, 2->1, 3->2, >=4 or 0->3
int encode_stratum(int stratum) {
    if (stratum == 1) return 0;
    if (stratum == 2) return 1;
    if (stratum == 3) return 2;
    return 3;  // stratum 4+ or not synchronized (0)
}

// Encode root dispersion (in seconds) into 3-bit bucket (0-7).
// Bucket boundaries double: 0.25, 0.5, 1, 2, 4, 8, 16 ms
int encode_root_dispersion(double dispersion_sec) {
    double dispersion_ms = dispersion_sec * 1000.0;
    if (dispersion_ms < 0.25)  return 0;
    if (dispersion_ms < 0.5)   return 1;
    if (dispersion_ms < 1.0)   return 2;
    if (dispersion_ms < 2.0)   return 3;
    if (dispersion_ms < 4.0)   return 4;
    if (dispersion_ms < 8.0)   return 5;
    if (dispersion_ms < 16.0)  return 6;
    return 7;  // >= 16 ms or not synchronized
}

static char *trim_whitespace(char *s) {
    while (*s == ' ' || *s == '\t' || *s == '\r' || *s == '\n') {
        s++;
    }

    char *end = s + strlen(s);
    while (end > s &&
           (end[-1] == ' ' || end[-1] == '\t' ||
            end[-1] == '\r' || end[-1] == '\n')) {
        end--;
    }
    *end = '\0';
    return s;
}

static bool parse_int_field(const char *value, int *out) {
    errno = 0;
    char *endptr = NULL;
    long parsed = strtol(value, &endptr, 10);
    if (endptr == value || errno != 0) {
        return false;
    }
    *out = (int)parsed;
    return true;
}

static bool parse_double_field(const char *value, double *out) {
    errno = 0;
    char *endptr = NULL;
    double parsed = strtod(value, &endptr);
    if (endptr == value || errno != 0 || !isfinite(parsed)) {
        return false;
    }
    *out = parsed;
    return true;
}

static bool parse_leap_status(const char *value, bool *synced) {
    // Labeled chrony output uses text; CSV output from some versions uses 0-3.
    char *endptr = NULL;
    long numeric = strtol(value, &endptr, 10);
    if (endptr != value) {
        *synced = (numeric != 3);
        return true;
    }

    if (strcasecmp(value, "Normal") == 0 ||
        strncasecmp(value, "Insert", 6) == 0 ||
        strncasecmp(value, "Delete", 6) == 0) {
        *synced = true;
        return true;
    }

    if (strncasecmp(value, "Not synchronised", 16) == 0 ||
        strncasecmp(value, "Not synchronized", 16) == 0) {
        *synced = false;
        return true;
    }

    return false;
}

#ifdef MOCK_GPIO

// In mock mode, report stratum 1 / excellent sync (no chronyc available)
void poll_chrony_status(irig_h_sender_t *sender) {
    sender->chrony_stratum = 1;
    sender->chrony_root_dispersion = 0.0001;  // 0.1 ms
    sender->chrony_synced = true;
}

#else  // !MOCK_GPIO

// Poll chrony for current sync status via `chronyc tracking`.
// Parse labeled fields instead of positional CSV columns so this keeps working
// across chrony versions that add/remove tracking fields.
// On any failure (chronyc missing, chrony not running), sets safe defaults.
void poll_chrony_status(irig_h_sender_t *sender) {
    // Use timeout(1) to bound chronyc runtime; a hung chronyc would otherwise
    // block fgets indefinitely and delay the next frame by >60 s.
    FILE *fp = popen("LC_ALL=C timeout 5 chronyc tracking 2>/dev/null", "r");
    if (!fp) {
        sender->chrony_stratum = 0;
        sender->chrony_root_dispersion = 1.0;  // 1 second = very bad
        sender->chrony_synced = false;
        printf("Chrony poll: chronyc not available\n");
        return;
    }

    char line[1024];
    int new_stratum = 0;
    double new_dispersion = 1.0;
    bool new_synced = false;
    bool saw_output = false;
    bool saw_stratum = false;
    bool saw_dispersion = false;
    bool saw_leap_status = false;

    while (fgets(line, sizeof(line), fp) != NULL) {
        saw_output = true;

        char *colon = strchr(line, ':');
        if (!colon) {
            continue;
        }

        *colon = '\0';
        char *label = trim_whitespace(line);
        char *value = trim_whitespace(colon + 1);

        if (strcasecmp(label, "Stratum") == 0) {
            saw_stratum = parse_int_field(value, &new_stratum);
        } else if (strcasecmp(label, "Root dispersion") == 0) {
            saw_dispersion = parse_double_field(value, &new_dispersion);
        } else if (strcasecmp(label, "Leap status") == 0) {
            saw_leap_status = parse_leap_status(value, &new_synced);
        }
    }
    pclose(fp);

    if (!saw_output) {
        sender->chrony_stratum = 0;
        sender->chrony_root_dispersion = 1.0;
        sender->chrony_synced = false;
        printf("Chrony poll: no output from chronyc\n");
        return;
    }

    if (!saw_stratum || !saw_dispersion || !saw_leap_status) {
        sender->chrony_stratum = 0;
        sender->chrony_root_dispersion = 1.0;
        sender->chrony_synced = false;
        printf("Chrony poll: incomplete chronyc output "
               "(stratum=%s root_dispersion=%s leap_status=%s)\n",
               saw_stratum ? "yes" : "no",
               saw_dispersion ? "yes" : "no",
               saw_leap_status ? "yes" : "no");
        return;
    }

    // If not synchronized, override stratum to 0
    if (!new_synced) {
        new_stratum = 0;
    }

    // Log changes
    if (new_stratum != sender->chrony_stratum ||
        new_synced != sender->chrony_synced) {
        printf("Chrony status changed: stratum=%d synced=%s root_dispersion=%.6f s\n",
               new_stratum, new_synced ? "yes" : "no", new_dispersion);
    }

    sender->chrony_stratum = new_stratum;
    sender->chrony_root_dispersion = new_dispersion;
    sender->chrony_synced = new_synced;
}

#endif  // MOCK_GPIO

// --- LED control via sysfs --- //

#ifdef MOCK_GPIO

void led_init(irig_h_sender_t *sender) {
    sender->led_original_trigger[0] = '\0';
}

void led_set_solid(void) {}

void led_set_blink(int delay_on_ms, int delay_off_ms) {
    (void)delay_on_ms; (void)delay_off_ms;
}

void led_cleanup(irig_h_sender_t *sender) {
    (void)sender;
}

void update_led_status(irig_h_sender_t *sender) {
    (void)sender;
}

#else  // !MOCK_GPIO

static const char *LED_TRIGGER_PATH = "/sys/class/leds/ACT/trigger";
static const char *LED_BRIGHTNESS_PATH = "/sys/class/leds/ACT/brightness";
static const char *LED_DELAY_ON_PATH = "/sys/class/leds/ACT/delay_on";
static const char *LED_DELAY_OFF_PATH = "/sys/class/leds/ACT/delay_off";

static void sysfs_write(const char *path, const char *value) {
    FILE *fp = fopen(path, "w");
    if (fp) {
        fprintf(fp, "%s", value);
        fclose(fp);
    }
}

static void sysfs_read(const char *path, char *buf, size_t len) {
    buf[0] = '\0';
    FILE *fp = fopen(path, "r");
    if (fp) {
        // The trigger file shows [active] with brackets; read current value
        char raw[256];
        if (fgets(raw, sizeof(raw), fp)) {
            // Find the [bracketed] active trigger
            char *start = strchr(raw, '[');
            char *end = start ? strchr(start, ']') : NULL;
            if (start && end) {
                size_t n = (size_t)(end - start - 1);
                if (n >= len) n = len - 1;
                memcpy(buf, start + 1, n);
                buf[n] = '\0';
            } else {
                // Fallback: just copy first token
                strncpy(buf, raw, len - 1);
                buf[len - 1] = '\0';
                // Remove trailing newline
                char *nl = strchr(buf, '\n');
                if (nl) *nl = '\0';
            }
        }
        fclose(fp);
    }
}

void led_init(irig_h_sender_t *sender) {
    // Save current trigger so we can restore on cleanup
    sysfs_read(LED_TRIGGER_PATH, sender->led_original_trigger,
               sizeof(sender->led_original_trigger));
    if (sender->led_original_trigger[0] != '\0') {
        printf("LED: saved original trigger '%s'\n", sender->led_original_trigger);
    }
    // Take control: disable current trigger
    sysfs_write(LED_TRIGGER_PATH, "none");
}

void led_set_solid(void) {
    sysfs_write(LED_TRIGGER_PATH, "none");
    sysfs_write(LED_BRIGHTNESS_PATH, "1");
}

void led_set_blink(int delay_on_ms, int delay_off_ms) {
    sysfs_write(LED_TRIGGER_PATH, "timer");
    char buf[16];
    snprintf(buf, sizeof(buf), "%d", delay_on_ms);
    sysfs_write(LED_DELAY_ON_PATH, buf);
    snprintf(buf, sizeof(buf), "%d", delay_off_ms);
    sysfs_write(LED_DELAY_OFF_PATH, buf);
}

void led_cleanup(irig_h_sender_t *sender) {
    if (sender->led_original_trigger[0] != '\0') {
        printf("LED: restoring trigger '%s'\n", sender->led_original_trigger);
        sysfs_write(LED_TRIGGER_PATH, sender->led_original_trigger);
    }
}

void update_led_status(irig_h_sender_t *sender) {
    double dispersion_ms = sender->chrony_root_dispersion * 1000.0;
    if (!sender->chrony_synced || dispersion_ms >= sender->warn_threshold_ms) {
        led_set_blink(500, 500);
    } else {
        led_set_solid();
    }
}

#endif  // MOCK_GPIO

void generate_irig_h_frame(irig_h_sender_t *sender, struct tm *time_info, irig_bit_t *frame) {
    append_double(&sender->encoded_times, (double)mktime(time_info));

    int seconds_bcd[7], minutes_bcd[7], hours_bcd[6];
    int day_of_year_bcd[10], deciseconds_bcd[4], year_bcd[8];

    bcd_encode(time_info->tm_sec, SECONDS_WEIGHTS, 7, seconds_bcd);
    bcd_encode(time_info->tm_min, MINUTES_WEIGHTS, 7, minutes_bcd);
    bcd_encode(time_info->tm_hour, HOURS_WEIGHTS, 6, hours_bcd);
    bcd_encode(time_info->tm_yday + 1, DAY_OF_YEAR_WEIGHTS, 10, day_of_year_bcd);
    bcd_encode(0, DECISECONDS_WEIGHTS, 4, deciseconds_bcd);
    bcd_encode((time_info->tm_year + 1900) % 100, YEARS_WEIGHTS, 8, year_bcd);

    int pos = 0;
    frame[pos++] = IRIG_P;

    for (int i = 0; i < 4; i++) frame[pos++] = seconds_bcd[i] ? IRIG_ONE : IRIG_ZERO;
    frame[pos++] = IRIG_ZERO;
    for (int i = 4; i < 7; i++) frame[pos++] = seconds_bcd[i] ? IRIG_ONE : IRIG_ZERO;
    frame[pos++] = IRIG_P;

    for (int i = 0; i < 4; i++) frame[pos++] = minutes_bcd[i] ? IRIG_ONE : IRIG_ZERO;
    frame[pos++] = IRIG_ZERO;
    for (int i = 4; i < 7; i++) frame[pos++] = minutes_bcd[i] ? IRIG_ONE : IRIG_ZERO;
    frame[pos++] = IRIG_ZERO;
    frame[pos++] = IRIG_P;

    for (int i = 0; i < 4; i++) frame[pos++] = hours_bcd[i] ? IRIG_ONE : IRIG_ZERO;
    frame[pos++] = IRIG_ZERO;
    for (int i = 4; i < 6; i++) frame[pos++] = hours_bcd[i] ? IRIG_ONE : IRIG_ZERO;
    frame[pos++] = IRIG_ZERO; frame[pos++] = IRIG_ZERO;
    frame[pos++] = IRIG_P;

    for (int i = 0; i < 4; i++) frame[pos++] = day_of_year_bcd[i] ? IRIG_ONE : IRIG_ZERO;
    frame[pos++] = IRIG_ZERO;
    for (int i = 4; i < 8; i++) frame[pos++] = day_of_year_bcd[i] ? IRIG_ONE : IRIG_ZERO;
    frame[pos++] = IRIG_P;
    for (int i = 8; i < 10; i++) frame[pos++] = day_of_year_bcd[i] ? IRIG_ONE : IRIG_ZERO;

    frame[pos++] = IRIG_ZERO; frame[pos++] = IRIG_ZERO; frame[pos++] = IRIG_ZERO;
    for (int i = 0; i < 4; i++) frame[pos++] = deciseconds_bcd[i] ? IRIG_ONE : IRIG_ZERO;
    frame[pos++] = IRIG_P;

    for (int i = 0; i < 4; i++) frame[pos++] = year_bcd[i] ? IRIG_ONE : IRIG_ZERO;
    frame[pos++] = IRIG_ZERO;
    for (int i = 4; i < 8; i++) frame[pos++] = year_bcd[i] ? IRIG_ONE : IRIG_ZERO;
    frame[pos++] = IRIG_P;

    // Overwrite sync status bits (NeuroKairos extension).
    // Bits 42, 45 remain zero (reserved).
    // Bits 43-44: encoded stratum (2 bits)
    int stratum_enc = encode_stratum(sender->chrony_stratum);
    frame[43] = (stratum_enc & 1) ? IRIG_ONE : IRIG_ZERO;
    frame[44] = (stratum_enc & 2) ? IRIG_ONE : IRIG_ZERO;

    // Bits 46-48: encoded root dispersion bucket (3 bits)
    int disp_enc = encode_root_dispersion(sender->chrony_root_dispersion);
    frame[46] = (disp_enc & 1) ? IRIG_ONE : IRIG_ZERO;
    frame[47] = (disp_enc & 2) ? IRIG_ONE : IRIG_ZERO;
    frame[48] = (disp_enc & 4) ? IRIG_ONE : IRIG_ZERO;
}

void init_timing_constants(void) {
    bit_length_ns = (uint64_t)(SENDING_BIT_LENGTH * NS_PER_SEC);
#ifndef MOCK_GPIO
    offset_ns = (pi_model_detected == 5) ? OFFSET_NS_RPI5 : OFFSET_NS_RPI4;
#else
    offset_ns = OFFSET_NS_RPI4;
#endif
    printf("Pin toggle offset: %llu ns\n", (unsigned long long)offset_ns);
}

uint64_t timespec_to_ns(const struct timespec *ts) {
    return (uint64_t)ts->tv_sec * NS_PER_SEC + (uint64_t)ts->tv_nsec;
}

// Wait until target_ns (CLOCK_MONOTONIC nanoseconds).
// Uses hybrid sleep/busy-wait: sleeps until BUSY_WAIT_BUFFER_NS before
// target, then spins for precision.
void ultra_wait_until_ns(uint64_t target_ns) {
    struct timespec current_time;
    uint64_t current_ns;
    int64_t remaining_ns;
    struct timespec sleep_time;

    // Cap individual nanosleep calls to limit overshoot variance.
    // Long sleeps (seconds+) can have 10ms+ jitter on macOS/Linux;
    // short sleeps (<100ms) typically overshoot by <5ms.
    static const uint64_t MAX_SLEEP_NS = 100000000ULL;  // 100ms

    while (running) {
        clock_gettime(CLOCK_MONOTONIC, &current_time);
        current_ns = timespec_to_ns(&current_time);
        remaining_ns = (int64_t)(target_ns - current_ns);

        if (remaining_ns <= 0) break;

        if (remaining_ns > BUSY_WAIT_BUFFER_NS) {
            uint64_t sleep_duration = remaining_ns - BUSY_WAIT_BUFFER_NS;
            if (sleep_duration > MAX_SLEEP_NS)
                sleep_duration = MAX_SLEEP_NS;

            sleep_time.tv_sec = sleep_duration / NS_PER_SEC;
            sleep_time.tv_nsec = sleep_duration % NS_PER_SEC;
            nanosleep(&sleep_time, NULL);
        } else {
            break;
        }
    }

    #if BUSY_WAIT_SLEEP_NS > 0
        sleep_time.tv_sec = 0;
        sleep_time.tv_nsec = BUSY_WAIT_SLEEP_NS;
        do {
            clock_gettime(CLOCK_MONOTONIC, &current_time);
            current_ns = (uint64_t)current_time.tv_sec * NS_PER_SEC + (uint64_t)current_time.tv_nsec;
            if (current_ns < target_ns) {
                nanosleep(&sleep_time, NULL);
            }
        } while (current_ns < target_ns && running);
    #else
        do {
            clock_gettime(CLOCK_MONOTONIC, &current_time);
            current_ns = (uint64_t)current_time.tv_sec * NS_PER_SEC + (uint64_t)current_time.tv_nsec;
        } while (current_ns < target_ns && running);
    #endif
}

double calculate_pulse_length(irig_bit_t bit) {
    switch (bit) {
        case IRIG_P: return 0.8 * SENDING_BIT_LENGTH;
        case IRIG_ONE: return 0.5 * SENDING_BIT_LENGTH;
        case IRIG_ZERO:
        default: return 0.2 * SENDING_BIT_LENGTH;
    }
}

// Generate a single pulse: set GPIO high, wait for pulse_duration_ns, set low.
// Uses CLOCK_MONOTONIC for timing (immune to chrony adjustments).
void ultra_fast_pulse(irig_h_sender_t *sender, uint64_t pulse_duration_ns) {
    struct timespec start_time, current_time, sleep_time;
    uint64_t start_ns, target_ns, current_ns;
    int64_t remaining_ns;

    clock_gettime(CLOCK_MONOTONIC, &start_time);
    start_ns = timespec_to_ns(&start_time);
    target_ns = start_ns + pulse_duration_ns;

    *(sender->gpio_set_reg) = sender->gpio_mask;
    *(sender->gpio_clr_reg) = sender->inverted_gpio_mask;

    static const uint64_t MAX_SLEEP_NS = 100000000ULL;  // 100ms

    while (running) {
        clock_gettime(CLOCK_MONOTONIC, &current_time);
        current_ns = timespec_to_ns(&current_time);
        remaining_ns = (int64_t)(target_ns - current_ns);

        if (remaining_ns <= 0) break;

        if (remaining_ns > BUSY_WAIT_BUFFER_NS) {
            uint64_t sleep_duration = remaining_ns - BUSY_WAIT_BUFFER_NS;
            if (sleep_duration > MAX_SLEEP_NS)
                sleep_duration = MAX_SLEEP_NS;
            sleep_time.tv_sec = sleep_duration / NS_PER_SEC;
            sleep_time.tv_nsec = sleep_duration % NS_PER_SEC;
            nanosleep(&sleep_time, NULL);
        } else {
            break;
        }
    }

    #if BUSY_WAIT_SLEEP_NS > 0
        struct timespec poll_sleep;
        poll_sleep.tv_sec = 0;
        poll_sleep.tv_nsec = BUSY_WAIT_SLEEP_NS;

        do {
            clock_gettime(CLOCK_MONOTONIC, &current_time);
            current_ns = (uint64_t)current_time.tv_sec * NS_PER_SEC + (uint64_t)current_time.tv_nsec;
            if (current_ns < target_ns) {
                nanosleep(&poll_sleep, NULL);
            }
        } while (current_ns < target_ns && running);
    #else
        do {
            clock_gettime(CLOCK_MONOTONIC, &current_time);
            current_ns = (uint64_t)current_time.tv_sec * NS_PER_SEC + (uint64_t)current_time.tv_nsec;
        } while (current_ns < target_ns && running);
    #endif

    *(sender->gpio_clr_reg) = sender->gpio_mask;
    *(sender->gpio_set_reg) = sender->inverted_gpio_mask;

#ifdef MOCK_GPIO
    if (mock_log) {
        struct timespec falling_ts;
        clock_gettime(CLOCK_REALTIME, &falling_ts);
        fprintf(mock_log, "%llu,%llu\n",
                (unsigned long long)start_ns,
                (unsigned long long)timespec_to_ns(&falling_ts));
    }
#endif
}

// Pre-calculate timing for the next frame
void precalculate_next_frame(irig_h_sender_t *sender, time_t target_second) {
    struct tm *time_info = gmtime(&target_second);
    generate_irig_h_frame(sender, time_info, sender->next_frame);

    uint64_t frame_start_ns = (uint64_t)target_second * NS_PER_SEC;
    for (int i = 0; i < 60; i++) {
        sender->pulse_lengths[i] = calculate_pulse_length(sender->next_frame[i]);
        sender->bit_start_times[i] = frame_start_ns + (i * NS_PER_SEC) - offset_ns;
    }
}

void* continuous_irig_sending(void *arg) {
    irig_h_sender_t *sender = (irig_h_sender_t*)arg;
    struct timespec current_time;
    struct timespec mono_ts, real_ts;
    int64_t rt_to_mono_offset_ns;
    time_t next_frame_time;
    int frames_sent = 0;

    static int constants_initialized = 0;
    if (!constants_initialized) {
        init_timing_constants();
        constants_initialized = 1;
    }

#ifndef MOCK_GPIO
    struct sched_param param;
    param.sched_priority = 80;
    if (sched_setscheduler(0, SCHED_FIFO, &param) < 0) {
        printf("Warning: Could not set SCHED_FIFO: %s (continuing with normal scheduling)\n", strerror(errno));
    } else {
        printf("Set SCHED_FIFO with priority 80\n");
    }

    // Lock all pages in RAM to prevent page faults during timing-critical path
    if (mlockall(MCL_CURRENT | MCL_FUTURE) < 0) {
        printf("Warning: Could not lock memory: %s (continuing without mlockall)\n", strerror(errno));
    } else {
        printf("Memory locked with mlockall(MCL_CURRENT | MCL_FUTURE)\n");
    }

#endif

    printf("IRIG-H continuous transmission thread started\n");

    // Poll chrony once at startup so the first frame has valid status bits
    poll_chrony_status(sender);
    update_led_status(sender);

    // Align first frame to next minute boundary so BCD seconds field is
    // always 0 and frames span :00 to :59.
    clock_gettime(CLOCK_REALTIME, &current_time);
    next_frame_time = ((current_time.tv_sec / 60) + 1) * 60;
    precalculate_next_frame(sender, next_frame_time);

    printf("Waiting for next minute boundary (%ld seconds)...\n",
           (long)(next_frame_time - current_time.tv_sec));

    while (sender->running && running) {
        memcpy(sender->current_frame, sender->next_frame, sizeof(sender->current_frame));

        for (int i = 0; i < 60 && sender->running && running; i++) {
            // Convert realtime target to monotonic (recomputed each bit
            // to handle chrony slew/step of CLOCK_REALTIME)
            clock_gettime(CLOCK_REALTIME, &real_ts);
            clock_gettime(CLOCK_MONOTONIC, &mono_ts);
            rt_to_mono_offset_ns = (int64_t)timespec_to_ns(&mono_ts)
                                 - (int64_t)timespec_to_ns(&real_ts);
            ultra_wait_until_ns(sender->bit_start_times[i] + rt_to_mono_offset_ns);

            if (!sender->running || !running) break;

            if (i == 0) {
                // Record actual start time for diagnostics
                struct timespec start_ts;
                clock_gettime(CLOCK_REALTIME, &start_ts);
                double start_time_double = (double)start_ts.tv_sec + (double)start_ts.tv_nsec * 1e-9;
                append_double(&sender->sending_starts, start_time_double);
            }

            // Record per-pulse latency if logging is enabled
            if (sender->pulse_latency_ns) {
                struct timespec onset_ts;
                clock_gettime(CLOCK_REALTIME, &onset_ts);
                uint64_t actual_ns = timespec_to_ns(&onset_ts);
                // ideal second boundary = bit_start_times[i] + offset_ns
                uint64_t ideal_ns = sender->bit_start_times[i] + offset_ns;
                int64_t latency = (int64_t)(actual_ns - ideal_ns);
                if (sender->pulse_count < sender->pulse_capacity) {
                    sender->pulse_latency_ns[sender->pulse_count++] = latency;
                }
            }

            if (debug_mode) {
                struct timespec current;
                clock_gettime(CLOCK_REALTIME, &current);
                printf("Bit %d sent at system time: %ld.%09ld\n", i, current.tv_sec, current.tv_nsec);
            }

            uint64_t pulse_ns = (uint64_t)(sender->pulse_lengths[i] * NS_PER_SEC);
            ultra_fast_pulse(sender, pulse_ns);
        }

        // Check frame limit
        frames_sent++;
        if (max_frames > 0 && frames_sent >= max_frames) {
            printf("Completed %d frame(s), exiting.\n", frames_sent);
            running = 0;
            break;
        }

        // Poll chrony after each full 60-bit frame (~every 60 seconds)
        poll_chrony_status(sender);
        update_led_status(sender);

        // Prepare next frame at next minute boundary
        next_frame_time += 60;
        precalculate_next_frame(sender, next_frame_time);
    }

#ifdef MOCK_GPIO
    if (mock_log) {
        fflush(mock_log);
    }
#endif

    printf("IRIG-H transmission thread stopping\n");
    return NULL;
}

irig_h_sender_t* create_irig_h_sender(int gpio_pin, int inverted_gpio_pin) {
    irig_h_sender_t *sender = malloc(sizeof(irig_h_sender_t));

    sender->sending_gpio_pin = gpio_pin;
    sender->inverted_gpio_pin = inverted_gpio_pin;
    sender->running = false;

    sender->encoded_times.data = malloc(sizeof(double) * 100);
    sender->encoded_times.length = 0;
    sender->encoded_times.capacity = 100;

    sender->sending_starts.data = malloc(sizeof(double) * 100);
    sender->sending_starts.length = 0;
    sender->sending_starts.capacity = 100;

    // Initialize per-pulse latency logging
    if (latency_log_path[0] != '\0') {
        // Default capacity: 6000 pulses (100 frames * 60 bits)
        sender->pulse_capacity = 6000;
        sender->pulse_latency_ns = malloc(sizeof(int64_t) * sender->pulse_capacity);
        sender->pulse_count = 0;
    } else {
        sender->pulse_latency_ns = NULL;
        sender->pulse_count = 0;
        sender->pulse_capacity = 0;
    }

    // Initialize chrony status to safe defaults (unsynchronized)
    sender->chrony_stratum = 0;
    sender->chrony_root_dispersion = 1.0;
    sender->chrony_synced = false;
    sender->warn_threshold_ms = 1.0;
    sender->led_original_trigger[0] = '\0';

    time_t now = time(NULL);
    struct tm *tm_info = localtime(&now); // localtime is intentional here -- just for the output filename
    strftime(sender->timestamp_filename, sizeof(sender->timestamp_filename),
             "irig_output_timestamps_%Y-%m-%d_%H-%M-%S.csv", tm_info);

    if (gpio_init() < 0) {
        printf("Failed to initialize GPIO hardware access\n");
        free(sender->encoded_times.data);
        free(sender->sending_starts.data);
        free(sender);
        return NULL;
    }

    // Only configure pins that are enabled (>= 0); -1 means disabled
    if (sender->sending_gpio_pin >= 0) {
        gpio_set_output(sender->sending_gpio_pin);
        gpio_write(sender->sending_gpio_pin, 0);
    }
    if (sender->inverted_gpio_pin >= 0) {
        gpio_set_output(sender->inverted_gpio_pin);
        gpio_write(sender->inverted_gpio_pin, 1);
    }

    init_gpio_cache(sender);
    led_init(sender);

    return sender;
}

void start_irig_sender(irig_h_sender_t *sender) {
    sender->running = true;
    pthread_create(&sender->sender_thread, NULL, continuous_irig_sending, sender);
}

void write_timestamps_to_file(irig_h_sender_t *sender) {
    FILE *file = fopen(sender->timestamp_filename, "w");
    if (!file) {
        printf("Could not open file for writing: %s\n", sender->timestamp_filename);
        return;
    }

    fprintf(file, "Encoded times,Sending starts\n");
    size_t min_length = (sender->encoded_times.length < sender->sending_starts.length)
                       ? sender->encoded_times.length : sender->sending_starts.length;

    for (size_t i = 0; i < min_length; i++) {
        fprintf(file, "%f,%f\n", sender->encoded_times.data[i], sender->sending_starts.data[i]);
    }

    fclose(file);
    printf("Timestamps written to %s\n", sender->timestamp_filename);
}

void write_latency_log(irig_h_sender_t *sender) {
    if (!sender->pulse_latency_ns || latency_log_path[0] == '\0') return;

    FILE *fp = fopen(latency_log_path, "w");
    if (!fp) {
        printf("Could not open latency log: %s\n", latency_log_path);
        return;
    }

    fprintf(fp, "pulse_index,latency_ns\n");
    for (size_t i = 0; i < sender->pulse_count; i++) {
        fprintf(fp, "%zu,%lld\n", i, (long long)sender->pulse_latency_ns[i]);
    }

    fclose(fp);
    printf("Latency log written to %s (%zu pulses)\n",
           latency_log_path, sender->pulse_count);
}

void finish_irig_sender(irig_h_sender_t *sender) {
    sender->running = false;
    pthread_join(sender->sender_thread, NULL);

    // Write latency log before cleanup
    write_latency_log(sender);

    // Reset enabled pins to idle state before cleanup
    if (sender->sending_gpio_pin >= 0)
        gpio_write(sender->sending_gpio_pin, 0);
    if (sender->inverted_gpio_pin >= 0)
        gpio_write(sender->inverted_gpio_pin, 1);

    led_cleanup(sender);
    gpio_cleanup();

    free(sender->encoded_times.data);
    free(sender->sending_starts.data);
    free(sender->pulse_latency_ns);
    free(sender);
}

void print_usage(const char *prog_name) {
    printf("Usage: %s [-p PIN] [-n PIN] [-w THRESHOLD] [-d] [-h] [--frames N] [--latency-log FILE]\n", prog_name);
    printf("  -p PIN           BCM GPIO pin for normal output (default: 9, -1 to disable)\n");
    printf("  -n PIN           BCM GPIO pin for inverted output (default: -1, disabled)\n");
    printf("  -w THRESHOLD     LED warning threshold in ms (default: 1.0)\n");
    printf("                   LED blinks when root dispersion exceeds this value\n");
    printf("  -d               Enable debug mode\n");
    printf("  -h               Print this help message and exit\n");
    printf("  --frames N       Exit after N frames (default: unlimited)\n");
    printf("  --latency-log F  Write per-pulse latency CSV to F\n");
#ifdef MOCK_GPIO
    printf("  --mock-log F     Write pulse timing CSV to F (default: %s)\n", mock_log_path);
#endif
    printf("\nPin numbers use BCM (Broadcom) GPIO numbering, not physical board pin numbers.\n");
}

// Validate a BCM GPIO pin number. Returns 0 on success, -1 on error (message printed).
int validate_gpio_pin(int pin, const char *label) {
    if (pin == -1) return 0;  // disabled is always valid

    if (pin < 0 || pin > 27) {
        fprintf(stderr, "Error: %s pin %d is out of range. Valid BCM GPIO pins are 0-27 (or -1 to disable).\n", label, pin);
        return -1;
    }

    // BCM 0, 1: reserved for I2C0 HAT EEPROM identification
    if (pin == 0 || pin == 1) {
        fprintf(stderr, "Error: BCM GPIO %d is reserved for I2C0 HAT EEPROM identification and cannot be used.\n", pin);
        return -1;
    }

    // BCM 14, 15: reserved for UART serial (typically used by GPS receiver)
    if (pin == 14 || pin == 15) {
        fprintf(stderr, "Error: BCM GPIO %d is reserved for UART serial (typically used by GPS receiver) and cannot be used.\n", pin);
        return -1;
    }

    // BCM 4: commonly used for GPS PPS input — warn but allow
    if (pin == 4) {
        printf("Warning: BCM GPIO 4 is commonly used for GPS PPS input; using it for IRIG output may conflict with clock disciplining.\n");
    }

    return 0;
}

int main(int argc, char *argv[]) {
    int normal_pin = 9;     // default: BCM GPIO 9
    int inverted_pin = -1;  // default: disabled
    double warn_threshold_ms = 1.0;  // default LED warning threshold

    // Long options for --frames, --latency-log, and --mock-log
    static struct option long_options[] = {
        {"frames",      required_argument, 0, 'F'},
        {"latency-log", required_argument, 0, 'L'},
        {"mock-log",    required_argument, 0, 'M'},
        {"help",        no_argument,       0, 'h'},
        {0, 0, 0, 0}
    };

    // Parse command-line arguments with getopt_long
    int opt;
    while ((opt = getopt_long(argc, argv, "p:n:w:dh", long_options, NULL)) != -1) {
        switch (opt) {
            case 'p':
                normal_pin = atoi(optarg);
                break;
            case 'n':
                inverted_pin = atoi(optarg);
                break;
            case 'w':
                warn_threshold_ms = atof(optarg);
                break;
            case 'd':
                debug_mode = 1;
                break;
            case 'h':
                print_usage(argv[0]);
                return 0;
            case 'F':
                max_frames = atoi(optarg);
                if (max_frames <= 0) {
                    fprintf(stderr, "Error: --frames must be a positive integer\n");
                    return 1;
                }
                break;
            case 'L':
                strncpy(latency_log_path, optarg, sizeof(latency_log_path) - 1);
                latency_log_path[sizeof(latency_log_path) - 1] = '\0';
                break;
            case 'M':
#ifdef MOCK_GPIO
                strncpy(mock_log_path, optarg, sizeof(mock_log_path) - 1);
                mock_log_path[sizeof(mock_log_path) - 1] = '\0';
#else
                printf("Warning: --mock-log is only available in mock mode (compile with -DMOCK_GPIO)\n");
#endif
                break;
            default:
                print_usage(argv[0]);
                return 1;
        }
    }

    if (debug_mode) {
        printf("Debug mode enabled\n");
    }

#ifndef MOCK_GPIO
    // Validate pin numbers (only meaningful for real GPIO)
    if (validate_gpio_pin(normal_pin, "Normal") < 0) return 1;
    if (validate_gpio_pin(inverted_pin, "Inverted") < 0) return 1;

    // Both pins cannot be the same (unless both disabled)
    if (normal_pin != -1 && normal_pin == inverted_pin) {
        fprintf(stderr, "Error: Normal and inverted pins cannot be the same (both set to BCM GPIO %d).\n", normal_pin);
        return 1;
    }

    // Warn if both pins are disabled
    if (normal_pin == -1 && inverted_pin == -1) {
        printf("Warning: Both output pins are disabled (-1). No GPIO output will be generated.\n");
    }
#endif

    signal(SIGTERM, signal_handler);
    signal(SIGINT, signal_handler);

#ifdef MOCK_GPIO
    printf("IRIG-H Timecode Sender starting (MOCK GPIO mode)...\n");
    mock_log = fopen(mock_log_path, "w");
    if (!mock_log) {
        fprintf(stderr, "Error: Cannot open mock log file: %s\n", mock_log_path);
        return 1;
    }
    fprintf(mock_log, "rising_edge_ns,falling_edge_ns\n");
    printf("Mock log: %s\n", mock_log_path);
#else
    // Print startup info with actual pin configuration
    printf("IRIG-H Timecode Sender starting...\n");
    if (normal_pin >= 0)
        printf("  Normal output:   BCM GPIO %d\n", normal_pin);
    else
        printf("  Normal output:   disabled\n");
    if (inverted_pin >= 0)
        printf("  Inverted output: BCM GPIO %d\n", inverted_pin);
    else
        printf("  Inverted output: disabled\n");
    printf("Using direct hardware register access\n");
#endif

    if (max_frames > 0) {
        printf("  Frame limit:     %d\n", max_frames);
    }
    if (latency_log_path[0] != '\0') {
        printf("  Latency log:     %s\n", latency_log_path);
    }

    irig_h_sender_t *sender = create_irig_h_sender(normal_pin, inverted_pin);
    if (!sender) {
        printf("Failed to initialize IRIG-H sender\n");
#ifdef MOCK_GPIO
        if (mock_log) fclose(mock_log);
#endif
        return 1;
    }
    sender->warn_threshold_ms = warn_threshold_ms;

    printf("IRIG-H sender initialized successfully\n");
    printf("  LED warn threshold: %.2f ms\n", sender->warn_threshold_ms);
    start_irig_sender(sender);
    printf("IRIG-H transmission started\n");

    while (running) {
        sleep(1);
    }

    printf("Stopping IRIG-H sender...\n");
    finish_irig_sender(sender);
    printf("IRIG-H sender stopped\n");

#ifdef MOCK_GPIO
    if (mock_log) {
        fclose(mock_log);
        printf("Mock log written to %s\n", mock_log_path);
    }
#endif

    return 0;
}
