/*
 * gpiouart v2 - bit-bang UART console over the BPI-R4 SFP1 cage control pins
 * for GPON ONU sticks (Huawei MA5671A: console TX on SFP pin 2 = TX_Fault,
 * console RX on SFP pin 7 = RS0).
 *
 * BPI-R4 wiring (mt7988a-bananapi-bpi-r4.dtsi, sfp1 node):
 *   pio 69  tx-fault      <- module console TX   (we receive here)
 *   pio 21  rate-select0  -> module console RX   (we transmit here)
 *   pio 70  tx-disable    -> held low
 *
 * MT7988 GPIO register layout (pinctrl-mt7988.c): DIR 0x000, DO 0x100,
 * DI 0x200, one 32-bit register per 32 pins, stride 0x10. Base 0x1001f000.
 *
 * Modes:
 *   selftest  - verify direction registers, TX pad read-back, RX idle level
 *   console   - interactive terminal (Ctrl+] exits, Ctrl+C is forwarded)
 *   auto      - unattended recovery session: catch U-Boot with Ctrl+C, log in
 *               to the ONU Linux console, run command files, detect TX
 *               polarity automatically, log everything with timestamps.
 *
 * Build (target): aarch64-openwrt-linux-gcc -O2 -pthread -o gpiouart gpiouart.c
 * Build (host simulation, --sim): gcc -O2 -pthread -DSIM_ONLY -o gpiouart-sim gpiouart.c
 */
#define _GNU_SOURCE
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <time.h>
#include <sched.h>
#include <pthread.h>
#include <signal.h>
#include <getopt.h>
#include <ctype.h>
#include <stdarg.h>
#include <termios.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#ifndef SIM_ONLY
#include <linux/gpio.h>
#endif

/* ------------------------------------------------------------------ config */
#define GPIO_PHYS_BASE   0x1001f000UL
#define GPIO_MAP_LEN     0x1000
#define REG_DIR_OFF      0x000
#define REG_DO_OFF       0x100
#define REG_DI_OFF       0x200
#define REG_MODE_OFF     0x300   /* 4 bits per pin, 8 pins per reg, stride 0x10 */

#define DEF_TX_PIN   21
#define DEF_RX_PIN   69
#define DEF_DIS_PIN  70

#define QUIET_MS        300      /* line considered quiescent after this much silence */
#define CTRLC_PERIOD_MS cfg.ctrlc_ms
#define NUDGE_PERIOD_MS 5000
#define STATS_PERIOD_MS 15000

enum { MODE_NONE, MODE_SELFTEST, MODE_CONSOLE, MODE_AUTO, MODE_EDGES };

static struct {
    int mode;
    int baud;
    int tx_pin, rx_pin, dis_pin;
    volatile int invert_tx;
    int invert_rx;
    int want_mmio;
    int cpu_rx, cpu_tx;
    const char *log_path, *mirror_path, *cmds_uboot, *cmds_linux, *cmds_failsafe, *state_path, *initial_cmd;
    const char *sim_path, *creds;
    int autoflip, reset_state;
    int timeout_s;
    int max_sessions;
    double sample_frac;     /* where inside a register read the pin is captured (0..1) */
    double rx_shift;        /* extra RX sampling phase shift, in bits (+ = later) */
    int rx_sampling;        /* 1 = legacy fixed-instant sampling decoder, 0 = edge-timing decoder */
    int ctrlc_ms;           /* U-Boot break-in: Ctrl+C period while hunting */
    int hunt_byte;          /* byte sent while hunting (0x03); --send-hex overrides for probing */
    int initial_baud;       /* baud for --initial-cmd (the module's Linux console may differ from U-Boot) */
} cfg = {
    .mode = MODE_NONE, .baud = 115200,
    .tx_pin = DEF_TX_PIN, .rx_pin = DEF_RX_PIN, .dis_pin = DEF_DIS_PIN,
    .invert_tx = 0, .invert_rx = 0, .want_mmio = 1, .cpu_rx = 2, .cpu_tx = 1,
    .creds = "root:admin123,root:,admin:admin", .autoflip = 1, .timeout_s = 0,
    .max_sessions = 1, .sample_frac = 0.5, .rx_shift = 0.0, .ctrlc_ms = 50, .hunt_byte = 0x03,
};

/* ----------------------------------------------------------- time source */
#if defined(__aarch64__)
static inline uint64_t now_cyc(void) { uint64_t v; asm volatile("mrs %0, cntvct_el0" : "=r"(v)); return v; }
static inline uint64_t cyc_freq(void) { uint64_t v; asm volatile("mrs %0, cntfrq_el0" : "=r"(v)); return v; }
#else
static inline uint64_t now_cyc(void) { struct timespec ts; clock_gettime(CLOCK_MONOTONIC, &ts); return (uint64_t)ts.tv_sec * 1000000000ULL + ts.tv_nsec; }
static inline uint64_t cyc_freq(void) { return 1000000000ULL; }
#endif
static volatile double bit_cyc; /* counter cycles per bit (changed at runtime by @baud) */
static uint64_t t_start_cyc;
static inline void wait_until(double target) { while ((double)now_cyc() < target) { } }
static uint64_t now_ms(void) { struct timespec ts; clock_gettime(CLOCK_MONOTONIC, &ts); return (uint64_t)ts.tv_sec * 1000ULL + ts.tv_nsec / 1000000ULL; }
static double elapsed_s(void) { return (double)(now_cyc() - t_start_cyc) / (double)cyc_freq(); }
static void sleep_ms(int ms) { struct timespec ts = { ms / 1000, (ms % 1000) * 1000000L }; nanosleep(&ts, NULL); }

/* ------------------------------------------------------------ SPSC rings */
#define RING_SZ 65536
#define RING_MASK (RING_SZ - 1)
typedef struct { volatile unsigned head, tail; int buf[RING_SZ]; unsigned long overflow; } ring_t;
static ring_t rx_ring, tx_ring;
static inline void ring_push(ring_t *r, int v) {
    unsigned h = r->head, n = (h + 1) & RING_MASK;
    if (n == __atomic_load_n(&r->tail, __ATOMIC_ACQUIRE)) { r->overflow++; return; }
    r->buf[h] = v;
    __atomic_store_n(&r->head, n, __ATOMIC_RELEASE);
}
static inline int ring_pop(ring_t *r) {
    unsigned t = r->tail;
    if (t == __atomic_load_n(&r->head, __ATOMIC_ACQUIRE)) return -1;
    int v = r->buf[t];
    __atomic_store_n(&r->tail, (t + 1) & RING_MASK, __ATOMIC_RELEASE);
    return v;
}
static inline int ring_empty(ring_t *r) { return r->tail == __atomic_load_n(&r->head, __ATOMIC_ACQUIRE); }

/* --------------------------------------------------------------- counters */
static volatile unsigned long st_rx_ok, st_rx_fe, st_glitch, st_tx, st_break;
static volatile int stop_flag;
#define RX_FLAG_FE 0x100

/* --------------------------------------------------------------- logging */
static FILE *log_fp;
static int mirror_fd = -1;
static int quiet_stdout;          /* console mode: don't echo events to stdout */
static int at_line_start = 1;

static void out_raw(const char *s, size_t n) {
    if (log_fp) fwrite(s, 1, n, log_fp);
    if (!quiet_stdout) fwrite(s, 1, n, stdout);
    if (mirror_fd >= 0) { ssize_t w = write(mirror_fd, s, n); (void)w; }
}
static void out_flush(void) { if (log_fp) fflush(log_fp); if (!quiet_stdout) fflush(stdout); }
static void log_sync(void) { if (log_fp) { fflush(log_fp); fsync(fileno(log_fp)); } }
static void ev(const char *fmt, ...) {
    char b[1024]; int n = 0;
    if (!at_line_start) { b[n++] = '\n'; at_line_start = 1; }
    n += snprintf(b + n, sizeof b - n, "[%9.3f] ### ", elapsed_s());
    va_list ap; va_start(ap, fmt); n += vsnprintf(b + n, sizeof b - n, fmt, ap); va_end(ap);
    if (n > (int)sizeof b - 2) n = sizeof b - 2;
    b[n++] = '\n';
    out_raw(b, n); out_flush();
}
/* log one received byte as text */
static void log_rx_byte(int v) {
    char b[32]; int n = 0;
    if (at_line_start) { n = snprintf(b, sizeof b, "[%9.3f] ", elapsed_s()); at_line_start = 0; }
    if (v & RX_FLAG_FE) n += snprintf(b + n, sizeof b - n, "<FE:%02X>", v & 0xff);
    else {
        int c = v & 0xff;
        if (c == '\n') { b[n++] = '\n'; at_line_start = 1; }
        else if (c == '\r') { /* drop */ }
        else if (c == '\t' || (c >= 0x20 && c < 0x7f)) b[n++] = (char)c;
        else n += snprintf(b + n, sizeof b - n, "\\x%02X", c);
    }
    if (n) out_raw(b, n);
}

/* ------------------------------------------------------- GPIO HW access */
static volatile uint32_t *gpio_base;
static int use_mmio;
static int chip_fd = -1, rx_fd = -1, tx_fd = -1, dis_fd = -1;
static int sim_fd = -1;

static inline volatile uint32_t *reg(unsigned off, int pin) { return gpio_base + (off + (pin / 32) * 0x10) / 4; }
static inline uint32_t pin_mask(int pin) { return 1u << (pin % 32); }

#ifndef SIM_ONLY
static int line_request(unsigned pin, unsigned long long flags, int def) {
    struct gpio_v2_line_request rq; memset(&rq, 0, sizeof rq);
    rq.offsets[0] = pin; rq.num_lines = 1; rq.config.flags = flags;
    snprintf(rq.consumer, sizeof rq.consumer, "gpiouart_%u", pin);
    if (flags & GPIO_V2_LINE_FLAG_OUTPUT) {
        rq.config.num_attrs = 1;
        rq.config.attrs[0].attr.id = GPIO_V2_LINE_ATTR_ID_OUTPUT_VALUES;
        rq.config.attrs[0].attr.values = def ? 1 : 0;
        rq.config.attrs[0].mask = 1;
    }
    if (ioctl(chip_fd, GPIO_V2_GET_LINE_IOCTL, &rq) < 0) return -1;
    return rq.fd;
}
static int line_get(int fd) { struct gpio_v2_line_values v = { .bits = 0, .mask = 1 }; if (ioctl(fd, GPIO_V2_LINE_GET_VALUES_IOCTL, &v) < 0) return -1; return v.bits & 1; }
static int line_set(int fd, int val) { struct gpio_v2_line_values v = { .bits = val ? 1 : 0, .mask = 1 }; return ioctl(fd, GPIO_V2_LINE_SET_VALUES_IOCTL, &v); }
#else
static int line_request(unsigned pin, unsigned long long flags, int def) { (void)pin; (void)flags; (void)def; return -1; }
static int line_get(int fd) { (void)fd; return -1; }
static int line_set(int fd, int val) { (void)fd; (void)val; return -1; }
#endif

static inline int rx_phys(void) {
    if (use_mmio) return (*reg(REG_DI_OFF, cfg.rx_pin) & pin_mask(cfg.rx_pin)) != 0;
    return line_get(rx_fd);
}
static inline int rx_level(void) { return rx_phys() ^ cfg.invert_rx; }
static double rd_cost;            /* cycles per DI register read (measured) */
static volatile uint64_t last_rx_edge_cyc;   /* last level change seen on RX (TX holds off while active) */
static volatile int tx_active;               /* TX byte in progress: RX must not touch the bus (its reads delay our writes) */
static void tx_byte_hw(unsigned c);
static int oversample;            /* 3 samples per bit when reads are fast enough */
static uint32_t do_shadow;        /* shadow of the TX pin's DOUT bank register */
static inline void tx_phys(int v) {
    if (use_mmio) {
        uint32_t m = pin_mask(cfg.tx_pin);
        if (v) do_shadow |= m; else do_shadow &= ~m;
        *reg(REG_DO_OFF, cfg.tx_pin) = do_shadow;
    } else line_set(tx_fd, v);
}
static inline void tx_set(int logical) { tx_phys(logical ^ cfg.invert_tx); }
static inline int tx_readback(void) { return (*reg(REG_DI_OFF, cfg.tx_pin) & pin_mask(cfg.tx_pin)) != 0; }

static int gpio_open(void) {
#ifdef SIM_ONLY
    fprintf(stderr, "built with SIM_ONLY: hardware access not available, use --sim\n");
    return -1;
#else
    chip_fd = open("/dev/gpiochip0", O_RDWR);
    if (chip_fd < 0) { fprintf(stderr, "open /dev/gpiochip0: %s\n", strerror(errno)); return -1; }
    if (cfg.rx_pin == cfg.tx_pin) {
        /* loopback self-test: decode our own TX by reading the pad state (DIN) of the TX pin */
        fprintf(stderr, "loopback mode: RX reads DIN of TX pin %d (needs MMIO)\n", cfg.tx_pin);
    } else {
        rx_fd = line_request(cfg.rx_pin, GPIO_V2_LINE_FLAG_INPUT, 0);
        if (rx_fd < 0) { fprintf(stderr, "request RX line %d: %s\n", cfg.rx_pin, strerror(errno)); return -1; }
    }
    tx_fd = line_request(cfg.tx_pin, GPIO_V2_LINE_FLAG_OUTPUT, 1 ^ cfg.invert_tx);
    if (tx_fd < 0) { fprintf(stderr, "request TX line %d: %s\n", cfg.tx_pin, strerror(errno)); return -1; }
    dis_fd = line_request(cfg.dis_pin, GPIO_V2_LINE_FLAG_OUTPUT, 0);
    if (dis_fd < 0) { fprintf(stderr, "request TX_DISABLE line %d: %s\n", cfg.dis_pin, strerror(errno)); return -1; }
    use_mmio = 0;
    if (cfg.want_mmio) {
        int mfd = open("/dev/mem", O_RDWR | O_SYNC);
        if (mfd < 0) fprintf(stderr, "open /dev/mem: %s -> falling back to ioctl\n", strerror(errno));
        else {
            void *m = mmap(NULL, GPIO_MAP_LEN, PROT_READ | PROT_WRITE, MAP_SHARED, mfd, GPIO_PHYS_BASE);
            close(mfd);
            if (m == MAP_FAILED) fprintf(stderr, "mmap GPIO: %s -> falling back to ioctl\n", strerror(errno));
            else { gpio_base = (volatile uint32_t *)m; use_mmio = 1; do_shadow = *reg(REG_DO_OFF, cfg.tx_pin); }
        }
    }
    /* measure the read cost: it decides the sampling plan */
    { uint64_t a = now_cyc(); for (int i = 0; i < 4000; i++) (void)rx_phys(); uint64_t b = now_cyc(); rd_cost = (double)(b - a) / 4000.0; }
    oversample = (rd_cost * 3.0 < bit_cyc * 0.4);
    tx_set(1);
    return 0;
#endif
}

/* ----------------------------------------------------------- threading */
static void pin_cpu(int cpu) {
    if (cpu < 0) return;
    cpu_set_t s; CPU_ZERO(&s); CPU_SET(cpu, &s);
    if (pthread_setaffinity_np(pthread_self(), sizeof s, &s) != 0) ev("warn: cannot pin to cpu%d: %s", cpu, strerror(errno));
}
static void set_fifo(void) {
    struct sched_param sp = { .sched_priority = sched_get_priority_max(SCHED_FIFO) };
    if (pthread_setschedparam(pthread_self(), SCHED_FIFO, &sp) != 0) ev("warn: SCHED_FIFO: %s", strerror(errno));
}

/* RX. A GPIO register read on MT7988 costs ~2.9 us (measured), a bit at 115200 is 8.68 us,
 * so the sampling plan must be built around the read cost:
 *  - the falling edge is seen up to one read late -> edge estimate = t(read done) - cost/2
 *  - every sample is scheduled so that the *middle* of the read lands on the bit centre
 *  - one read per bit; 3x oversampling only when reads are fast enough (cost*3 < 0.4 bit) */
/* Model: a register read lasting rd_cost captures the pin at fraction sample_frac of the read.
 * The read that returns 0 captured at (done - rd_cost*(1-sf)); the previous read captured one
 * rd_cost earlier; the edge lies between -> estimate done - rd_cost*(1.5-sf), error +-rd_cost/2.
 * To capture at instant T, start the read at T - rd_cost*sf. rx_shift (in bits) is an extra
 * empirical phase correction, tunable at runtime. */
/* trace (loopback diagnostics): record RX read instants relative to the TX start-bit time */
static volatile uint64_t tx_start_cyc;
#define TRN 96
static struct { char tag; int val; double t; } tr[TRN]; static volatile int tr_n; static int trace_frames;
static inline void TR(char tag, int val, uint64_t when) { int n = tr_n; if (trace_frames > 0 && n < TRN) { tr[n].tag = tag; tr[n].val = val; tr[n].t = ((double)when - (double)tx_start_cyc) / bit_cyc; tr_n = n + 1; } }

/* Edge-timing decoder (default). Instead of sampling at fixed instants, every level transition is
 * timestamped and the run lengths are rounded to whole bits. Only interval accuracy matters
 * (+- one register read ~0.33 bit), not the phase of individual samples, which was what broke the
 * sampling decoder at 2.9 us per read. Verified by hand against captured edge lists. */
static void *rx_thread_edges(void *a) {
    (void)a; pin_cpu(cfg.cpu_rx); set_fifo();
    unsigned spins = 0;
    double carry_start = -1;                        /* falling edge that closed the previous frame */
    for (;;) {
        double t_start;
        if (carry_start >= 0) { t_start = carry_start; carry_start = -1; }
        else {
            /* idle: wait for a falling edge; stamp = middle of the read that saw 0 */
            uint64_t t;
            const uint64_t quiet = (uint64_t)((double)cyc_freq() * 0.0006);   /* 600 us of silence before we talk */
            /* line held low (break: the module's console output is dead, e.g. after its GPON
             * daemon reclaims TX_FAULT) - its input may still work, so keep transmitting */
            { uint64_t low0 = now_cyc();
              while (rx_level() == 0) {
                  if (!ring_empty(&tx_ring) && now_cyc() - low0 > (uint64_t)((double)cyc_freq() * 0.002)) {
                      int c = ring_pop(&tx_ring); tx_byte_hw((unsigned)c); st_tx++;
                  }
                  if ((++spins & 0xffff) == 0 && stop_flag) return NULL;
              } }
            for (;;) {
                t = now_cyc();
                if (rx_level() == 0) break;
                /* TX happens HERE, from this always-spinning core: a byte sent from any other
                 * thread (after a sleep / with a second busy core) got its register accesses
                 * stretched 2-10x and the module received garbage. Measured, not theory. */
                if (!ring_empty(&tx_ring) && t - last_rx_edge_cyc > quiet) {
                    int c = ring_pop(&tx_ring);
                    tx_byte_hw((unsigned)c); st_tx++;
                    last_rx_edge_cyc = now_cyc();   /* our own byte counts as line activity for pacing */
                    continue;
                }
                if ((++spins & 0xffff) == 0 && stop_flag) return NULL;
            }
            t_start = (double)t + rd_cost * 0.5;
            last_rx_edge_cyc = t;
        }
        int bits[10]; int pos = 0; int cur = 0;
        for (;;) {
            uint64_t t = now_cyc(); int v = rx_level();
            double stamp = (double)t + rd_cost * 0.5;
            double elapsed = (stamp - t_start) / bit_cyc;
            if (v != cur) {
                last_rx_edge_cyc = t;
                int upto = (int)(elapsed + 0.5); if (upto > 10) upto = 10;
                while (pos < upto) bits[pos++] = cur;
                cur = v;
                if (pos >= 10) { if (v == 0 && elapsed >= 9.5) carry_start = stamp; break; }
            } else if (elapsed >= 9.5) {
                while (pos < 10) bits[pos++] = cur;
                break;
            }
        }
        unsigned c = 0;
        for (int i = 0; i < 8; i++) if (bits[1 + i]) c |= 1u << i;
        if (bits[0] != 0) { st_glitch++; continue; }
        if (bits[9] != 1) {
            if (c == 0) { st_break++; continue; }          /* all-zero frame = break condition, not data */
            st_rx_fe++; ring_push(&rx_ring, (int)(c | RX_FLAG_FE)); continue;
        }
        st_rx_ok++;
        ring_push(&rx_ring, (int)c);
    }
}

static void *rx_thread_hw(void *a) {
    if (!cfg.rx_sampling) return rx_thread_edges(a);
    (void)a; pin_cpu(cfg.cpu_rx); set_fifo();
    unsigned spins = 0;
    /* A read captures the pin at fraction sf of its duration. The read that returns 0 captured at
     * before + sf*rd; the previous read (which saw 1) captured at before - rd + sf*rd; the edge is
     * between: estimate before + rd*(sf - 0.5), error +-rd/2. Loopback traces confirmed that
     * measuring from the read *end* (as before) put every sample ~0.5 bit late. */
    const double edge_back = -rd_cost * (cfg.sample_frac - 0.5);
    const double lead = rd_cost * cfg.sample_frac - cfg.rx_shift * bit_cyc;   /* read start = T - lead */
    for (;;) {
        while (rx_level() == 0) { if ((++spins & 0xffff) == 0 && stop_flag) return NULL; }
        double edge; int v;
        for (;;) {
            uint64_t before = now_cyc();
            v = rx_level();
            if (v == 0) { uint64_t done = now_cyc(); edge = (double)before - edge_back; TR('E', 0, before); TR('e', 0, done); break; }
            if ((++spins & 0xffff) == 0 && stop_flag) return NULL;
        }
        /* start-bit validation at 0.5 bit (also the glitch filter) */
        wait_until(edge + bit_cyc * 0.5 - lead);
        { uint64_t b = now_cyc(); v = rx_level(); TR('S', v, b); }
        if (v != 0) { st_glitch++; if (trace_frames > 0) trace_frames--; continue; }
        unsigned c = 0;
        for (int i = 0; i < 8; i++) {
            double ctr = edge + bit_cyc * (1.5 + i);
            if (oversample) {
                wait_until(ctr - bit_cyc * 0.15 - lead); int s1 = rx_level();
                wait_until(ctr - lead);                  int s2 = rx_level();
                wait_until(ctr + bit_cyc * 0.15 - lead); int s3 = rx_level();
                if (s1 + s2 + s3 >= 2) c |= 1u << i;
            } else {
                wait_until(ctr - lead);
                uint64_t b = now_cyc(); v = rx_level(); TR('0' + i, v, b);
                if (v) c |= 1u << i;
            }
        }
        wait_until(edge + bit_cyc * 9.5 - lead);
        { uint64_t b = now_cyc(); v = rx_level(); TR('P', v, b); }
        if (trace_frames > 0) trace_frames--;
        if (v != 1) { st_rx_fe++; ring_push(&rx_ring, (int)(c | RX_FLAG_FE)); continue; }
        st_rx_ok++;
        ring_push(&rx_ring, (int)c);
    }
}
static void trace_dump(void) {
    if (!tr_n) return;
    ev("trace (times in bits since the TX start-bit write; E/e = edge read begin/end, S = start check, 0-7 = data, P = stop):");
    char b[1024]; int n = 0;
    for (int i = 0; i < tr_n; i++) { n += snprintf(b + n, sizeof b - n, "%c%d@%.2f ", tr[i].tag, tr[i].val, tr[i].t); if (n > 900 || tr[i].tag == 'P') { ev("  %s", b); n = 0; } }
    if (n) ev("  %s", b);
    tr_n = 0;
}
/* TX: one register write per bit (shadow copy of DOUT, refreshed once per byte) instead of a
 * read-modify-write, which costs ~5 us and would eat most of the bit time */
/* TX self-trace: after each bit write, read the pad back (DIN of the TX pin) and note when */
static int tx_trace_on; static double tx_tr_t[12]; static int tx_tr_v[12], tx_tr_w[12];
static void tx_byte_hw(unsigned c) {
    if (use_mmio) do_shadow = *reg(REG_DO_OFF, cfg.tx_pin);
    double t0 = (double)now_cyc();
    tx_start_cyc = (uint64_t)t0;
    tx_set(0);
    if (tx_trace_on) { tx_tr_w[0] = ((double)now_cyc() - t0) / bit_cyc * 100; tx_tr_v[0] = tx_readback(); tx_tr_t[0] = ((double)now_cyc() - t0) / bit_cyc; }
    for (int i = 0; i < 8; i++) {
        wait_until(t0 + bit_cyc * (1 + i)); tx_set((c >> i) & 1);
        if (tx_trace_on) { tx_tr_w[1 + i] = ((double)now_cyc() - t0) / bit_cyc * 100; tx_tr_v[1 + i] = tx_readback(); tx_tr_t[1 + i] = ((double)now_cyc() - t0) / bit_cyc; }
    }
    wait_until(t0 + bit_cyc * 9); tx_set(1);
    if (tx_trace_on) { tx_tr_w[9] = ((double)now_cyc() - t0) / bit_cyc * 100; tx_tr_v[9] = tx_readback(); tx_tr_t[9] = ((double)now_cyc() - t0) / bit_cyc; }
    wait_until(t0 + bit_cyc * 10.5);
}
static void tx_trace_print(void) {
    printf("TX self-trace (bit: write-done at %%bit, pad read-back value @ time in bits):\n  ");
    for (int i = 0; i < 10; i++) printf("b%d:w%d%% pad=%d@%.2f  ", i, tx_tr_w[i], tx_tr_v[i], tx_tr_t[i]);
    printf("\n");
}
/* Register writes from the TX thread stretch the RX thread's register reads (2.9 -> 4.8 us, bus
 * contention) and wreck the edge timing, so never transmit while the receive line is active:
 * wait until RX has been idle for a while (the module's echo of the previous byte included). */
static void *tx_thread_hw(void *a) {
    (void)a; pin_cpu(cfg.cpu_tx); set_fifo();
    const uint64_t quiet = (uint64_t)((double)cyc_freq() * 0.0006);   /* 600 us */
    for (;;) {
        int c = ring_pop(&tx_ring);
        if (c < 0) { if (stop_flag) return NULL; struct timespec ts = { 0, 100000 }; nanosleep(&ts, NULL); continue; }
        uint64_t t0 = now_cyc();
        while (now_cyc() - last_rx_edge_cyc < quiet && now_cyc() - t0 < (uint64_t)cyc_freq() / 2) { struct timespec ts = { 0, 50000 }; nanosleep(&ts, NULL); }
        tx_active = 1;
        tx_byte_hw((unsigned)c); st_tx++;
        tx_active = 0;
    }
}
/* simulation: plain tty/pty instead of GPIO */
static void *rx_thread_sim(void *a) {
    (void)a; unsigned char b[256];
    for (;;) {
        ssize_t n = read(sim_fd, b, sizeof b);
        if (n <= 0) { if (stop_flag || n == 0) return NULL; if (errno == EINTR) continue; sleep_ms(10); continue; }
        for (ssize_t i = 0; i < n; i++) { st_rx_ok++; ring_push(&rx_ring, b[i]); }
    }
}
static void *tx_thread_sim(void *a) {
    (void)a;
    for (;;) {
        int c = ring_pop(&tx_ring);
        if (c < 0) { if (stop_flag) return NULL; sleep_ms(1); continue; }
        unsigned char b = (unsigned char)c; ssize_t w = write(sim_fd, &b, 1); (void)w; st_tx++;
    }
}
static void tx_send(const char *s, size_t n) { for (size_t i = 0; i < n; i++) ring_push(&tx_ring, (unsigned char)s[i]); }
static void tx_str(const char *s) { tx_send(s, strlen(s)); }
static void tx_wait_drained(int max_ms) { uint64_t t = now_ms(); while (!ring_empty(&tx_ring) && now_ms() - t < (uint64_t)max_ms) sleep_ms(1); }

/* ------------------------------------------------------------ selftest */
static int do_selftest(void) {
    printf("gpiouart selftest: tx=pio%d rx=pio%d dis=pio%d baud=%d cntfrq=%llu Hz bit=%.2f cyc\n",
           cfg.tx_pin, cfg.rx_pin, cfg.dis_pin, cfg.baud, (unsigned long long)cyc_freq(), bit_cyc);
    if (gpio_open() < 0) return 2;
    printf("gpio access: %s\n", use_mmio ? "MMIO /dev/mem (fast)" : "gpiochip ioctl (slow fallback)");
    int fail = 0;
    if (use_mmio) {
        uint32_t dir_tx = *reg(REG_DIR_OFF, cfg.tx_pin), dir_rx = *reg(REG_DIR_OFF, cfg.rx_pin), dir_dis = *reg(REG_DIR_OFF, cfg.dis_pin);
        int d_tx = (dir_tx & pin_mask(cfg.tx_pin)) != 0, d_rx = (dir_rx & pin_mask(cfg.rx_pin)) != 0, d_dis = (dir_dis & pin_mask(cfg.dis_pin)) != 0;
        printf("DIR: pio%d=%s (expect out) pio%d=%s (expect in) pio%d=%s (expect out)\n",
               cfg.tx_pin, d_tx ? "out" : "IN", cfg.rx_pin, d_rx ? "OUT" : "in", cfg.dis_pin, d_dis ? "out" : "IN");
        if (!d_tx || d_rx || !d_dis) fail++;
        volatile uint32_t *mr;
        mr = gpio_base + (REG_MODE_OFF + (cfg.tx_pin / 8) * 0x10) / 4; printf("MODE: pio%d=%u", cfg.tx_pin, (*mr >> ((cfg.tx_pin % 8) * 4)) & 0xf);
        mr = gpio_base + (REG_MODE_OFF + (cfg.rx_pin / 8) * 0x10) / 4; printf(" pio%d=%u", cfg.rx_pin, (*mr >> ((cfg.rx_pin % 8) * 4)) & 0xf);
        mr = gpio_base + (REG_MODE_OFF + (cfg.dis_pin / 8) * 0x10) / 4; printf(" pio%d=%u (0 = GPIO expected)\n", cfg.dis_pin, (*mr >> ((cfg.dis_pin % 8) * 4)) & 0xf);
        /* TX pad read-back: DIN must follow DOUT */
        int bad = 0;
        for (int i = 0; i < 2000; i++) { int v = i & 1; tx_phys(v); wait_until((double)now_cyc() + bit_cyc * 0.25); if (tx_readback() != v) bad++; }
        tx_set(1);
        printf("TX read-back (pio%d DOUT->DIN, 2000 toggles): %d mismatches -> %s\n", cfg.tx_pin, bad, bad == 0 ? "PASS" : (bad < 2000 ? "FLAKY" : "FAIL (pad does not follow: mux/dir/short?)"));
        if (bad) fail++;
        /* timing of a DI read */
        uint64_t a = now_cyc(); for (int i = 0; i < 10000; i++) (void)rx_phys(); uint64_t b = now_cyc();
        printf("DI read cost: %.3f us (%.1f reads per bit)\n", (double)(b - a) / 10000.0 * 1e6 / (double)cyc_freq(), bit_cyc / ((double)(b - a) / 10000.0));
    }
    /* RX idle level / activity over 500 ms */
    unsigned long ones = 0, zeros = 0, edges = 0; int last = rx_phys();
    uint64_t t = now_ms();
    while (now_ms() - t < 500) { int v = rx_phys(); if (v) ones++; else zeros++; if (v != last) edges++; last = v; }
    printf("RX pio%d over 500ms: high %.1f%%, %lu edges -> %s\n", cfg.rx_pin, 100.0 * ones / (ones + zeros), edges,
           edges == 0 ? (ones ? "idle HIGH, silent (module idle or absent, RX polarity normal)" : "stuck LOW (module absent/pulled or RX inverted)")
                      : "ACTIVITY (module is transmitting)");
    printf("selftest result: %s\n", fail ? "FAIL" : "PASS");
    return fail ? 1 : 0;
}

/* edges: send one byte, then record every level transition on RX for a while and print the
 * interval statistics. The shortest intervals are single bits of the module's UART -> direct
 * calibration of the bit period in counter cycles, independent of cntfrq assumptions. */
static int edges_send_thread, edges_gate, edges_swap;
static void *edges_sender(void *a) {
    (void)a; pin_cpu(cfg.cpu_tx); set_fifo(); sleep_ms(30);   /* main thread is capturing by now */
    tx_active = 1; tx_byte_hw((unsigned)cfg.hunt_byte); tx_active = 0;
    return NULL;
}
/* --swap: a capture thread polls (CPU rx, FIFO) while the MAIN thread sends inline after 30 ms */
enum { MAXE_S = 20000 };
static uint64_t sw_ts[MAXE_S]; static unsigned char sw_lv[MAXE_S]; static volatile int sw_n;
static void *edges_capturer(void *a) {
    (void)a; pin_cpu(cfg.cpu_rx); set_fifo();
    int last = rx_phys(); uint64_t t0 = now_cyc(), limit = (uint64_t)(0.25 * (double)cyc_freq()); int n = 0;
    while (now_cyc() - t0 < limit && n < MAXE_S) {
        if (edges_gate && tx_active) { struct timespec ts = { 0, 10000 }; nanosleep(&ts, NULL); continue; }
        int v = rx_phys();
        if (v != last) { sw_ts[n] = now_cyc(); sw_lv[n] = (unsigned char)v; n++; last = v; }
    }
    sw_n = n; return NULL;
}
static int do_edges(void) {
    if (gpio_open() < 0) return 2;
    if (!use_mmio) { printf("edges needs MMIO\n"); return 2; }
    enum { MAXE = 20000 };
    static uint64_t ts[MAXE]; static unsigned char lv[MAXE]; int n = 0;
    pthread_t th; int threaded = edges_send_thread;
    if (edges_swap) {
        pthread_create(&th, NULL, edges_capturer, NULL);
        sleep_ms(30);
        tx_active = 1; tx_byte_hw((unsigned)cfg.hunt_byte); tx_active = 0;
        pthread_join(th, NULL);
        n = sw_n; memcpy(ts, sw_ts, n * sizeof ts[0]); memcpy(lv, sw_lv, n);
        printf("edges (swap: main sends inline, thread captures): sent 0x%02x\n", cfg.hunt_byte);
        goto report;
    }
    /* create the sender BEFORE pinning/raising this thread, or it inherits CPU2+FIFO99 and starves */
    if (threaded) pthread_create(&th, NULL, edges_sender, NULL);
    pin_cpu(cfg.cpu_rx); set_fifo();
    printf("edges: bit(assumed)=%.2f cyc, read cost=%.2f cyc; sending 0x%02x (%s) then capturing 200 ms\n", bit_cyc, rd_cost, cfg.hunt_byte, threaded ? "from a thread" : "inline");
    int last = rx_phys();
    if (!threaded) tx_byte_hw((unsigned)cfg.hunt_byte);
    uint64_t t0 = now_cyc(), limit = (uint64_t)(0.2 * (double)cyc_freq());
    while (now_cyc() - t0 < limit && n < MAXE) {
        if (edges_gate && tx_active) { struct timespec ts = { 0, 10000 }; nanosleep(&ts, NULL); continue; }
        int v = rx_phys();
        if (v != last) { ts[n] = now_cyc(); lv[n] = (unsigned char)v; n++; last = v; }
    }
    if (threaded) pthread_join(th, NULL);
report:
    if (tx_trace_on) tx_trace_print();
    printf("%d transitions captured\n", n);
    if (n < 2) return 0;
    /* histogram of intervals in units of assumed bits */
    int hist[40] = {0}; double minint = 1e18;
    for (int i = 1; i < n; i++) {
        double d = (double)(ts[i] - ts[i - 1]);
        if (d < minint) minint = d;
        int b = (int)(d / bit_cyc * 4.0 + 0.5); if (b > 39) b = 39; hist[b]++;
    }
    printf("shortest interval: %.1f cyc = %.2f assumed bits (%.2f us)\n", minint, minint / bit_cyc, minint * 1e6 / (double)cyc_freq());
    printf("interval histogram (quarter-bit bins): ");
    for (int b = 0; b < 40; b++) if (hist[b]) printf("%.2f:%d ", b / 4.0, hist[b]);
    printf("\ntransitions (new level : duration of the previous level in assumed bits), first %d:\nEDGES ", n < 4000 ? n : 4000);
    for (int i = 1; i < n && i < 4000; i++) printf("%d:%.2f ", lv[i], (double)(ts[i] - ts[i - 1]) / bit_cyc);
    printf("\n");
    return 0;
}

/* ------------------------------------------------- auto mode: text window */
#define WIN_SZ 4096
static char win[WIN_SZ + 1]; static int wlen;
static unsigned long rx_total;          /* good bytes consumed by logic */
static uint64_t last_rx_ms;
static void win_add(int c) {
    if (c == '\r') return;
    if (wlen >= WIN_SZ) { memmove(win, win + WIN_SZ / 2, WIN_SZ / 2); wlen = WIN_SZ / 2; }
    win[wlen++] = (char)c; win[wlen] = 0;
}
static const char *last_line(void) { char *p = memrchr(win, '\n', wlen); return p ? p + 1 : win; }
static int strcasestr_n(const char *hay, const char *needle) { return strcasestr(hay, needle) != NULL; }

/* pump: drain rx ring into log+window; returns number of bytes consumed */
static int pump(void) {
    int n = 0, v;
    while ((v = ring_pop(&rx_ring)) >= 0) {
        log_rx_byte(v);
        if (!(v & RX_FLAG_FE)) { win_add(v & 0xff); rx_total++; }
        last_rx_ms = now_ms(); n++;
    }
    if (n) out_flush();
    return n;
}
static void pump_ms(int ms) { uint64_t t = now_ms(); do { pump(); sleep_ms(2); } while (!stop_flag && now_ms() - t < (uint64_t)ms); pump(); }

enum { P_NONE = 0, P_UBOOT, P_LOGIN, P_PASSWORD, P_SHELL, P_CONT };
static const char *pname(int p) { static const char *n[] = { "none", "U-Boot prompt", "login prompt", "password prompt", "shell prompt", "shell continuation" }; return n[p]; }
/* classify the current last line (call only when quiescent) */
static int classify(void) {
    const char *l = last_line(); int n = strlen(l);
    while (n > 0 && l[n - 1] == ' ') n--;
    if (n == 0) return P_NONE;
    if (n >= 2 && l[n - 2] == '=' && l[n - 1] == '>') return P_UBOOT;
    /* a bare ">" is ash's continuation prompt (unterminated quote after a garbled command), not U-Boot */
    { int k = 0; while (k < n && (l[k] == ' ' || l[k] == '\b')) k++; if (n - k == 1 && l[k] == '>') return P_CONT; }
    /* "FALCON => " arrives garbled at 115200 (e.g. "F=N?> "): any short line ending in '>' is U-Boot here */
    if (n < 40 && l[n - 1] == '>') return P_UBOOT;
    if (strcasestr_n(l, "login:") || strcasestr_n(l, "login :")) return P_LOGIN;
    if (strcasestr_n(l, "assword")) return P_PASSWORD;
    if (n < 80 && (l[n - 1] == '#' || l[n - 1] == '$')) return P_SHELL;
    return P_NONE;
}
/* wait until line quiescent and a prompt class appears; returns class or P_NONE on timeout */
static int wait_prompt(int timeout_ms) {
    uint64_t t = now_ms();
    for (;;) {
        pump();
        if (now_ms() - last_rx_ms >= QUIET_MS && wlen > 0) { int p = classify(); if (p != P_NONE) return p; }
        if (stop_flag || now_ms() - t >= (uint64_t)timeout_ms) return P_NONE;
        sleep_ms(5);
    }
}

/* ------------------------------------------------- auto mode: actions */
static int tx_confirmed, tx_fail, uboot_sessions, linux_sessions;
static int cmd_timeout_ms = 10000;      /* per-command prompt wait, changed by @timeout in command files */
static void save_state(void) {
    if (!cfg.state_path) return;
    FILE *f = fopen(cfg.state_path, "w");
    if (f) { fprintf(f, "invert_tx=%d\ntx_confirmed=%d\n", cfg.invert_tx, tx_confirmed); fclose(f); }
}
static void load_state(void) {
    if (!cfg.state_path || cfg.reset_state) return;
    FILE *f = fopen(cfg.state_path, "r"); if (!f) return;
    char l[128];
    while (fgets(l, sizeof l, f)) { int v; if (sscanf(l, "invert_tx=%d", &v) == 1) { cfg.invert_tx = v; ev("state: invert_tx=%d loaded from %s", v, cfg.state_path); } }
    fclose(f);
}
static void flip_polarity(const char *why) {
    cfg.invert_tx ^= 1; tx_fail = 0;
    tx_set(1);
    ev("TX POLARITY FLIPPED -> invert_tx=%d (%s). If the module is in U-Boot window now, re-plug it.", cfg.invert_tx, why);
    save_state();
}
static void tx_failed_once(const char *what) {
    tx_fail++;
    ev("no reaction to TX (%s), tx_fail=%d, invert_tx=%d", what, tx_fail, cfg.invert_tx);
    if (cfg.autoflip && !tx_confirmed && tx_fail >= 2) flip_polarity("no echo/reaction");
}
static void confirm_tx(const char *how) { if (!tx_confirmed) { tx_confirmed = 1; ev("TX CONFIRMED (%s) with invert_tx=%d", how, cfg.invert_tx); save_state(); } tx_fail = 0; }

/* send one command line, wait for next prompt */
static int run_cmd(const char *cmd, int timeout_ms) {
    ev("send: %s", cmd);
    int from = wlen;
    tx_str(cmd); tx_str("\r"); tx_wait_drained(2000);
    int p = wait_prompt(timeout_ms);
    if (p == P_NONE) ev("no prompt after '%s' within %d ms", cmd, timeout_ms);
    else if (wlen > from && strstr(win + from, cmd)) confirm_tx("command echo");
    return p;
}
static void run_cmd_file(const char *path, const char *what) {
    if (!path) { ev("no %s command file configured", what); return; }
    FILE *f = fopen(path, "r");
    if (!f) { ev("cannot open %s command file %s: %s", what, path, strerror(errno)); return; }
    ev("running %s commands from %s", what, path);
    char l[512];
    while (!stop_flag && fgets(l, sizeof l, f)) {
        char *s = l; while (*s == ' ' || *s == '\t') s++;
        char *e = s + strlen(s); while (e > s && (e[-1] == '\n' || e[-1] == '\r' || e[-1] == ' ')) *--e = 0;
        if (!*s || *s == '#') continue;
        int delay = 0;
        if (!strncmp(s, "@sleep ", 7)) { delay = atoi(s + 7); ev("sleep %d ms", delay); pump_ms(delay); continue; }
        if (!strncmp(s, "@timeout ", 9)) { cmd_timeout_ms = atoi(s + 9); ev("command timeout = %d ms", cmd_timeout_ms); continue; }
        if (!strncmp(s, "@verify ", 8)) {
            /* "@verify CMD :: EXPECT" - a single corrupted TX byte can wreck a command (seen: uci Parse error),
             * so send CMD, require EXPECT in the reply, retry up to 4 times */
            char *sep = strstr(s + 8, " :: ");
            if (!sep) { ev("bad @verify line"); continue; }
            *sep = 0; const char *vcmd = s + 8, *expect = sep + 4;
            int ok = 0;
            for (int attempt = 1; attempt <= 4 && !ok; attempt++) {
                int from = wlen;
                run_cmd(vcmd, cmd_timeout_ms); pump_ms(300);
                if (wlen > from && strstr(win + from, expect)) ok = 1;
                else ev("verify attempt %d FAILED (expected \"%s\")", attempt, expect);
            }
            ev("verify %s: %s", ok ? "OK" : "GAVE UP", vcmd);
            if (!ok) { ev("aborting command file"); break; }
            continue;
        }
        if (!strncmp(s, "@baud ", 6)) {
            int b = atoi(s + 6);
            if (b >= 300 && b <= 230400) { cfg.baud = b; bit_cyc = (double)cyc_freq() / (double)b; ev("BAUD switched to %d (bit=%.1f cyc, %.1f reads/bit)", b, bit_cyc, bit_cyc / rd_cost); }
            continue;
        }
        if (!strcmp(s, "@enter")) { ev("send: <Enter>"); tx_str("\r"); tx_wait_drained(2000); wait_prompt(5000); continue; }
        if (!strncmp(s, "@raw ", 5)) { ev("send raw: %s", s + 5); tx_str(s + 5); tx_wait_drained(3000); pump_ms(300); continue; }
        run_cmd(s, cmd_timeout_ms);
        pump_ms(200);
    }
    fclose(f);
    ev("%s commands finished", what);
    log_sync();
}
/* try credentials; returns 1 when a shell prompt was reached */
static int do_login(void) {
    char creds[256]; strncpy(creds, cfg.creds, sizeof creds - 1); creds[sizeof creds - 1] = 0;
    char *save = NULL;
    tx_wait_drained(200);            /* let queued Ctrl+C go out before typing */
    for (char *tok = strtok_r(creds, ",", &save); tok; tok = strtok_r(NULL, ",", &save)) {
        char *user = tok, *pass = strchr(tok, ':'); if (pass) *pass++ = 0; else pass = "";
        ev("login attempt user='%s' pass='%s'", user, pass);
        int from = wlen;
        tx_str(user); tx_str("\r"); tx_wait_drained(2000);
        uint64_t t = now_ms(); int echo = 0, p = P_NONE;
        while (!stop_flag && now_ms() - t < 6000) {
            pump();
            if (!echo && wlen > from && strstr(win + from, user)) echo = 1;
            if (now_ms() - last_rx_ms >= QUIET_MS && wlen > from) { p = classify(); if (p != P_NONE) break; }
            sleep_ms(5);
        }
        if (echo) confirm_tx("login echo");
        if (p == P_PASSWORD) {
            confirm_tx("password prompt");
            from = wlen;
            tx_str(pass); tx_str("\r"); tx_wait_drained(2000);
            p = wait_prompt(10000);
        }
        if (p == P_SHELL) { ev("LOGIN OK as '%s'", user); return 1; }
        if (p == P_LOGIN) { ev("login rejected for '%s'", user); continue; }
        if (p == P_NONE && !echo) { tx_failed_once("login name not echoed"); return 0; }
        if (p == P_NONE) { ev("login: timeout waiting for prompt"); return 0; }
    }
    ev("all credentials rejected");
    return 0;
}
static void stats(void) {
    ev("stats: rx_ok=%lu rx_fe=%lu glitch=%lu break=%lu tx=%lu ring_ovf=%lu invert_tx=%d tx_confirmed=%d uboot_sessions=%d linux_sessions=%d",
       st_rx_ok, st_rx_fe, st_glitch, st_break, st_tx, rx_ring.overflow, cfg.invert_tx, tx_confirmed, uboot_sessions, linux_sessions);
    log_sync();   /* survive a power cut: the log is the whole point of the run */
}

static void on_signal(int s) { (void)s; stop_flag = 1; }

static int do_auto(void) {
    ev("auto mode start: baud=%d tx=pio%d rx=pio%d invert_tx=%d invert_rx=%d access=%s cpus rx=%d tx=%d",
       cfg.baud, cfg.tx_pin, cfg.rx_pin, cfg.invert_tx, cfg.invert_rx, cfg.sim_path ? "SIM" : (use_mmio ? "MMIO" : "ioctl"), cfg.cpu_rx, cfg.cpu_tx);
    if (!cfg.sim_path) ev("timing: bit=%.2f cyc, DI read=%.2f cyc (%.2f us, %.1f reads/bit), decoder=%s",
                          bit_cyc, rd_cost, rd_cost * 1e6 / (double)cyc_freq(), bit_cyc / rd_cost,
                          cfg.rx_sampling ? (oversample ? "sampling 3x" : "sampling 1x") : "edge-timing");
    signal(SIGINT, on_signal); signal(SIGTERM, on_signal);
    uint64_t t0 = now_ms(), next_ctrlc = t0, next_nudge = t0 + NUDGE_PERIOD_MS, next_stats = t0 + STATS_PERIOD_MS;
    unsigned long handled_at = 0;
    uint64_t boot_seen_ms = 0; unsigned long boot_seen_rx = 0;
    int failsafe_done = 0; unsigned long failsafe_seen_rx = 0; uint64_t blackout_until = 0;
    if (cfg.initial_cmd) {
        double saved = bit_cyc;
        if (cfg.initial_baud) { bit_cyc = (double)cyc_freq() / (double)cfg.initial_baud; }
        ev("initial command (blind, %d baud): %s", cfg.initial_baud ? cfg.initial_baud : cfg.baud, cfg.initial_cmd);
        tx_str("\r"); tx_str(cfg.initial_cmd); tx_str("\r"); tx_wait_drained(5000); pump_ms(800);
        bit_cyc = saved;
        /* the echo/prompt of the initial command must not be mistaken for a live session */
        wlen = 0; win[0] = 0; handled_at = rx_total;
        blackout_until = now_ms() + 15000;   /* a reboot follows: ignore shell/login prompts for a while */
    }
    ev("hunting: sending Ctrl+C every %d ms (U-Boot break-in). Plug / re-plug the ONU module now.", CTRLC_PERIOD_MS);
    while (!stop_flag) {
        /* OpenWrt failsafe window during the module's Linux boot: "Press [enter] to enter failsafe mode".
         * Text arrives garbled at times, so match several fragments. */
        if (cfg.cmds_failsafe && !failsafe_done && rx_total != failsafe_seen_rx && wlen > 0) {
            failsafe_seen_rx = rx_total;
            const char *tail = wlen > 400 ? win + wlen - 400 : win;
            if (strcasestr(tail, "failsafe") || strcasestr(tail, "[f] key") || strcasestr(tail, "enter] to") || strcasestr(tail, "[1], [2]") || strcasestr(tail, "afe mode")) {
                /* this firmware: "Press the [f] key and hit [enter] to enter failsafe mode" */
                ev("FAILSAFE prompt seen: sending f + Enter");
                for (int k = 0; k < 5; k++) { tx_str("f\r"); tx_wait_drained(800); pump_ms(200); }
                int p = wait_prompt(15000);
                ev("after failsafe Enter: %s: \"%s\"", pname(p), last_line());
                if (p == P_SHELL || p == P_UBOOT) { failsafe_done = 1; run_cmd_file(cfg.cmds_failsafe, "failsafe"); handled_at = rx_total; }
                else { ev("failsafe: no shell prompt (retry on next boot)"); }
            }
        }
        pump();
        uint64_t now = now_ms();
        if (cfg.timeout_s && now - t0 > (uint64_t)cfg.timeout_s * 1000) { ev("timeout reached, exiting"); break; }
        if (now >= next_stats) { stats(); next_stats = now + STATS_PERIOD_MS; }
        if (tr_n > 0 && trace_frames == 0) trace_dump();
        int hunting = uboot_sessions < cfg.max_sessions;
        /* module boot banner: if we see it but never get a prompt, our TX is probably not heard
         * (wrong polarity) -> flip and ask for another re-plug. Consoles without a getty only
         * talk during boot, so this is the only automatic polarity probe available then. */
        if (!boot_seen_ms && rx_total != boot_seen_rx && wlen > 0 &&
            (strcasestr(win, "u-boot") || strcasestr(win, "autoboot") || strcasestr(win, "hit any key"))) {
            boot_seen_ms = now; boot_seen_rx = rx_total;
            ev("module BOOT BANNER seen (invert_tx=%d); expecting U-Boot prompt from the Ctrl+C stream", cfg.invert_tx);
        }
        if (boot_seen_ms && hunting && !tx_confirmed && now - boot_seen_ms > 30000 && now - last_rx_ms > 5000) {
            boot_seen_ms = 0; boot_seen_rx = rx_total;
            if (cfg.autoflip) { flip_polarity("boot banner seen, no prompt, no reaction"); ev(">>> RE-PLUG THE MODULE NOW (TX polarity changed) <<<"); }
            else ev("boot banner seen but no prompt; autoflip disabled");
        }
        if (hunting && now >= next_ctrlc) {
            /* if the last line already looks like a prompt, hold fire: every Ctrl+C makes
             * U-Boot/getty reprint the prompt, so the line would never become quiescent */
            if (wlen > 0 && classify() != P_NONE) next_ctrlc = now + QUIET_MS + 100;
            else { ring_push(&tx_ring, cfg.hunt_byte); next_ctrlc = now + CTRLC_PERIOD_MS; }
        }
        if (now >= next_nudge) {
            /* provoke a prompt when the line is silent (module already booted before us) */
            if (now - last_rx_ms > NUDGE_PERIOD_MS && (uboot_sessions < cfg.max_sessions || linux_sessions < cfg.max_sessions)) tx_str("\r");
            next_nudge = now + NUDGE_PERIOD_MS;
        }
        if (rx_total != handled_at && wlen > 0 && now - last_rx_ms >= QUIET_MS) {
            int p = classify();
            if (p == P_NONE) { handled_at = rx_total; sleep_ms(5); continue; }
            if (p != P_UBOOT && now < blackout_until) { handled_at = rx_total; continue; }   /* rebooting: only U-Boot counts */
            ev("detected %s: \"%s\"", pname(p), last_line());
            handled_at = rx_total;
            if (p == P_CONT) {
                /* stuck ash continuation: EOF aborts the half-typed construct and returns to PS1 */
                ev("stuck shell continuation: sending Ctrl+D");
                tx_str("\x04\x04"); tx_wait_drained(2000); pump_ms(800); tx_str("\r"); pump_ms(500);
                continue;
            }
            if (p == P_UBOOT) {
                if (uboot_sessions >= cfg.max_sessions) { ev("U-Boot session limit reached, passive"); continue; }
                uboot_sessions++; confirm_tx("U-Boot prompt after Ctrl+C");
                pump_ms(300);
                run_cmd_file(cfg.cmds_uboot, "U-Boot");
                handled_at = rx_total;
            } else if (p == P_LOGIN) {
                if (linux_sessions >= cfg.max_sessions) { ev("Linux session limit reached, passive"); continue; }
                if (do_login()) { linux_sessions++; run_cmd_file(cfg.cmds_linux, "Linux"); }
                handled_at = rx_total;
            } else if (p == P_SHELL) {
                if (linux_sessions >= cfg.max_sessions) continue;
                linux_sessions++; confirm_tx("shell prompt");
                run_cmd_file(cfg.cmds_linux, "Linux");
                handled_at = rx_total;
            } else if (p == P_PASSWORD) {
                /* stray password prompt: send Enter to get back to login */
                tx_str("\r"); pump_ms(500); handled_at = rx_total;
            }
        }
        sleep_ms(2);
    }
    stats();
    ev("auto mode end");
    return 0;
}

/* ---------------------------------------------------------- console mode */
static struct termios tio_saved; static int tio_set;
static void tio_restore(void) { if (tio_set) tcsetattr(STDIN_FILENO, TCSANOW, &tio_saved); }
static void console_sigint(int s) { (void)s; ring_push(&tx_ring, 0x03); }
static int do_console(void) {
    quiet_stdout = 1;
    fprintf(stderr, "gpiouart console: baud=%d tx=pio%d rx=pio%d invert_tx=%d access=%s. Ctrl+] exits, Ctrl+C is forwarded.\n",
            cfg.baud, cfg.tx_pin, cfg.rx_pin, cfg.invert_tx, cfg.sim_path ? "SIM" : (use_mmio ? "MMIO" : "ioctl"));
    if (isatty(STDIN_FILENO)) {
        tcgetattr(STDIN_FILENO, &tio_saved); struct termios t = tio_saved; cfmakeraw(&t); t.c_lflag |= ISIG;
        tcsetattr(STDIN_FILENO, TCSANOW, &t); tio_set = 1; atexit(tio_restore);
    }
    signal(SIGINT, console_sigint);
    int fl = fcntl(STDIN_FILENO, F_GETFL, 0); fcntl(STDIN_FILENO, F_SETFL, fl | O_NONBLOCK);
    for (;;) {
        int v, n = 0;
        while ((v = ring_pop(&rx_ring)) >= 0) {
            if (v & RX_FLAG_FE) { char b[8]; int k = snprintf(b, sizeof b, "<%02X>", v & 0xff); if (write(STDOUT_FILENO, b, k) < 0) {} }
            else { unsigned char c = v; if (write(STDOUT_FILENO, &c, 1) < 0) {} }
            log_rx_byte(v); n++;
        }
        if (n && log_fp) fflush(log_fp);
        unsigned char c;
        ssize_t r = read(STDIN_FILENO, &c, 1);
        if (r == 1) { if (c == 29) break; ring_push(&tx_ring, c); }
        else if (r == 0) break;
        if (!n) sleep_ms(1);
    }
    fprintf(stderr, "\nconsole closed. rx_ok=%lu fe=%lu glitch=%lu tx=%lu\n", st_rx_ok, st_rx_fe, st_glitch, st_tx);
    return 0;
}

/* ------------------------------------------------------------------ main */
static void usage(const char *p) {
    printf("Usage: %s <selftest|console|auto> [options]\n"
           "  -b, --baud N           baud rate (115200)\n"
           "  -i, --invert-tx        start with inverted TX polarity\n"
           "      --invert-rx        inverted RX polarity\n"
           "      --tx-pin/--rx-pin/--dis-pin N   pio numbers (21/69/70)\n"
           "      --no-mmio          force gpiochip ioctl access\n"
           "      --cpu-rx/--cpu-tx N            cpu pinning (2/1), -1 = none\n"
           "  -o, --log FILE         append timestamped log\n"
           "      --mirror DEV       also write log to DEV (e.g. /dev/console)\n"
           "      --cmds-uboot FILE  commands to run at the U-Boot prompt\n"
           "      --cmds-linux FILE  commands to run at the Linux shell\n"
           "      --creds LIST       user:pass,user:pass (root:admin123,root:,admin:admin)\n"
           "      --state FILE       persist learned TX polarity\n"
           "      --reset-state      ignore saved state\n"
           "      --no-autoflip      never flip TX polarity automatically\n"
           "      --max-sessions N   U-Boot/Linux sessions per run (2)\n"
           "  -t, --timeout SEC      auto: exit after SEC seconds (0 = never)\n"
           "      --sim TTY          use a tty/pty instead of GPIO (testing)\n"
           "      --sample-frac F    where inside a register read the pin is captured, 0..1 (0.7)\n"
           "      --rx-shift BITS    extra RX sampling phase shift in bits, e.g. -0.2 / 0.2 (0)\n", p);
}
int main(int argc, char **argv) {
    static struct option lo[] = {
        {"baud", 1, 0, 'b'}, {"invert-tx", 0, 0, 'i'}, {"invert-rx", 0, 0, 1}, {"tx-pin", 1, 0, 2}, {"rx-pin", 1, 0, 3},
        {"dis-pin", 1, 0, 4}, {"no-mmio", 0, 0, 5}, {"cpu-rx", 1, 0, 6}, {"cpu-tx", 1, 0, 7}, {"log", 1, 0, 'o'},
        {"mirror", 1, 0, 8}, {"cmds-uboot", 1, 0, 9}, {"cmds-linux", 1, 0, 10}, {"creds", 1, 0, 11}, {"state", 1, 0, 12},
        {"reset-state", 0, 0, 13}, {"no-autoflip", 0, 0, 14}, {"max-sessions", 1, 0, 15}, {"timeout", 1, 0, 't'},
        {"sim", 1, 0, 16}, {"sample-frac", 1, 0, 17}, {"rx-shift", 1, 0, 18}, {"trace", 1, 0, 19}, {"rx-sampling", 0, 0, 20}, {"ctrlc-period", 1, 0, 21}, {"send-hex", 1, 0, 22}, {"send-thread", 0, 0, 23}, {"gate", 0, 0, 24}, {"swap", 0, 0, 25}, {"tx-trace", 0, 0, 26}, {"cmds-failsafe", 1, 0, 27}, {"initial-cmd", 1, 0, 28}, {"initial-baud", 1, 0, 29}, {"help", 0, 0, 'h'}, {0, 0, 0, 0} };
    if (argc < 2) { usage(argv[0]); return 1; }
    if (!strcmp(argv[1], "selftest")) cfg.mode = MODE_SELFTEST;
    else if (!strcmp(argv[1], "edges")) cfg.mode = MODE_EDGES;
    else if (!strcmp(argv[1], "console")) cfg.mode = MODE_CONSOLE;
    else if (!strcmp(argv[1], "auto")) cfg.mode = MODE_AUTO;
    else { usage(argv[0]); return 1; }
    optind = 2; int o;
    while ((o = getopt_long(argc, argv, "b:io:t:h", lo, NULL)) != -1) switch (o) {
        case 'b': cfg.baud = atoi(optarg); break; case 'i': cfg.invert_tx = 1; break; case 1: cfg.invert_rx = 1; break;
        case 2: cfg.tx_pin = atoi(optarg); break; case 3: cfg.rx_pin = atoi(optarg); break; case 4: cfg.dis_pin = atoi(optarg); break;
        case 5: cfg.want_mmio = 0; break; case 6: cfg.cpu_rx = atoi(optarg); break; case 7: cfg.cpu_tx = atoi(optarg); break;
        case 'o': cfg.log_path = optarg; break; case 8: cfg.mirror_path = optarg; break; case 9: cfg.cmds_uboot = optarg; break;
        case 10: cfg.cmds_linux = optarg; break; case 11: cfg.creds = optarg; break; case 12: cfg.state_path = optarg; break;
        case 13: cfg.reset_state = 1; break; case 14: cfg.autoflip = 0; break; case 15: cfg.max_sessions = atoi(optarg); break;
        case 't': cfg.timeout_s = atoi(optarg); break; case 16: cfg.sim_path = optarg; break;
        case 17: cfg.sample_frac = atof(optarg); break; case 18: cfg.rx_shift = atof(optarg); break;
        case 19: trace_frames = atoi(optarg); break; case 20: cfg.rx_sampling = 1; break;
        case 21: cfg.ctrlc_ms = atoi(optarg); break; case 22: cfg.hunt_byte = (int)strtol(optarg, NULL, 16) & 0xff; break;
        case 23: edges_send_thread = 1; break; case 24: edges_gate = 1; break; case 25: edges_swap = 1; break;
        case 26: tx_trace_on = 1; break; case 27: cfg.cmds_failsafe = optarg; break; case 28: cfg.initial_cmd = optarg; break;
        case 29: cfg.initial_baud = atoi(optarg); break;
        default: usage(argv[0]); return 1; }

    t_start_cyc = now_cyc();
    bit_cyc = (double)cyc_freq() / (double)cfg.baud;
    /* Linux RT throttling stalls a SCHED_FIFO spinner for 50 ms every second (sched_rt_runtime_us
     * = 950000): that cut TX bytes short mid-frame and produced periodic RX garbage. Disable it. */
    { FILE *f = fopen("/proc/sys/kernel/sched_rt_runtime_us", "w"); if (f) { fputs("-1\n", f); fclose(f); } }
    if (cfg.log_path) { log_fp = fopen(cfg.log_path, "a"); if (!log_fp) fprintf(stderr, "cannot open log %s: %s\n", cfg.log_path, strerror(errno)); }
    if (cfg.mirror_path) { mirror_fd = open(cfg.mirror_path, O_WRONLY | O_NOCTTY | O_APPEND); if (mirror_fd < 0) fprintf(stderr, "cannot open mirror %s: %s\n", cfg.mirror_path, strerror(errno)); }
    load_state();

    if (cfg.mode == MODE_SELFTEST) { if (cfg.sim_path) { printf("selftest is meaningless in --sim\n"); return 0; } return do_selftest(); }
    if (cfg.mode == MODE_EDGES) return do_edges();

    pthread_t th_rx, th_tx;
    if (cfg.sim_path) {
        sim_fd = open(cfg.sim_path, O_RDWR | O_NOCTTY);
        if (sim_fd < 0) { fprintf(stderr, "open %s: %s\n", cfg.sim_path, strerror(errno)); return 2; }
        struct termios t; if (tcgetattr(sim_fd, &t) == 0) { cfmakeraw(&t); tcsetattr(sim_fd, TCSANOW, &t); }
        pthread_create(&th_rx, NULL, rx_thread_sim, NULL); pthread_create(&th_tx, NULL, tx_thread_sim, NULL);
    } else {
        if (gpio_open() < 0) return 2;
        pthread_create(&th_rx, NULL, rx_thread_hw, NULL);
        /* the edge decoder transmits from the RX thread itself; only the legacy sampling
         * decoder still uses a separate TX thread */
        if (cfg.rx_sampling) pthread_create(&th_tx, NULL, tx_thread_hw, NULL); else th_tx = th_rx;
    }
    last_rx_ms = now_ms();
    int rc = cfg.mode == MODE_AUTO ? do_auto() : do_console();
    stop_flag = 1;
    if (cfg.sim_path) { pthread_cancel(th_rx); }
    if (th_tx != th_rx) pthread_join(th_tx, NULL);
    pthread_join(th_rx, NULL);
    if (!cfg.sim_path) tx_set(1);
    if (log_fp) fclose(log_fp);
    return rc;
}
