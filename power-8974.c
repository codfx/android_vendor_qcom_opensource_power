/*
 * Copyright (c) 2013, The Linux Foundation. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 * *    * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer in the documentation and/or other materials provided
 *       with the distribution.
 *     * Neither the name of The Linux Foundation nor the names of its
 *       contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
 * BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN
 * IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#define LOG_NIDEBUG 0

#include <dlfcn.h>
#include <errno.h>
#include <fcntl.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <time.h>
#include <unistd.h>

#define LOG_TAG "QTI PowerHAL"
#include <hardware/hardware.h>
#include <hardware/power.h>
#include <log/log.h>

#include "hint-data.h"
#include "metadata-defs.h"
#include "performance.h"
#include "power-common.h"
#include "utils.h"

/* Interactive governor overrides for display off and video (replacing the perf-lock resource lists). */
#define INTERACTIVE_NODE "/sys/devices/system/cpu/cpufreq/interactive/"
enum { TUN_TIMER_RATE, TUN_GO_HISPEED_LOAD, TUN_HISPEED_FREQ, TUN_IO_IS_BUSY, TUN_SYNC_THRESHOLD, NUM_TUN };
static const char* const tun_path[NUM_TUN] = {
        INTERACTIVE_NODE "timer_rate", INTERACTIVE_NODE "go_hispeed_load", INTERACTIVE_NODE "hispeed_freq",
        INTERACTIVE_NODE "io_is_busy", "/sys/module/cpu_boost/parameters/sync_threshold"};
/* Override sets, highest priority first; NULL = leave alone. */
enum { OVR_VIDEO_ENCODE, OVR_VIDEO_DECODE, OVR_DISPLAY_OFF, NUM_OVR };
static const char* const ovr_val[NUM_OVR][NUM_TUN] = {
        /* TR_MS_30, HISPEED_LOAD_90, HS_FREQ_1026, THREAD_MIGRATION_SYNC_OFF, INTERACTIVE_IO_BUSY_OFF */
        [OVR_VIDEO_ENCODE] = {"30000", "90", "1036800", "0", "0"},
        /* TR_MS_30, HISPEED_LOAD_90, HS_FREQ_1026, THREAD_MIGRATION_SYNC_OFF */
        [OVR_VIDEO_DECODE] = {"30000", "90", "1036800", NULL, "0"},
        /* TR_MS_50, THREAD_MIGRATION_SYNC_OFF */
        [OVR_DISPLAY_OFF] = {"50000", NULL, NULL, NULL, "0"},
};
static bool ovr_active[NUM_OVR];
static char tun_base[NUM_TUN][32];
static bool tun_saved[NUM_TUN];
static pthread_mutex_t tun_lock = PTHREAD_MUTEX_INITIALIZER;

static void set_override(int ovr, bool on) {
    pthread_mutex_lock(&tun_lock);
    if (on) {
        for (int t = 0; t < NUM_TUN; t++) {
            if (!ovr_val[ovr][t] || tun_saved[t]) continue;
            bool other = false;  // save the base value only while no other override owns this node
            for (int o = 0; o < NUM_OVR; o++)
                if (ovr_active[o] && ovr_val[o][t]) other = true;
            if (!other && sysfs_read(tun_path[t], tun_base[t], sizeof(tun_base[t]) - 1) == 0) {
                tun_base[t][strcspn(tun_base[t], "\n")] = '\0';
                tun_saved[t] = true;
            }
        }
    }
    ovr_active[ovr] = on;
    for (int t = 0; t < NUM_TUN; t++) {
        if (!ovr_val[ovr][t]) continue;
        const char* want = NULL;
        for (int o = 0; o < NUM_OVR && !want; o++)
            if (ovr_active[o] && ovr_val[o][t]) want = ovr_val[o][t];
        if (want) {
            sysfs_write(tun_path[t], (char*)want);
        } else if (tun_saved[t]) {
            sysfs_write(tun_path[t], tun_base[t]);
            tun_saved[t] = false;
        }
    }
    pthread_mutex_unlock(&tun_lock);
}

static int process_video_encode_hint(void* metadata) {
    char governor[80];
    struct video_encode_metadata_t video_encode_metadata;

    if (!metadata) return HINT_NONE;

    if (get_scaling_governor(governor, sizeof(governor)) == -1) {
        ALOGE("Can't obtain scaling governor.");
        return HINT_NONE;
    }

    /* Initialize encode metadata struct fields */
    memset(&video_encode_metadata, 0, sizeof(struct video_encode_metadata_t));
    video_encode_metadata.state = -1;
    video_encode_metadata.hint_id = DEFAULT_VIDEO_ENCODE_HINT_ID;

    if (parse_video_encode_metadata((char*)metadata, &video_encode_metadata) == -1) {
        ALOGE("Error occurred while parsing metadata.");
        return HINT_NONE;
    }

    if (video_encode_metadata.state == 1) {
        if (is_interactive_governor(governor)) {
            set_override(OVR_VIDEO_ENCODE, true);
            return HINT_HANDLED;
        }
    } else if (video_encode_metadata.state == 0) {
        if (is_interactive_governor(governor)) {
            set_override(OVR_VIDEO_ENCODE, false);
            return HINT_HANDLED;
        }
    }
    return HINT_NONE;
}

static int process_video_decode_hint(void* metadata) {
    char governor[80];
    struct video_decode_metadata_t video_decode_metadata;

    if (!metadata) return HINT_NONE;

    if (get_scaling_governor(governor, sizeof(governor)) == -1) {
        ALOGE("Can't obtain scaling governor.");
        return HINT_NONE;
    }

    /* Initialize decode metadata struct fields */
    memset(&video_decode_metadata, 0, sizeof(struct video_decode_metadata_t));
    video_decode_metadata.state = -1;
    video_decode_metadata.hint_id = DEFAULT_VIDEO_DECODE_HINT_ID;

    if (parse_video_decode_metadata((char*)metadata, &video_decode_metadata) == -1) {
        ALOGE("Error occurred while parsing metadata.");
        return HINT_NONE;
    }

    if (video_decode_metadata.state == 1) {
        if (is_interactive_governor(governor)) {
            set_override(OVR_VIDEO_DECODE, true);
            return HINT_HANDLED;
        }
    } else if (video_decode_metadata.state == 0) {
        if (is_interactive_governor(governor)) {
            set_override(OVR_VIDEO_DECODE, false);
            return HINT_HANDLED;
        }
    }
    return HINT_NONE;
}

/* The perf-lock resource lists (CPUS_ONLINE_MIN_* + min freq) are replaced by the direct boosts above. */

const int kDefaultInteractiveDuration = 500; /* ms */
const int kMinFlingDuration = 1500;          /* ms */
const int kMaxInteractiveDuration = 5000;    /* ms */
const int kMaxLaunchDuration = 5000;         /* ms */


/*
 * z3tc: boosts without mpdecision.
 *
 * On the Z3 Tablet Compact, bringing a CPU online while its cpufreq policy is being changed (which is what
 * mpdecision's perf locks do on every launch and touch: CPUS_ONLINE_MIN_* plus a raised min frequency) makes
 * the SoC reset (PS_HOLD drop). The device therefore runs without mpdecision, with all cores online, and this
 * HAL applies the same boosts itself through sysfs:
 *   - LAUNCH: scaling_min_freq = cpuinfo_max_freq on all online CPUs while the launch lasts (at most 5 s)
 *   - INTERACTION: scaling_min_freq = 1.5 GHz (0x20F..0x50F) for the hinted duration (500 ms .. 5 s)
 *   - display off / video encode / video decode: the interactive governor values of the perf-lock versions
 * It never changes a CPU's online state.
 */
#define NUM_CPUS 4
#define CPUFREQ_NODE "/sys/devices/system/cpu/cpu%d/cpufreq/%s"
#define INTERACTION_MIN_KHZ 1500000

static pthread_mutex_t boost_lock = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t boost_cond;
static bool boost_thread_started;
static bool launch_on;
static struct timespec launch_until, interaction_until;

static unsigned int read_cpu0_khz(const char* node) {
    char path[96], buf[32] = {0};
    snprintf(path, sizeof(path), CPUFREQ_NODE, 0, node);
    if (sysfs_read(path, buf, sizeof(buf) - 1) != 0) return 0;
    return (unsigned int)strtoul(buf, NULL, 10);
}

/* True only while all CPUs are online. A boost must never meet a CPU that is coming online (see above). */
static bool all_cpus_online(void) {
    char path[64], buf[4];
    for (int cpu = 1; cpu < NUM_CPUS; cpu++) {
        snprintf(path, sizeof(path), "/sys/devices/system/cpu/cpu%d/online", cpu);
        memset(buf, 0, sizeof(buf));
        if (sysfs_read(path, buf, sizeof(buf) - 1) != 0 || buf[0] != '1') return false;
    }
    return true;
}

static void write_min_freq_all(unsigned int khz, bool boost) {
    char path[96], val[16];
    if (boost && !all_cpus_online()) {
        ALOGW("Skipping boost: not all CPUs are online");
        return;
    }
    for (int cpu = 0; cpu < NUM_CPUS; cpu++) {
        unsigned int want = khz, max = 0;
        char buf[32] = {0};
        snprintf(path, sizeof(path), CPUFREQ_NODE, cpu, "scaling_min_freq");
        if (access(path, W_OK) != 0) continue;  // offline CPUs have no cpufreq node; skip them
        snprintf(path, sizeof(path), CPUFREQ_NODE, cpu, "scaling_max_freq");
        if (sysfs_read(path, buf, sizeof(buf) - 1) == 0) max = (unsigned int)strtoul(buf, NULL, 10);
        // A min above the current (thermally lowered) max is rejected by cpufreq; clamp to it instead.
        if (max && want > max) want = max;
        snprintf(path, sizeof(path), CPUFREQ_NODE, cpu, "scaling_min_freq");
        snprintf(val, sizeof(val), "%u", want);
        sysfs_write(path, val);
    }
}

static bool ts_before(struct timespec a, struct timespec b) {
    return a.tv_sec < b.tv_sec || (a.tv_sec == b.tv_sec && a.tv_nsec < b.tv_nsec);
}

static struct timespec ts_after_ms(int ms) {
    struct timespec t;
    clock_gettime(CLOCK_MONOTONIC, &t);
    t.tv_sec += ms / 1000;
    t.tv_nsec += (long)(ms % 1000) * 1000000L;
    if (t.tv_nsec >= 1000000000L) {
        t.tv_sec++;
        t.tv_nsec -= 1000000000L;
    }
    return t;
}

static void* boost_thread_fn(void* unused) {
    (void)unused;
    unsigned int max_khz = 0, base_khz = 0, applied = 0;
    pthread_mutex_lock(&boost_lock);
    for (;;) {
        struct timespec now;
        clock_gettime(CLOCK_MONOTONIC, &now);
        if (!max_khz) max_khz = read_cpu0_khz("cpuinfo_max_freq");
        if (!base_khz) base_khz = read_cpu0_khz("cpuinfo_min_freq");

        if (launch_on && !ts_before(now, launch_until)) launch_on = false;
        bool interaction_on = ts_before(now, interaction_until);
        unsigned int want = launch_on ? max_khz : interaction_on ? INTERACTION_MIN_KHZ : base_khz;
        if (want && want != applied) {
            write_min_freq_all(want, want != base_khz);
            applied = want;
        }

        if (launch_on || interaction_on) {
            struct timespec deadline = interaction_on ? interaction_until : launch_until;
            if (launch_on && interaction_on && ts_before(launch_until, interaction_until))
                deadline = launch_until;
            pthread_cond_timedwait(&boost_cond, &boost_lock, &deadline);
        } else {
            pthread_cond_wait(&boost_cond, &boost_lock);
        }
    }
    return NULL;
}

/* Called with boost_lock held. */
static void boost_kick_locked(void) {
    if (!boost_thread_started) {
        pthread_condattr_t attr;
        pthread_t thread;
        pthread_condattr_init(&attr);
        pthread_condattr_setclock(&attr, CLOCK_MONOTONIC);
        pthread_cond_init(&boost_cond, &attr);
        pthread_condattr_destroy(&attr);
        if (pthread_create(&thread, NULL, boost_thread_fn, NULL) != 0) {
            ALOGE("Failed to start the boost thread");
            return;
        }
        pthread_detach(thread);
        boost_thread_started = true;
    }
    pthread_cond_signal(&boost_cond);
}

static void direct_interaction_boost(int duration_ms) {
    pthread_mutex_lock(&boost_lock);
    struct timespec until = ts_after_ms(duration_ms);
    if (ts_before(interaction_until, until)) interaction_until = until;
    boost_kick_locked();
    pthread_mutex_unlock(&boost_lock);
}

static void direct_launch_boost(bool on) {
    pthread_mutex_lock(&boost_lock);
    launch_on = on;
    if (on) launch_until = ts_after_ms(kMaxLaunchDuration);
    boost_kick_locked();
    pthread_mutex_unlock(&boost_lock);
}


static void process_interaction_hint(void* data) {
    static struct timespec s_previous_boost_timespec;
    static int s_previous_duration = 0;

    struct timespec cur_boost_timespec;
    long long elapsed_time;
    int duration = kDefaultInteractiveDuration;

    if (data) {
        int input_duration = *((int*)data);
        if (input_duration > duration) {
            duration = (input_duration > kMaxInteractiveDuration) ? kMaxInteractiveDuration
                                                                  : input_duration;
        }
    }

    clock_gettime(CLOCK_MONOTONIC, &cur_boost_timespec);

    elapsed_time = calc_timespan_us(s_previous_boost_timespec, cur_boost_timespec);
    // don't hint if it's been less than 250ms since last boost
    // also detect if we're doing anything resembling a fling
    // support additional boosting in case of flings
    if (elapsed_time < 250000 && duration <= 750) {
        return;
    }
    s_previous_boost_timespec = cur_boost_timespec;
    s_previous_duration = duration;

    // Flings (>= kMinFlingDuration) used CPUS_ONLINE_MIN_3 instead of _2; with all cores online both are the
    // same 1.5 GHz min-frequency boost for the hinted duration.
    direct_interaction_boost(duration);
}

static int process_activity_launch_hint(void* data) {
    // data == NULL: the launch has finished, release the boost early (as the perf-lock version did)
    direct_launch_boost(data != NULL);
    return HINT_HANDLED;
}

int power_hint_override(power_hint_t hint, void* data) {
    int ret_val = HINT_NONE;
    switch (hint) {
        case POWER_HINT_VIDEO_ENCODE:
            ret_val = process_video_encode_hint(data);
            break;
        case POWER_HINT_VIDEO_DECODE:
            ret_val = process_video_decode_hint(data);
            break;
        case POWER_HINT_INTERACTION:
            process_interaction_hint(data);
            ret_val = HINT_HANDLED;
            break;
        case POWER_HINT_LAUNCH:
            ret_val = process_activity_launch_hint(data);
            break;
        default:
            break;
    }
    return ret_val;
}

int set_interactive_override(int on) {
    char governor[80];

    if (get_scaling_governor(governor, sizeof(governor)) == -1) {
        ALOGE("Can't obtain scaling governor.");
        return HINT_NONE;
    }

    if (!on) {
        /* Display off */
        /*
         * We need to be able to identify the first display off hint
         * and release the current lock holder
         */
        /* The 8974pro display-on CPUS_ONLINE_MIN_2 lock is gone (all cores stay online), nothing to release. */
        if (is_interactive_governor(governor)) set_override(OVR_DISPLAY_OFF, true);
    } else {
        /* Display on */
        if (is_interactive_governor(governor)) set_override(OVR_DISPLAY_OFF, false);
    }
    return HINT_HANDLED;
}
