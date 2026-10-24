// Support homing/probing "trigger" notification from analog sensors
//
// Copyright (C) 2025  Gareth Farrington <gareth@waves.ky>
// Copyright (C) 2024-2026  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include "basecmd.h" // oid_alloc
#include "board/io.h" // writeb
#include "board/misc.h" // timer_read_time
#include "command.h" // DECL_COMMAND
#include "sched.h" // shutdown
#include "sos_filter.h" // sos_filter_apply
#include "trigger_analog.h" // trigger_analog_update
#include "trsync.h" // trsync_do_trigger

// Diagnostic history: retain recent samples and snapshot the samples around
// the most recent trigger. This does not change trigger decisions.
#define TA_DEBUG_RING_SIZE 24
#define TA_DEBUG_POST_SAMPLES 8
#define TA_DEBUG_CAPTURE_SIZE (TA_DEBUG_RING_SIZE + TA_DEBUG_POST_SAMPLES)
struct trigger_analog_sample { uint32_t time; int32_t raw, filtered; };

// Main trigger_analog storage
struct trigger_analog {
    // Raw value range check
    int32_t raw_min, raw_max;
    // Filtering
    struct sos_filter *sf;
    // Trigger value checking
    int32_t trigger_value, trigger_peak;
    uint8_t trigger_type;
    uint8_t trigger_confirm_count;
    // Trsync triggering
    uint8_t flags, trigger_reason, error_reason;
    struct trsync *ts;
    uint32_t homing_clock;
    // Sensor activity monitoring
    uint8_t monitor_max, monitor_count;
    struct timer time;
    uint32_t monitor_ticks;
    // Diagnostic sample history
    struct trigger_analog_sample debug_ring[TA_DEBUG_RING_SIZE];
    uint8_t debug_ring_count, debug_ring_pos;
    struct trigger_analog_sample debug_capture[TA_DEBUG_CAPTURE_SIZE];
    uint8_t debug_capture_count, debug_trigger_index, debug_post_remaining;
    uint8_t debug_capture_triggered;
    uint8_t debug_capture_valid;
    uint32_t debug_trigger_count, debug_last_trigger_time;
    // Stream diagnostic output gradually to avoid overflowing the UART TX
    // buffer (96 bytes on the STM32 serial driver).
    struct trigger_analog_sample debug_dump_samples[TA_DEBUG_CAPTURE_SIZE];
    struct timer debug_dump_timer;
    uint32_t debug_dump_request_id, debug_dump_trigger_count;
    uint32_t debug_dump_last_trigger_time;
    int32_t debug_dump_trigger_value;
    uint8_t debug_dump_oid, debug_dump_count, debug_dump_pos;
    uint8_t debug_dump_source, debug_dump_trigger_index;
    uint8_t debug_dump_triggered, debug_dump_snapshot_valid;
    uint8_t debug_dump_done_sent;
};

// Homing flags
enum {
    TA_AWAIT_HOMING = 1<<1, TA_CAN_TRIGGER = 1<<2
};

// Trigger types
enum {
    TT_ABS_GE, TT_GT, TT_DIFF_PEAK_GT
};
#define TA_ABS_GE_CONFIRM_SAMPLES 3

DECL_ENUMERATION("trigger_analog_type", "abs_ge", TT_ABS_GE);
DECL_ENUMERATION("trigger_analog_type", "gt", TT_GT);
DECL_ENUMERATION("trigger_analog_type", "diff_peak_gt", TT_DIFF_PEAK_GT);

// Sample errors sent via trsync error code
enum {
    TE_RAW_RANGE, TE_OVERFLOW, TE_MONITOR, TE_SENSOR_SPECIFIC
};
DECL_ENUMERATION("trigger_analog_error:", "RAW_RANGE", TE_RAW_RANGE);
DECL_ENUMERATION("trigger_analog_error:", "OVERFLOW", TE_OVERFLOW);
DECL_ENUMERATION("trigger_analog_error:", "MONITOR", TE_MONITOR);
DECL_ENUMERATION("trigger_analog_error:", "SENSOR_SPECIFIC"
                 , TE_SENSOR_SPECIFIC);

// Timer callback that monitors for sensor timeouts
static uint_fast8_t
monitor_event(struct timer *t)
{
    struct trigger_analog *ta = container_of(t, struct trigger_analog, time);

    if (!(ta->flags & TA_CAN_TRIGGER))
        return SF_DONE;

    if (ta->monitor_count > ta->monitor_max) {
        trsync_do_trigger(ta->ts, ta->error_reason + TE_MONITOR);
        return SF_DONE;
    }

    // A sample was recently delivered, continue monitoring
    ta->monitor_count++;
    ta->time.waketime += ta->monitor_ticks;
    return SF_RESCHEDULE;
}

// Note recent activity
static void
monitor_note_activity(struct trigger_analog *ta)
{
    writeb(&ta->monitor_count, 0);
}

// Check if a value should signal a "trigger" event
static int
check_trigger(struct trigger_analog *ta, uint32_t time, int32_t value)
{
    switch (ta->trigger_type) {
    case TT_ABS_GE: ;
        uint32_t abs_value = value < 0 ? -value : value;
        return abs_value >= ta->trigger_value;
    case TT_GT:
        return value > ta->trigger_value;
    case TT_DIFF_PEAK_GT:
        if (value > ta->trigger_peak) {
            ta->trigger_peak = value;
            return 0;
        }
        uint32_t delta = ta->trigger_peak - value;
        return delta > ta->trigger_value;
    }
    return 0;
}

static void
debug_record_ring(struct trigger_analog *ta, uint32_t time, int32_t raw,
                  int32_t filtered)
{
    uint8_t pos = ta->debug_ring_pos;
    ta->debug_ring[pos].time = time;
    ta->debug_ring[pos].raw = raw;
    ta->debug_ring[pos].filtered = filtered;
    ta->debug_ring_pos = (pos + 1) % TA_DEBUG_RING_SIZE;
    if (ta->debug_ring_count < TA_DEBUG_RING_SIZE)
        ta->debug_ring_count++;
}

static void
debug_save_trigger(struct trigger_analog *ta, uint32_t time)
{
    uint8_t count = ta->debug_ring_count;
    uint8_t start = (ta->debug_ring_pos + TA_DEBUG_RING_SIZE - count)
                    % TA_DEBUG_RING_SIZE;
    uint8_t i;
    for (i = 0; i < count; i++)
        ta->debug_capture[i] = ta->debug_ring[(start + i) % TA_DEBUG_RING_SIZE];
    ta->debug_capture_count = count;
    ta->debug_trigger_index = count ? count - 1 : 0;
    ta->debug_capture_triggered = 1;
    ta->debug_post_remaining = TA_DEBUG_POST_SAMPLES;
    ta->debug_capture_valid = 1;
    ta->debug_trigger_count++;
    ta->debug_last_trigger_time = time;
}

static void
debug_record_post(struct trigger_analog *ta, uint32_t time, int32_t raw,
                  int32_t filtered)
{
    if (!ta->debug_post_remaining)
        return;
    if (ta->debug_capture_count < TA_DEBUG_CAPTURE_SIZE) {
        uint8_t pos = ta->debug_capture_count++;
        ta->debug_capture[pos].time = time;
        ta->debug_capture[pos].raw = raw;
        ta->debug_capture[pos].filtered = filtered;
    }
    ta->debug_post_remaining--;
}

// Reset fields associated with trigger checking
static void
trigger_reset(struct trigger_analog *ta)
{
    ta->trigger_peak = INT32_MIN;
    ta->trigger_confirm_count = 0;
}

// Stop homing due to an error
static void
cancel_homing(struct trigger_analog *ta, uint8_t error_code)
{
    if (!(ta->flags & TA_CAN_TRIGGER))
        return;
    ta->flags = 0;
    trsync_do_trigger(ta->ts, ta->error_reason + error_code);
}

// Handle an error reported by the sensor
void
trigger_analog_note_error(struct trigger_analog *ta, uint8_t sensor_code)
{
    if (!ta)
        return;
    cancel_homing(ta, sensor_code + TE_SENSOR_SPECIFIC);
}

// Used by Sensors to report new raw ADC sample
void
trigger_analog_update(struct trigger_analog *ta, int32_t sample)
{
    // Check homing is active
    if (!ta)
        return;
    uint8_t flags = ta->flags;
    // Continue filtering briefly after a trigger to capture post-trigger data.
    int active = !!(flags & TA_CAN_TRIGGER);
    if (!active && !ta->debug_post_remaining)
        return;

    // Check if homing has started
    uint32_t time = timer_read_time();
    if (active) {
        if ((flags & TA_AWAIT_HOMING)
            && timer_is_before(time, ta->homing_clock))
            return;
        flags &= ~TA_AWAIT_HOMING;
        // Reset the sensor timeout checking
        monitor_note_activity(ta);
    }

    // Check that raw value is in range
    if (sample < ta->raw_min || sample > ta->raw_max) {
        if (active)
            cancel_homing(ta, TE_RAW_RANGE);
        return;
    }

    // Perform filtering
    int32_t filtered_value = sample;
    int ret = sos_filter_apply(ta->sf, &filtered_value);
    if (ret) {
        if (active)
            cancel_homing(ta, TE_OVERFLOW);
        return;
    }

    if (active)
        debug_record_ring(ta, time, sample, filtered_value);
    if (!active || ta->debug_post_remaining)
        debug_record_post(ta, time, sample, filtered_value);

    // Check if this is a "trigger"
    if (active) {
        ret = check_trigger(ta, time, filtered_value);
        // For absolute load-cell thresholds, require three consecutive
        // samples over threshold to reject single-sample spikes. Other
        // trigger types retain their existing behavior.
        if (ta->trigger_type == TT_ABS_GE) {
            if (ret) {
                if (ta->trigger_confirm_count < TA_ABS_GE_CONFIRM_SAMPLES)
                    ta->trigger_confirm_count++;
                ret = ta->trigger_confirm_count >= TA_ABS_GE_CONFIRM_SAMPLES;
            } else {
                ta->trigger_confirm_count = 0;
            }
        }
        if (ret) {
            debug_save_trigger(ta, time);
            trsync_do_trigger(ta->ts, ta->trigger_reason);
            flags = 0;
            ta->homing_clock = time;
        }
        ta->flags = flags;
    }
}

// Create a trigger_analog
void
command_config_trigger_analog(uint32_t *args)
{
    struct trigger_analog *ta = oid_alloc(
        args[0], command_config_trigger_analog, sizeof(*ta));
    ta->sf = sos_filter_oid_lookup(args[1]);
}
DECL_COMMAND(command_config_trigger_analog
             , "config_trigger_analog oid=%c sos_filter_oid=%c");

// Lookup a trigger_analog
struct trigger_analog *
trigger_analog_oid_lookup(uint8_t oid)
{
    return oid_lookup(oid, command_config_trigger_analog);
}

// Set valid raw range
void
command_trigger_analog_set_raw_range(uint32_t *args)
{
    struct trigger_analog *ta = trigger_analog_oid_lookup(args[0]);
    ta->raw_min = args[1];
    ta->raw_max = args[2];
}
DECL_COMMAND(command_trigger_analog_set_raw_range,
    "trigger_analog_set_raw_range oid=%c raw_min=%i raw_max=%i");

// Set the triggering type and value
void
command_trigger_analog_set_trigger(uint32_t *args)
{
    struct trigger_analog *ta = trigger_analog_oid_lookup(args[0]);
    ta->trigger_type = args[1];
    ta->trigger_value = args[2];
}
DECL_COMMAND(command_trigger_analog_set_trigger, "trigger_analog_set_trigger"
    " oid=%c trigger_analog_type=%c trigger_value=%i");

// Home an axis
void
command_trigger_analog_home(uint32_t *args)
{
    struct trigger_analog *ta = trigger_analog_oid_lookup(args[0]);
    sched_del_timer(&ta->time);
    ta->monitor_ticks = args[5];
    if (!ta->monitor_ticks) {
        ta->flags = 0;
        ta->ts = NULL;
        return;
    }
    ta->ts = trsync_oid_lookup(args[1]);
    ta->trigger_reason = args[2];
    ta->error_reason = args[3];
    ta->time.waketime = ta->homing_clock = args[4];
    ta->monitor_max = args[6];
    ta->monitor_count = 0;
    ta->time.func = monitor_event;
    ta->flags = TA_AWAIT_HOMING | TA_CAN_TRIGGER;
    // Start a fresh sample ring for each move. Preserve the last trigger
    // snapshot until the next actual trigger overwrites it.
    ta->debug_post_remaining = 0;
    ta->debug_ring_count = ta->debug_ring_pos = 0;
    trigger_reset(ta);
    sched_add_timer(&ta->time);
}
DECL_COMMAND(command_trigger_analog_home,
             "trigger_analog_home oid=%c trsync_oid=%c trigger_reason=%c"
             " error_reason=%c clock=%u monitor_ticks=%u monitor_max=%u");

void
command_trigger_analog_query_state(uint32_t *args)
{
    uint8_t oid = args[0];
    struct trigger_analog *ta = trigger_analog_oid_lookup(args[0]);
    sendf("trigger_analog_state oid=%c homing=%c homing_clock=%u"
          , oid, !!(ta->flags & TA_CAN_TRIGGER), ta->homing_clock);
}
DECL_COMMAND(command_trigger_analog_query_state
             , "trigger_analog_query_state oid=%c");

// Emit one diagnostic message per timer event. A burst of up to 33 sendf()
// messages can exceed the STM32 UART driver's 96-byte TX buffer; sendf() then
// drops messages without reporting an error. Send metadata first, then pace
// the sample messages so the UART can drain between them.
static uint_fast8_t
debug_dump_event(struct timer *t)
{
    struct trigger_analog *ta = container_of(
        t, struct trigger_analog, debug_dump_timer);
    if (!ta->debug_dump_done_sent) {
        sendf("trigger_analog_debug_done oid=%c request_id=%u count=%c source=%c"
              " trigger_value=%i trigger_count=%u last_trigger_time=%u"
              " snapshot_valid=%c", ta->debug_dump_oid,
              ta->debug_dump_request_id, ta->debug_dump_count,
              ta->debug_dump_source, ta->debug_dump_trigger_value,
              ta->debug_dump_trigger_count, ta->debug_dump_last_trigger_time,
              ta->debug_dump_snapshot_valid);
        ta->debug_dump_done_sent = 1;
        ta->debug_dump_timer.waketime = timer_read_time() + timer_from_us(2000);
        return SF_RESCHEDULE;
    }
    uint8_t pos = ta->debug_dump_pos;
    if (pos >= ta->debug_dump_count)
        return SF_DONE;
    struct trigger_analog_sample *s = &ta->debug_dump_samples[pos];
    sendf("trigger_analog_debug_sample oid=%c request_id=%u index=%c time=%u raw=%i"
          " filtered=%i triggered=%c", ta->debug_dump_oid,
          ta->debug_dump_request_id, pos, s->time, s->raw, s->filtered,
          ta->debug_dump_triggered && pos == ta->debug_dump_trigger_index);
    ta->debug_dump_pos++;
    ta->debug_dump_timer.waketime = timer_read_time() + timer_from_us(2000);
    return SF_RESCHEDULE;
}

void
command_trigger_analog_dump(uint32_t *args)
{
    uint8_t oid = args[0];
    struct trigger_analog *ta = trigger_analog_oid_lookup(oid);
    ta->debug_dump_oid = oid;
    ta->debug_dump_request_id = args[1];
    ta->debug_dump_trigger_value = ta->trigger_value;
    ta->debug_dump_trigger_count = ta->debug_trigger_count;
    ta->debug_dump_last_trigger_time = ta->debug_last_trigger_time;
    ta->debug_dump_snapshot_valid = ta->debug_capture_valid;
    if (ta->debug_capture_valid) {
        ta->debug_dump_count = ta->debug_capture_count;
        ta->debug_dump_source = 1;
        ta->debug_dump_triggered = ta->debug_capture_triggered;
        ta->debug_dump_trigger_index = ta->debug_trigger_index;
        uint8_t i;
        for (i = 0; i < ta->debug_dump_count; i++)
            ta->debug_dump_samples[i] = ta->debug_capture[i];
    } else {
        uint8_t count = ta->debug_ring_count;
        uint8_t start = (ta->debug_ring_pos + TA_DEBUG_RING_SIZE - count)
                        % TA_DEBUG_RING_SIZE;
        ta->debug_dump_count = count;
        ta->debug_dump_source = 0;
        ta->debug_dump_triggered = 0;
        ta->debug_dump_trigger_index = 0;
        uint8_t i;
        for (i = 0; i < count; i++)
            ta->debug_dump_samples[i] =
                ta->debug_ring[(start + i) % TA_DEBUG_RING_SIZE];
    }
    ta->debug_dump_pos = 0;
    ta->debug_dump_done_sent = 0;
    ta->debug_dump_timer.func = debug_dump_event;
    ta->debug_dump_timer.waketime = timer_read_time() + timer_from_us(2000);
    sched_add_timer(&ta->debug_dump_timer);
}
DECL_COMMAND(command_trigger_analog_dump,
             "trigger_analog_dump oid=%c request_id=%u");
