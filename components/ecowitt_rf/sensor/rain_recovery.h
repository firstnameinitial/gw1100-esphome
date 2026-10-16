// SPDX-License-Identifier: GPL-3.0-only
#pragma once
#include "rain_window.h"

namespace esphome::ecowitt_rf {

// Primary rain sensor, matching the existing flash-persisted calendar bucket.
// Stored in ESPHome's allocator-owned RTC preference region, never eboot/crash slots.
// No pointers/millis timestamps are persisted; wall-clock time ages the windows on boot.
struct RainCheckpoint {
    uint32_t key = 0, unix_time = 0;
    RainWindow24h::State rolling;
    RainWindow10m::State recent;

    bool valid() const {
        return key != 0 && unix_time >= 1704067200UL &&
               rolling.initialized && recent.initialized &&
               rolling.head < RainWindow24h::SLOTS && rolling.phase_ms < RainWindow24h::SLOT_MS &&
               recent.head < RainWindow10m::SLOTS && recent.phase_ms < RainWindow10m::SLOT_MS;
    }
    bool restore(uint32_t sensor_key, uint32_t now_unix, uint32_t now_ms,
                 RainWindow24h &day, RainWindow10m &rate) const {
        if (!valid() || sensor_key != key || now_unix < unix_time) return false;
        // Saturation preserves expiry while avoiding overflow after a long outage.
        uint32_t seconds = now_unix - unix_time;
        uint32_t elapsed = seconds >= 90000UL ? 90000000UL : seconds * 1000UL;
        return day.restore(rolling, elapsed, now_ms) && rate.restore(recent, elapsed, now_ms);
    }
};
static_assert(sizeof(RainCheckpoint) <= 180, "RTC rain checkpoint budget");

struct RainHistoryPoint { uint32_t unix_time; float counter_mm; };

// Explicit recovery from an external recorder, including migration from pre-checkpoint
// firmware. Calendar/lifetime counters are not modified. Reject incomplete/reordered or
// stale input rather than inventing rainfall or replacing a live sensor's newer counter.
inline bool restore_rain_history(const RainHistoryPoint *points, size_t count,
                                 float current_counter, uint32_t now_unix, uint32_t now_ms,
                                 RainWindow24h &day, RainWindow10m &recent) {
    if (!points || count < 2 || count > 64 || now_unix < 1704067200UL) return false;
    for (size_t i = 0; i < count; ++i) {
        if (!std::isfinite(points[i].counter_mm) || points[i].counter_mm < 0 ||
            points[i].unix_time > now_unix || now_unix - points[i].unix_time > 90000UL ||
            (i && points[i].unix_time <= points[i - 1].unix_time)) return false;
    }
    if (!std::isfinite(current_counter) ||
        std::fabs(points[count - 1].counter_mm - current_counter) > 0.02f) return false;
    RainWindow24h restored_day;
    RainWindow10m restored_recent;
    for (size_t i = 0; i < count; ++i) {
        uint32_t at = now_ms - (now_unix - points[i].unix_time) * 1000UL;
        // Recorder history omits unchanged packets. Age the previous baseline up
        // to the observed change before sampling it, so a long dry gap does not
        // discard a newly dated increment as undated offline accumulation.
        if (i) {
            restored_day.sample(points[i - 1].counter_mm, at - 1);
            restored_recent.sample(points[i - 1].counter_mm, at - 1);
        }
        restored_day.sample(points[i].counter_mm, at);
        restored_recent.sample(points[i].counter_mm, at);
    }
    restored_day.sample(current_counter, now_ms);
    restored_recent.sample(current_counter, now_ms);
    day = restored_day; recent = restored_recent;
    return true;
}

}  // namespace esphome::ecowitt_rf
