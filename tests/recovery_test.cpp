// SPDX-License-Identifier: GPL-3.0-only
#include <cassert>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include "../components/ecowitt_rf/sensor/rain_recovery.h"
using namespace esphome::ecowitt_rf;
static void eq(float a, float b) { if (std::fabs(a - b) >= 0.02f) { std::fprintf(stderr, "actual %f expected %f\n", a, b); std::abort(); } }
int main() {
    constexpr uint32_t epoch = 1792160000, key = 0x12345678;
    RainWindow24h day; RainWindow10m recent;
    day.sample(100, 1000); recent.sample(100, 1000);
    day.sample(101.2f, 61000); recent.sample(101.2f, 61000);
    RainCheckpoint saved;
    saved.key = key; saved.unix_time = epoch;
    saved.rolling = day.state(); saved.recent = recent.state();
    assert(saved.valid());
    RainWindow24h resumed; RainWindow10m resumed_recent;
    assert(!saved.restore(key, 0, 100, resumed, resumed_recent)); // pre-NTP
    assert(!saved.restore(key + 1, epoch, 100, resumed, resumed_recent));
    assert(saved.restore(key, epoch + 60, 100, resumed, resumed_recent));
    eq(resumed.sample(101.2f, 100), 1.2f);
    eq(resumed_recent.sample(101.2f, 100), 1.2f);
    eq(resumed_recent.sample(101.2f, 600100), 0);
    eq(resumed.sample(101.2f, 600100), 1.2f);
    assert(saved.restore(key, epoch + 90000, UINT32_MAX - 10, resumed, resumed_recent));
    eq(resumed.sample(101.2f, 50), 0); // expiry + millis wrap
    eq(resumed_recent.sample(101.2f, 50), 0);
    auto corrupt = saved; corrupt.rolling.head = RainWindow24h::SLOTS;
    assert(!corrupt.restore(key, epoch, 1, resumed, resumed_recent));
    corrupt = saved; corrupt.recent.phase_ms = RainWindow10m::SLOT_MS;
    assert(!corrupt.valid());
    // Sparse recorder points, counter reset, old rain expiry, recent rain rate.
    const RainHistoryPoint points[] = {{epoch - 90000, 90}, {epoch - 89500, 90.3f},
        {epoch - 3600, 91.5f}, {epoch - 3000, 0}, {epoch - 120, 0.2f}};
    assert(restore_rain_history(points, 5, 0.2f, epoch, 30, resumed, resumed_recent));
    eq(resumed.sample(0.2f, 30), 1.4f); // expired old0.3, preserved1.2 + new0.2
    eq(resumed_recent.sample(0.2f, 30), 0.2f);
    assert(!restore_rain_history(points, 5, 0.3f, epoch, 30, resumed, resumed_recent));
    assert(!restore_rain_history(points, 5, 0.2f, epoch - 1000, 30, resumed, resumed_recent));
    auto invalid = points[0]; invalid.counter_mm = NAN;
    assert(!restore_rain_history(&invalid, 1, 0, epoch, 30, resumed, resumed_recent));
    // First counter after a restart adds only the newly observed delta.
    assert(saved.restore(key, epoch + 30, 10, resumed, resumed_recent));
    eq(resumed.sample(101.3f, 20), 1.3f);
    std::printf("Rain RTC/history recovery passed; checkpoint=%zu bytes\n", sizeof(RainCheckpoint));
}
