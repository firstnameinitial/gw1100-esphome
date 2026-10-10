// SPDX-License-Identifier: GPL-3.0-only
#include <cassert>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include "../components/ecowitt_rf/sensor/fineoffset.h"

using namespace esphome::ecowitt_rf;
static void eq(float actual, float expected) {
    if (std::fabs(actual - expected) > 0.02f) {
        std::fprintf(stderr, "actual %.4f != expected %.4f\n", actual, expected);
        std::abort();
    }
}

int main() {
    // Continuous RF reports formerly overwrote the only baseline forever.
    RainWindow24h window;
    eq(window.sample(100, 0), 0);
    for (uint32_t t = 8500; t < 3600000; t += 8500)
        eq(window.sample(t < 60000 ? 100 : 100.3f, t), t < 60000 ? 0 : 0.3f);
    eq(window.sample(101.3f, 3600000), 1.3f);
    // Cross local midnight: the elapsed-time window does not reset.
    eq(window.sample(101.3f, 12 * 3600000UL), 1.3f);
    // First rain expires at its half-hour boundary, later rain stays.
    eq(window.sample(101.3f, 49 * RainWindow24h::SLOT_MS), 1.0f);
    eq(window.sample(101.3f, 51 * RainWindow24h::SLOT_MS), 0);

    RainWindow24h reset;
    reset.sample(250, 1000);
    eq(reset.sample(250.5f, 2000), 0.5f);
    eq(reset.sample(0, 3000), 0.5f);  // sensor resets, real recent rain remains
    eq(reset.sample(0.2f, 4000), 0.7f);
    eq(reset.sample(NAN, 5000), 0.7f);
    eq(reset.sample(-1, 6000), 0.7f);
    eq(reset.sample(0.2f, 3 * 86400000UL), 0);  // long dry/reporting gap
    eq(reset.sample(10.2f, 6 * 86400000UL), 0); // do not label old, undated rain as recent

    RainWindow24h wrap;
    wrap.sample(10, UINT32_MAX - 1000);
    eq(wrap.sample(10.3f, 2000), 0.3f);
    eq(wrap.sample(10.3f, 90000000UL), 0);

    RainWindow24h calibration;
    calibration.sample(100, 0);
    for (unsigned i = 1; i <= 8; ++i)
        calibration.sample(100 + i * 0.125f, i * 1000);
    eq(calibration.sample(101, 10000), 1);  // no per-delta rounding drift

    RainAggregator rain;
    RainAggregate a;
    auto sample = [&](uint32_t id, float counter, uint32_t t, uint16_t day = 249) {
        rain.sample((uint8_t)Family::WS90, id, counter, day, 6, 35, 9, 2026, t, a);
    };
    sample(1, 100, 1000);
    sample(2, 500, 1500);  // independent sensor, not another 400mm of rainfall
    sample(1, 100.3f, 10000);
    eq(a.last24h_mm, 0.3f);
    assert(a.rate_mmh > 0);
    sample(2, 500, 11000);
    eq(a.last24h_mm, 0);
    sample(1, 100.3f, 19000);
    eq(a.rate_mmh, 1.8f);  // persists between RF reports and scheduled HTTP uploads
    sample(1, 100.3f, 610000);
    eq(a.rate_mmh, 0);  // clears after the short window, despite a wet-surface flag
    sample(1, 100.3f, 12 * 3600000UL, 250);
    eq(a.day_mm, 0);
    eq(a.last24h_mm, 0.3f);
    rain.reset();
    sample(3, 200, 13 * 3600000UL);
    eq(a.last24h_mm, 0);
    eq(a.day_mm, 0);

    // Restored totals survive RF reception before NTP, then same-day clock sync.
    RainAggregator resumed;
    resumed.set_count(1);
    auto &b = resumed.buckets()[0];
    b.key = ((uint32_t)Family::WS90 << 24) | 1;
    b.last_counter = 111.1f;
    b.event_mm = b.day_mm = b.week_mm = b.month_mm = 0.3f;
    b.year_mm = 111.1f;
    b.restore_calendar(249, 9, 2026);
    resumed.sample((uint8_t)Family::WS90, 1, 111.1f, 0, 0, 0, 0, 0, 1000, a);
    eq(a.day_mm, 0.3f); eq(a.month_mm, 0.3f); eq(a.year_mm, 111.1f);
    resumed.sample((uint8_t)Family::WS90, 1, 111.1f, 249, 14, 35, 9, 2026, 2000, a);
    eq(a.day_mm, 0.3f); eq(a.week_mm, 0.3f); eq(a.month_mm, 0.3f);
    resumed.sample((uint8_t)Family::WS90, 1, 111.1f, 250, 0, 35, 9, 2026, 3000, a);
    eq(a.day_mm, 0); eq(a.week_mm, 0.3f); eq(a.month_mm, 0.3f);

    // WS90's wet-surface flag is independent of measurable rainfall; preserve it.
    uint8_t packet[32] = {0x90, 0, 0x3f, 0x0c};
    packet[16] = 0x10;
    packet[19] = 0x04; packet[20] = 0x57;  // 111.1mm lifetime counter
    packet[30] = crc8(0x31, packet, 30);
    packet[31] = add_sum(packet, 31);
    Reading reading;
    assert(decode_ws90(packet, sizeof(packet), reading));
    assert(reading.rain_state);
    eq(reading.rain_mm, 111.1f);
    eq(reading.rain_rate_mmh, 0);
    static_assert(sizeof(RainWindow24h) <= 116, "ESP8266 window memory budget");
    std::printf("Rain regressions passed; window=%zu bytes, aggregator=%zu bytes\n",
                sizeof(RainWindow24h), sizeof(RainAggregator));
}
