// SPDX-License-Identifier: GPL-3.0-only
// Modified C++ port of rtl_433 (GPL-2.0-or-later), 2026-10-10.
// Adapted sensor payload parsing, bounds checks and ESPHome reading aggregation.
// Original copyright notices: THIRD_PARTY_NOTICES.md.
// Fine Offset / Ecowitt sensor packet decoders (C++ port of the rtl_433 reference
// decoders in rtl_433 src/devices/). The CMT2300A does the FSK demod + preamble/sync
// detection in packet mode; this parses the payload bytes + verifies integrity.
//
// Radio layer: 2-FSK, ~58 µs NRZ bit (~17.2 kbit/s), preamble AA AA..., sync 2D D4.
// The FIFO payload passed to decode() starts at the family/type byte (the CMT2300A
// packet mode strips preamble/sync) and is
// zero-padded up to the fixed 69-byte drain — so every decoder below checks a MINIMUM
// length and validates integrity over its own exact byte range.
//
// Integrity (per family, exactly as the rtl_433 reference): CRC-8 poly 0x31 init 0x00
// (compared against the CRC byte, or computed including it for a zero residue) plus a
// trailing additive SUM-8; WH25 additionally has the nibble-swapped XOR bitsum.
//
// Each decoder cites the rtl_433 source lines it was ported from.
#pragma once
#include <cstdint>
#include <cstddef>
#include <cstring>
#include "rain_window.h"

namespace esphome {
namespace ecowitt_rf {

// ---- CRC / checksums ----------------------------------------------------------
inline uint8_t crc8(uint8_t poly, const uint8_t *d, size_t n) {
    uint8_t crc = 0;
    for (size_t i = 0; i < n; ++i) {
        crc ^= d[i];
        for (int b = 0; b < 8; ++b)
            crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ poly) : (uint8_t)(crc << 1);
    }
    return crc;
}

inline uint8_t add_sum(const uint8_t *d, size_t n) {
    uint8_t s = 0;
    for (size_t i = 0; i < n; ++i) s += d[i];
    return s;
}

// XOR of the bytes, then nibbles of the result swapped (rtl_433 xor_bytes + swap,
// fineoffset.c:661-662). XOR commutes with the nibble swap, so this equals the XOR of
// the nibble-swapped bytes.
inline uint8_t xor_sum_swapped(const uint8_t *d, size_t n) {
    uint8_t s = 0;
    for (size_t i = 0; i < n; ++i) s ^= d[i];
    return (uint8_t)(((s & 0x0F) << 4) | (s >> 4));
}

// ---- Decoded sensor payload (a tagged union for the families we support) ------
enum class Family : uint8_t {
    Unknown = 0,
    WH25,    // also WH32 (0xD*) / WH32B — indoor T/H/P
    WH31,    // also WH31E (0x30) / WH31B (0x37) — 8-ch outdoor T/H
    WH51,    // soil moisture
    WH24,    // 7-in-1 array
    WH65,    // WH24 variant (0x65 type byte per GW1100 brief)
    WH0290,  // also WH41 — PM2.5/PM10
    WS69,    // WH24-family array with pressure extension
    WH40,    // rain gauge
    WN20,    // rain gauge (0.3 mm tips)
    WS68,    // solar anemometer w/ LUX+UVI (GW1100 names this "wh68")
    WH43,    // air quality, 24-bit ID
    WH45,    // CO2/PM (also WH0295)
    WH46,    // air quality + PM1/PM4
    WH52,    // soil moisture/temp/EC
    WH55,    // water leak
    WH54,    // leak (UNVERIFIED stub)
    WH57,    // lightning (rtl_433 WH31L)
    WN34,    // soil/water temp (GW1100 "wh34")
    WN38,    // black globe thermometer
    WS80,    // ultrasonic array (GW1100 "wh80")
    WS85,    // ultrasonic array + piezo rain (GW1100 "wh85")
    WS90,    // ultrasonic array + piezo rain + pressure (GW1100 "wh90")
    WH26,    // rainfall (UNVERIFIED stub)
    WH35,    // leaf wetness (UNVERIFIED stub)
    Builtin = 250,  // the gateway's OWN built-in indoor T/H/P (I2C HP203B+SHT), not RF
};

inline const char *family_name(Family f) {
    switch (f) {
        case Family::WH25:   return "WH25/WH32";
        case Family::WH31:   return "WH31";
        case Family::WH51:   return "WH51";
        case Family::WH24:   return "WH24";
        case Family::WH65:   return "WH65";
        case Family::WH0290: return "WH0290/WH41";
        case Family::WS69:   return "WS69";
        case Family::WH40:   return "WH40";
        case Family::WN20:   return "WN20";
        case Family::WS68:   return "WS68";
        case Family::WH43:   return "WH43";
        case Family::WH45:   return "WH45";
        case Family::WH46:   return "WH46";
        case Family::WH52:   return "WH52";
        case Family::WH55:   return "WH55";
        case Family::WH54:   return "WH54";
        case Family::WH57:   return "WH57";
        case Family::WN34:   return "WN34";
        case Family::WN38:   return "WN38";
        case Family::WS80:   return "WS80";
        case Family::WS85:   return "WS85";
        case Family::WS90:   return "WS90";
        case Family::WH26:   return "WH26";
        case Family::WH35:   return "WH35";
        case Family::Builtin: return "Indoor";
        default:             return "Unknown";
    }
}

// Families that carry a cumulative rain counter (feeding the RainAggregator).
inline bool is_rain_family(Family f) {
    switch (f) {
        case Family::WH24: case Family::WH65: case Family::WS69:
        case Family::WH40: case Family::WN20:
        case Family::WS85: case Family::WS90:
            return true;
        default:
            return false;
    }
}

// Which measurements this packet actually carried. The Fine Offset families transmit a
// fixed-width frame and mark an absent/faulty sub-sensor with an all-ones sentinel, so
// "the field is zero" and "the field is not present" are DIFFERENT things — 0.0 °C, calm
// wind and night-time UV/lux are all legitimate readings. The decoders set a bit here only
// after the value passes its sentinel check; every consumer must gate on has() rather than
// on a magic zero. (The previous 0-as-absent convention silently dropped genuine zeros and,
// worse, forwarded un-sentinel-checked garbage as real data.)
enum Field : uint32_t {
    F_TEMP  = 1u << 0,  F_HUM   = 1u << 1,  F_PRESS = 1u << 2,
    F_WIND  = 1u << 3,  F_WDIR  = 1u << 4,  F_GUST  = 1u << 5,
    F_UV    = 1u << 6,  F_LIGHT = 1u << 7,
    F_BATV  = 1u << 8,  F_CAPV  = 1u << 9,
};

struct Reading {
    Family family = Family::Unknown;
    uint32_t present = 0;     // bitmask of Field — see enum above
    void mark(uint32_t f) { present |= f; }
    bool has(uint32_t f) const { return (present & f) != 0; }

    uint8_t  model = 0;       // model nibble (WH25/WH32: 0xD old / 0xE new)
    uint32_t id = 0;
    uint8_t  channel = 0;     // channeled families (WH31/WH55; the GW1100 assigns
                              // channels for WH51/WH34/WH35/WH41 by registration —
                              // those packets carry no channel on the wire)
    bool     battery_low = false;
    float    battery_v = 0;   // battery voltage in VOLTS (families whose *batt is BATTERY_VOLTAGE)
    float    cap_v = 0;       // supercapacitor voltage in VOLTS (WS90/WS85 piezo rain)
    bool     rain_state = false;  // raw piezo wet/start flag; can stay on after rain stops
    int8_t   rssi_dbm = 0;    // set from the RX path (reg 0x70 - 128), not decoded here

    float temp_c = 0;         // WH25/WH31/WH24/WH65/WS80/WS90/WH45/WH46
    int   humidity = 0;       // 0..100
    float pressure_hpa = 0;   // WH25/WH32B/WS69/WS90
    float wind_mps = 0;       // WH24/WH65/WS68/WS80/WS85/WS90
    int   wind_dir_deg = 0;
    float gust_mps = 0;
    float rain_mm = 0;        // total counter (WH24/WH65/WH40/WN20/WS85/WS90)
    float rain_rate_mmh = 0;  // WH40 role: not on the wire in the rtl_433 reference —
                              // the gateway derives rate from total deltas; left 0 here
    // Derived rain aggregates (filled by RainAggregator, not decoded): running totals
    // since the last hour/day/week/month/year boundary plus the current rain event.
    float rain_event_mm = 0;
    float rain_hour_mm = 0;
    float rain_day_mm = 0;
    float rain_week_mm = 0;
    float rain_month_mm = 0;
    float rain_year_mm = 0;
    float daily_gust_mps = 0;   // max gust since midnight (m/s), for maxdailygust
    int   wind_dir_avg_deg = 0; // 10-min circular-avg wind direction, for winddir_avg10m
    float last24h_rain_mm = 0;  // rain over the last 24 h (rolling), for last24hrain_piezomm
    int   uv = 0;             // UV index
    float light_wm2 = 0;
    int   moisture_pct = 0;   // WH51/WH52
    float soil_temp_c = 0;    // WN34/WN38, WH52
    float soil_ec_uscm = 0;   // WH52 electrical conductivity
    int   leaf_wetness = 0;   // WH35 (UNVERIFIED stub)
    bool  leak = false;       // WH55/WH54
    int   pm25_ugm3 = 0;      // WH0290/WH41/WH43/WH45/WH46
    int   pm10_ugm3 = 0;
    int   co2_ppm = 0;        // WH45/WH46
    uint32_t strikes = 0;          // WH57 lightning strike count
    float    lightning_dist_km = 0; // WH57 storm-front distance

    bool valid = false;
};

// ---- Derived rain aggregates --------------------------------------------------
// The Fine Offset rain counters (r.rain_mm) are monotonic cumulative totals per sensor;
// the hourly/daily/weekly/monthly/yearly/event totals and ten-minute mean rate are
// derived here from counter deltas, with period boundaries detected from the LOCAL clock
// components passed in by the caller. State is per rain sensor (keyed by family+id) and
// must be persisted by the caller to survive reboot — so a 3am reboot does not zero the
// running daily total. A counter reset/wrap is treated as zero delta (not negative rain).

struct RainAggregate {
    float event_mm = 0, hour_mm = 0, day_mm = 0, week_mm = 0, month_mm = 0, year_mm = 0;
    float rate_mmh = 0;
    float last24h_mm = 0;
};

class RainAggregator {
 public:
    static constexpr uint8_t MAX_BUCKETS = 6;

    struct Bucket {
        RainWindow24h rolling;
        RainWindow10m recent;
        uint32_t key = 0;             // (family << 24) | (id & 0xFFFFFF)
        float last_counter = -1.0f;   // -1 = uninitialized
        float event_mm = 0, hour_mm = 0, day_mm = 0, week_mm = 0, month_mm = 0, year_mm = 0;
        uint32_t last_rain_ms = 0;    // millis() of last nonzero delta (event = 24h-dry reset)
        uint32_t last_sample_ms = 0;  // millis() of last sample (rate = delta / dt)
        uint16_t mark_doy = 0;        // boundary marks (local time)
        uint8_t  mark_hour = 0xFF;
        uint8_t  mark_week = 0xFF;
        uint8_t  mark_month = 0xFF;
        uint16_t mark_year = 0;

        void restore_calendar(uint16_t doy, uint8_t month, uint16_t year) {
            mark_doy = doy;
            mark_month = month;
            mark_year = year;
            // Week is derivable from the saved day; 0xFF would clear week/day on boot.
            mark_week = doy ? (uint8_t)((doy - 1) / 7) : 0xFF;
        }
    };

    Bucket *buckets() { return buckets_; }
    uint8_t count() const { return count_; }
    void set_count(uint8_t n) { count_ = (n <= MAX_BUCKETS) ? n : 0; }
    void reset() { count_ = 0; }
    // True if the last sample changed the persisted state (a boundary roll or rain) — the
    // caller uses this to persist only when needed, avoiding flash-wear on a dry day.
    bool take_dirty() { bool d = dirty_; dirty_ = false; return d; }

    Bucket *find(uint32_t key) {
        for (uint8_t i = 0; i < count_; i++)
            if (buckets_[i].key == key)
                return &buckets_[i];
        if (count_ < MAX_BUCKETS) {
            buckets_[count_] = Bucket{};
            buckets_[count_].key = key;
            return &buckets_[count_++];
        }
        return nullptr;
    }

    // Sample one calibrated rain counter. doy/hour/week/month/year are LOCAL-time components
    // (week = (day_of_year-1)/7); now_ms = millis(). Fills `out` with the running totals.
    void sample(uint8_t family, uint32_t id, float counter,
                uint16_t doy, uint8_t hour, uint8_t week, uint8_t month, uint16_t year,
                uint32_t now_ms, RainAggregate &out) {
        out = RainAggregate{};
        Bucket *b = find(((uint32_t)family << 24) | (id & 0xFFFFFF));
        if (b == nullptr)
            return;
        out.last24h_mm = b->rolling.sample(counter, now_ms);
        // Keep rain visible between RF packets and through the 30-second upload interval.
        out.rate_mmh = b->recent.sample(counter, now_ms) * 6.0f;

        // Roll running totals at local-time period boundaries, BEFORE folding in the delta.
        bool rolled = false;
        if (b->last_counter < 0.0f) b->year_mm = counter;
        // year=0 means the clock has not synchronised: preserve real calendar marks.
        if (year != 0) {
        if (b->last_counter < 0.0f || b->mark_year == 0) {
            // First ever sample: seed the yearly bucket to the counter. The rain counter IS
            // the lifetime total, and the OEM reports it as the year-to-date figure (its own
            // 11 Aug value was 91.8 mm). Monthly/weekly/daily stay 0 — their baselines need
            // the counter at the month/week/day start, which we can't know cold.
            if (b->last_counter < 0.0f)
                b->year_mm = counter;
            b->mark_year = year; b->mark_month = month; b->mark_week = week;
            b->mark_doy = doy; b->mark_hour = hour;
        } else if (year != b->mark_year) {
            b->year_mm = b->month_mm = b->week_mm = b->day_mm = b->hour_mm = 0;
            b->mark_year = year; b->mark_month = month; b->mark_week = week; b->mark_doy = doy; b->mark_hour = hour;
            rolled = true;
        } else if (month != b->mark_month) {
            b->month_mm = b->week_mm = b->day_mm = b->hour_mm = 0;
            b->mark_month = month; b->mark_week = week; b->mark_doy = doy; b->mark_hour = hour;
            rolled = true;
        } else if (week != b->mark_week) {
            b->week_mm = b->day_mm = b->hour_mm = 0;
            b->mark_week = week; b->mark_doy = doy; b->mark_hour = hour;
            rolled = true;
        } else if (doy != b->mark_doy) {
            b->day_mm = b->hour_mm = 0;
            b->mark_doy = doy; b->mark_hour = hour;
            rolled = true;
        } else if (hour != b->mark_hour) {
            b->hour_mm = 0;
            b->mark_hour = hour;
            rolled = true;
        }
        }

        // Event rain: reset after 24 h with no rain.
        if (b->event_mm > 0 && (now_ms - b->last_rain_ms) >= 86400000UL) {
            b->event_mm = 0;
            rolled = true;
        }

        // Delta vs the previous counter (reset/wrap => not negative rain).
        float delta = 0;
        if (b->last_counter >= 0.0f && counter >= b->last_counter)
            delta = counter - b->last_counter;
        b->last_counter = counter;
        if (delta > 0.0f) {
            b->last_rain_ms = now_ms;
            dirty_ = true;
        }
        if (rolled)
            dirty_ = true;

        b->last_sample_ms = now_ms;

        b->event_mm += delta;
        b->hour_mm  += delta;
        b->day_mm   += delta;
        b->week_mm  += delta;
        b->month_mm += delta;
        b->year_mm  += delta;

        out.event_mm = b->event_mm; out.hour_mm = b->hour_mm; out.day_mm = b->day_mm;
        out.week_mm = b->week_mm; out.month_mm = b->month_mm; out.year_mm = b->year_mm;
    }

 private:
    Bucket buckets_[MAX_BUCKETS];
    uint8_t count_ = 0;
    bool dirty_ = false;
};

// ---- Family decoders ----------------------------------------------------------
// Each returns true only after integrity validates. b: payload starting at the
// family/type byte; n: bytes available (may include zero padding past the packet).

// WH25 / WH32B / WN32P / WH32 indoor T/H/P — 7 bytes: MI IT TT HH PP PP CC [XX]
// Ported from rtl_433 src/devices/fineoffset.c:604-694 (fineoffset_WH25_callback).
// Type byte high nibble: 0xE = WH25/WH32B (has pressure), 0xD = old WH32 (no baro).
// rtl_433 checks the XOR bitsum only for WH25 and tells WH25/WH32B apart by RF bit
// length, which the CMT2300A FIFO does not give us; we accept 0xE* on the additive
// sum alone (WH32B has no XOR byte on the wire).
inline bool decode_wh25(const uint8_t *b, size_t n, Reading &r) {
    if (n < 7) return false;
    if (add_sum(b, 6) != b[6]) return false;                 // fineoffset.c:654

    // Modern WH32B/WN32P frames end at the additive checksum, byte 6.
    // The radio FIFO contains trailing bytes, not the sensor's wire length;
    // demanding the legacy WH25 XOR in byte 7 silently rejects these sensors.
    // Keep plausibility checks below plus the receiver RSSI/repeat guards.
    r.family = Family::WH25;
    r.model  = (b[0] >> 4) & 0xF;
    // NOTE: only 8 bits of ID — 256 possible values. Nothing can be done about that (it is
    // what the protocol carries), which is why the anti-phantom defence cannot rely on ID
    // uniqueness alone.
    r.id     = ((b[0] & 0x0F) << 4) | (b[1] >> 4);
    r.battery_low = (b[1] & 0x08) != 0;

    // 2. Plausibility bounds. A surviving noise packet is overwhelmingly likely to decode
    //    to a physically impossible value; a real indoor sensor never will.
    float t = ((((b[1] & 0x07) << 8) | b[2]) - 400) * 0.1f;  // fineoffset.c:672-673
    if (t < -40.0f || t > 60.0f)
        return false;
    r.temp_c = t; r.mark(F_TEMP);
    if (b[3] > 100)                                          // no 0xFF sentinel: any >100 is junk
        return false;
    r.humidity = b[3]; r.mark(F_HUM);
    uint16_t p_raw = ((uint16_t)b[4] << 8) | b[5];
    if ((b[0] & 0xF0) == 0xE0 && p_raw != 0xFFFF) {          // fineoffset.c:675-676,687
        float p = p_raw * 0.1f;
        if (p < 300.0f || p > 1100.0f)
            return false;
        r.pressure_hpa = p; r.mark(F_PRESS);
    }
    r.valid = true;
    return true;
}

// WH31 / WH31E (0x30) / WH31B (0x37) 8-channel outdoor T/H — 7 bytes:
// YY II CT TT HH XX AA. Ported from rtl_433 src/devices/ambientweather_wh31e.c:231-267.
inline bool decode_wh31(const uint8_t *b, size_t n, Reading &r) {
    if (n < 7) return false;
    if (crc8(0x31, b, 6) != 0) return false;   // residue over b[0..5] (CRC byte b[5])
    if (add_sum(b, 6) != b[6]) return false;

    r.family = Family::WH31;
    r.id      = b[1];
    r.channel = ((b[2] & 0x70) >> 4) + 1;      // ambientweather_wh31e.c:245
    r.battery_low = (b[2] & 0x04) != 0;
    r.temp_c  = ((((b[2] & 0x03) << 8) | b[3]) - 400) * 0.1f;
    r.mark(F_TEMP);
    if (b[4] != 0xFF && b[4] <= 100) { r.humidity = b[4]; r.mark(F_HUM); }
    r.valid = true;
    return true;
}

// WH40 rain gauge — 9 bytes: YY 0I II II FV RR RR XX AA (tips in 0.1 mm).
// Ported from rtl_433 src/devices/ambientweather_wh31e.c:308-346.
inline bool decode_wh40(const uint8_t *b, size_t n, Reading &r) {
    if (n < 9) return false;
    if (crc8(0x31, b, 8) != 0) return false;   // residue over b[0..7] (CRC byte b[7])
    if (add_sum(b, 8) != b[8]) return false;

    r.family = Family::WH40;
    r.id = (((uint32_t)b[1] & 0x0F) << 16) | ((uint32_t)b[2] << 8) | b[3];
    r.battery_low = (b[4] & 0x1F) <= 9;        // (b[4]&0x1f)*0.1 V; <=0.9 V is 0%
    r.battery_v = (b[4] & 0x1F) * 0.1f;        // battery V (0.1 V units)
    r.rain_mm = (((uint16_t)b[5] << 8) | b[6]) * 0.1f;
    r.valid = true;
    return true;
}

// WN20 rain gauge — 10 bytes: YY 00 II II FV RR RR UU XX AA (tips in 0.1 mm).
// Ported from rtl_433 src/devices/ambientweather_wh31e.c:348-390.
inline bool decode_wn20(const uint8_t *b, size_t n, Reading &r) {
    if (n < 10) return false;
    if (crc8(0x31, b, 9) != 0) return false;   // residue over b[0..8] (CRC byte b[8])
    if (add_sum(b, 9) != b[9]) return false;

    r.family = Family::WN20;
    r.id = ((uint16_t)b[2] << 8) | b[3];
    r.battery_low = b[4] <= 90;                // b[4]*0.02 V; <=1.8 V is 0%
    r.battery_v = b[4] * 0.02f;                // battery V (0.02 V units)
    r.rain_mm = (((uint16_t)b[5] << 8) | b[6]) * 0.1f;
    r.valid = true;
    return true;
}

// WS68 solar anemometer with LUX/UVI — 16 bytes. This is the unit the GW1100 names
// "wh68" (slot 1). Ported from rtl_433 src/devices/ambientweather_wh31e.c:392-434.
inline bool decode_ws68(const uint8_t *b, size_t n, Reading &r) {
    if (n < 16) return false;
    if (crc8(0x31, b, 15) != 0) return false;  // residue over b[0..14] (CRC byte b[14])
    if (add_sum(b, 15) != b[15]) return false;

    r.family = Family::WS68;
    r.id = ((uint16_t)b[2] << 8) | b[3];
    r.battery_low = b[6] <= 0x20;              // ambientweather_wh31e.c:409
    // ambientweather_wh31e.c has no DATA_COND guards for the WS68 measurements, but the
    // frame layout is the WS80's and the family uses the same all-ones "sub-sensor absent"
    // convention, so the same sentinels are applied here. UNVERIFIED against a real WS68.
    uint16_t lux_raw = ((uint16_t)b[4] << 8) | b[5];
    if (lux_raw != 0xFFFF) { r.light_wm2 = lux_raw * 10.0f / 126.0f; r.mark(F_LIGHT); }
    uint16_t wind_raw = (uint16_t)(((b[7] & 0x10) << 4) | b[10]);
    if (wind_raw != 0x1FF) { r.wind_mps = wind_raw * 0.1f; r.mark(F_WIND); }
    uint16_t dir_raw = (uint16_t)(((b[7] & 0x20) << 3) | b[11]);
    if (dir_raw != 0x1FF && dir_raw < 360) { r.wind_dir_deg = dir_raw; r.mark(F_WDIR); }
    uint16_t gust_raw = (uint16_t)(((b[7] & 0x40) << 2) | b[12]);
    if (gust_raw != 0x1FF) { r.gust_mps = gust_raw * 0.1f; r.mark(F_GUST); }
    if (b[13] != 0xFF) { r.uv = (b[13] + 5) / 10; r.mark(F_UV); }  // UVI is x10 on the wire
    r.valid = true;
    return true;
}

// WH24 / WH65(B) weather array — 17 bytes:
// FF II DD VT TT HH WW GG RR RR UU UU LL LL LL CC BB
// WS69 appends an 8-byte pressure extension (CRC over b[0..23], SUM at b[24]).
// Ported from rtl_433 src/devices/fineoffset.c:309-449 (fineoffset_WH24_callback).
// rtl_433 tells WH24/WH65/WS69 apart by RF bit length; we dispatch on the type byte
// (0x24/0x65) and promote to WS69 when the pressure extension validates.
inline bool decode_wh24(const uint8_t *b, size_t n, Reading &r) {
    if (n < 17) return false;
    if (crc8(0x31, b, 16) != 0) return false;  // residue over b[0..15] (CRC byte b[15])
    if (add_sum(b, 16) != b[16]) return false;

    r.family = (b[0] == 0x65) ? Family::WH65 : Family::WH24;
    r.id     = b[1];
    r.battery_low = (b[3] & 0x08) != 0;
    r.rain_mm  = (((uint16_t)b[8] << 8) | b[9]) * ((r.family == Family::WH65) ? 0.254f : 0.3f);

    // Sentinel-gated, per the DATA_COND conditions in fineoffset.c:433-442. Note the
    // widths differ from the WS80/WS90 family: temp is 11 bits (0x7FF) and gust is 8 (0xFF).
    uint16_t dir_raw = (uint16_t)(b[2] | ((b[3] & 0x80) << 1));   // 9 bits — fineoffset.c:379
    if (dir_raw != 0x1FF && dir_raw < 360) { r.wind_dir_deg = dir_raw; r.mark(F_WDIR); }
    uint16_t temp_raw = (uint16_t)(((b[3] & 0x07) << 8) | b[4]);
    if (temp_raw != 0x7FF) { r.temp_c = (temp_raw - 400) * 0.1f; r.mark(F_TEMP); }
    if (b[5] != 0xFF && b[5] <= 100) { r.humidity = b[5]; r.mark(F_HUM); }
    float factor = (r.family == Family::WH65) ? 0.51f : 1.12f;  // m/s per count
    uint16_t wind_raw = (uint16_t)(b[6] | ((b[3] & 0x10) << 4));                 // 9 bits
    if (wind_raw != 0x1FF) { r.wind_mps = wind_raw * 0.125f * factor; r.mark(F_WIND); }
    if (b[7] != 0xFF) { r.gust_mps = b[7] * factor; r.mark(F_GUST); }  // fineoffset.c:397-399
    uint16_t uv_raw = ((uint16_t)b[10] << 8) | b[11];   // 0..20000 — fineoffset.c:402
    if (uv_raw != 0xFFFF) {
        // UVI lookup table, fineoffset.c:422-424
        static const uint16_t uvi_upper[13] =
            {432, 851, 1210, 1570, 2017, 2450, 2761, 3100, 3512, 3918, 4277, 4650, 5029};
        int uvi = 0;
        while (uvi < 13 && uvi_upper[uvi] < uv_raw) ++uvi;
        r.uv = uvi;
        r.mark(F_UV);
    }
    uint32_t light_raw = ((uint32_t)b[12] << 16) | ((uint32_t)b[13] << 8) | b[14];
    if (light_raw != 0xFFFFFF) {
        r.light_wm2 = (light_raw * 0.1f) / 126.0f;     // lux = raw/10; lux/126 = W/m2
        r.mark(F_LIGHT);
    }

    // WS69 pressure extension — fineoffset.c:364-375
    if (n >= 25 && crc8(0x31, b, 24) == 0 && add_sum(b, 24) == b[24]) {
        uint32_t p_raw = ((uint32_t)b[17] << 16) | ((uint32_t)b[18] << 8) | b[19];
        r.family = Family::WS69;
        if (p_raw < 0x01FFFF) {                      // 0x1FFFF = no pressure module
            float p = p_raw * 0.01f;
            if (p >= 300.0f && p <= 1100.0f) { r.pressure_hpa = p; r.mark(F_PRESS); }
        }
    }
    r.valid = true;
    return true;
}

// WH0290 / Ecowitt WH41 PM2.5 — 8 bytes: FF DD ?P PP ?A AA CC BB (family 0x41/0x42).
// Ported from rtl_433 src/devices/fineoffset.c:524-573 (fineoffset_WH0290_callback).
// The GW1100's 4 WH41 channels are assigned by registration; no channel on the wire.
inline bool decode_wh0290(const uint8_t *b, size_t n, Reading &r) {
    if (n < 8) return false;
    if (crc8(0x31, b, 6) != b[6]) return false;
    if (add_sum(b, 7) != b[7]) return false;

    r.family = Family::WH0290;
    r.id = b[1];
    int bars = ((b[2] & 0x40) >> 4) | ((b[4] & 0xC0) >> 6);  // fineoffset.c:555
    r.battery_low = bars <= 1;
    r.pm25_ugm3 = ((((b[2] & 0x3F) << 8) | b[3]) + 5) / 10;
    r.pm10_ugm3 = ((((b[4] & 0x3F) << 8) | b[5]) + 5) / 10;
    r.valid = true;
    return true;
}

// WH43 air quality — 10 bytes: FF II II II ?P PP ?A AA CC BB (24-bit ID).
// Ported from rtl_433 src/devices/fineoffset_wh43.c:85-135.
inline bool decode_wh43(const uint8_t *b, size_t n, Reading &r) {
    if (n < 10) return false;
    if (crc8(0x31, b, 8) != b[8]) return false;
    if (add_sum(b, 9) != b[9]) return false;

    r.family = Family::WH43;
    r.id = ((uint32_t)b[1] << 16) | ((uint32_t)b[2] << 8) | b[3];
    int bars = ((b[4] & 0x40) >> 4) | ((b[6] & 0xC0) >> 6);  // 6 = external power
    r.battery_low = bars <= 1;
    r.pm25_ugm3 = ((((b[4] & 0x3F) << 8) | b[5]) + 5) / 10;
    r.pm10_ugm3 = ((((b[6] & 0x3F) << 8) | b[7]) + 5) / 10;
    r.valid = true;
    return true;
}

// WH45 CO2/PM (also WH0295) — 15 bytes:
// YY II II II 0T TT HH Bp pp BP PP CC CC XX AA
// Ported from rtl_433 src/devices/fineoffset_wh45.c:64-129.
inline bool decode_wh45(const uint8_t *b, size_t n, Reading &r) {
    if (n < 15) return false;
    if (crc8(0x31, b, 13) != b[13]) return false;
    if (add_sum(b, 14) != b[14]) return false;

    r.family = Family::WH45;
    r.id = ((uint32_t)b[1] << 16) | ((uint32_t)b[2] << 8) | b[3];
    r.temp_c = ((((b[4] & 0x07) << 8) | b[5]) - 400) * 0.1f;
    r.mark(F_TEMP);
    if (b[6] != 0xFF && b[6] <= 100) { r.humidity = b[6]; r.mark(F_HUM); }
    int bars = ((b[7] & 0x40) >> 4) | ((b[9] & 0xC0) >> 6);  // 6 = external power
    r.battery_low = bars <= 1;
    r.pm25_ugm3 = ((((b[7] & 0x3F) << 8) | b[8]) + 5) / 10;
    r.pm10_ugm3 = ((((b[9] & 0x3F) << 8) | b[10]) + 5) / 10;
    r.co2_ppm = ((uint16_t)b[11] << 8) | b[12];
    r.valid = true;
    return true;
}

// WH46 air quality — 21 bytes (WH45 layout + PM1/PM4 + 2 unknown bytes).
// Ported from rtl_433 src/devices/fineoffset_wh46.c:60-132. PM1/PM4 have no Reading
// fields and are dropped.
inline bool decode_wh46(const uint8_t *b, size_t n, Reading &r) {
    if (n < 21) return false;
    if (crc8(0x31, b, 19) != b[19]) return false;
    if (add_sum(b, 20) != b[20]) return false;

    r.family = Family::WH46;
    r.id = ((uint32_t)b[1] << 16) | ((uint32_t)b[2] << 8) | b[3];
    r.temp_c = ((((b[4] & 0x07) << 8) | b[5]) - 400) * 0.1f;
    r.mark(F_TEMP);
    if (b[6] != 0xFF && b[6] <= 100) { r.humidity = b[6]; r.mark(F_HUM); }
    int bars = ((b[7] & 0x40) >> 4) | ((b[9] & 0xC0) >> 6);  // 6 = external power
    r.battery_low = bars <= 1;
    r.pm25_ugm3 = ((((b[7] & 0x3F) << 8) | b[8]) + 5) / 10;
    r.pm10_ugm3 = ((((b[9] & 0x3F) << 8) | b[10]) + 5) / 10;
    r.co2_ppm = ((uint16_t)b[11] << 8) | b[12];
    r.valid = true;
    return true;
}

// WH51 / WN31 soil moisture — 14 bytes:
// 0x51 ID[3] batt/boost 0x7F moist AD 0xFF[3] CRC SUM
// Ported from rtl_433 src/devices/fineoffset.c:736-839 (fineoffset_WH51_callback).
// The GW1100's 16 channels are assigned by registration; no channel on the wire.
// The brief's 0x52/0x53 type bytes are UNVERIFIED: rtl_433 knows only 0x51 (0x52 is
// the WH31E RCC clock packet, ambientweather_wh31e.c:269-306).
inline bool decode_wh51(const uint8_t *b, size_t n, Reading &r) {
    if (n < 14) return false;
    if (crc8(0x31, b, 12) != b[12]) return false;
    if (add_sum(b, 13) != b[13]) return false;

    r.family = Family::WH51;
    r.id = ((uint32_t)b[1] << 16) | ((uint32_t)b[2] << 8) | b[3];
    // b[4] top 3 bits = tx-period boost; low 5 bits = battery * 100 mV
    r.battery_low = (b[4] & 0x1F) <= 12;         // <=1200 mV — fineoffset.c:778-822
    if (b[6] > 100) return false;                // percent field; >100 is a bad frame
    r.moisture_pct = b[6];
    r.valid = true;
    return true;
}

// WH52 soil moisture/temp/EC — 24 bytes, family 0xA2.
// Ported from rtl_433 src/devices/fineoffset_wh52.c:61-121.
inline bool decode_wh52(const uint8_t *b, size_t n, Reading &r) {
    if (n < 24) return false;
    if (add_sum(b, 23) != b[23]) return false;
    if (crc8(0x31, b, 22) != b[22]) return false;

    r.family = Family::WH52;
    r.id = ((uint32_t)b[1] << 16) | ((uint32_t)b[2] << 8) | b[3];
    r.soil_temp_c = ((((b[4] & 0x1F) << 8) | b[5]) * 0.1f) - 40.0f;
    if (b[6] > 100) return false;                // percent field; >100 is a bad frame
    r.moisture_pct = b[6];
    uint32_t ec_raw = (((uint32_t)b[8] & 0x0F) << 16) | ((uint32_t)b[9] << 8) | b[10];
    r.soil_ec_uscm = ec_raw / 25.6f;             // empirical — fineoffset_wh52.c:103
    r.battery_low = (b[15] * 0.02f - 0.06f) < 1.2f;  // empirical — fineoffset_wh52.c:104
    r.valid = true;
    return true;
}

// WH55 water leak — 9+ bytes: MM FI II II BB VV VV AD XX (CRC only, no SUM in the
// reference). Ported from rtl_433 src/devices/fineoffset_wh55.c:46-97.
inline bool decode_wh55(const uint8_t *b, size_t n, Reading &r) {
    if (n < 9) return false;
    if (crc8(0x31, b, 9) != 0) return false;     // residue over b[0..8] (CRC byte b[8])

    r.family = Family::WH55;
    r.channel = (b[1] >> 4) + 1;                 // fineoffset_wh55.c:72
    r.id = ((uint16_t)b[2] << 8) | b[3];
    r.battery_low = b[4] <= 1;                   // 1..5 bars of 20%
    r.leak = (b[7] & 0x40) != 0;                 // alarm bit — fineoffset_wh55.c:80
    // 16-bit raw wetness measurement (b[5]<<8 | b[6]) not mapped to a Reading field
    r.valid = true;
    return true;
}

// WH57 lightning (rtl_433 "WH31L", AS3935) — 9 bytes:
// YY SI II II FF KK CC XX AA. Ported from rtl_433 src/devices/fineoffset_wh31l.c:88-155.
inline bool decode_wh57(const uint8_t *b, size_t n, Reading &r) {
    if (n < 9) return false;
    if (crc8(0x31, b, 8) != 0) return false;     // residue over b[0..7] (CRC byte b[7])
    if (add_sum(b, 8) != b[8]) return false;

    r.family = Family::WH57;
    // b[1] >> 4 = state (0 reset / 1 interference / 4 noise / 8 strike); not mapped
    r.id = (((uint32_t)b[1] & 0x0F) << 16) | ((uint32_t)b[2] << 8) | b[3];
    r.battery_low = ((b[4] & 0x06) >> 1) == 0;   // 2..0 bars — fineoffset_wh31l.c:124
    uint8_t dist = b[5] & 0x3F;
    if (dist != 63)                              // 63 = invalid/no strike
        r.lightning_dist_km = dist;
    r.strikes = b[6];
    r.valid = true;
    return true;
}

// WN34 soil/water temp (GW1100 "wh34") / WN38 black globe — 9 bytes:
// YY II II II ST TT BB XX AA, type 0x34 / 0x38.
// Ported from rtl_433 src/devices/fineoffset_wn34.c:42-126.
// The GW1100's 8 WH34 channels are assigned by registration; no channel on the wire.
inline bool decode_wn34(const uint8_t *b, size_t n, Reading &r) {
    if (n < 9) return false;
    if (crc8(0x31, b, 7) != b[7]) return false;
    if (add_sum(b, 8) != b[8]) return false;

    r.family = (b[0] == 0x38) ? Family::WN38 : Family::WN34;
    r.id = ((uint32_t)b[1] << 16) | ((uint32_t)b[2] << 8) | b[3];
    int16_t temp_raw = (int16_t)(((uint16_t)(b[4] & 0x0F) << 12) | ((uint16_t)b[5] << 4));
    uint8_t sub = b[4] >> 4;                     // 4 = WN34D: no -40 offset
    r.soil_temp_c = ((temp_raw >> 4) * 0.1f) - ((sub == 4) ? 0.0f : 40.0f);
    r.battery_low = ((b[6] & 0x7F) * 20) <= 1200;  // mV — fineoffset_wn34.c:87-107
    r.valid = true;
    return true;
}

// WS80 ultrasonic array (GW1100 "wh80") — 18 bytes:
// YY II II II LL LL BB FF TT HH WW DD GG VV UU UU AA XX
// Ported from rtl_433 src/devices/fineoffset_ws80.c:46-114.
inline bool decode_ws80(const uint8_t *b, size_t n, Reading &r) {
    if (n < 18) return false;
    if (crc8(0x31, b, 17) != 0) return false;    // residue over b[0..16] (CRC byte b[16])
    if (add_sum(b, 17) != b[17]) return false;

    r.family = Family::WS80;
    r.id = ((uint32_t)b[1] << 16) | ((uint32_t)b[2] << 8) | b[3];
    r.battery_low = (b[6] * 20) < 1400;          // mV — fineoffset_ws80.c:81-82
    r.battery_v = b[6] * 0.02f; r.mark(F_BATV);  // battery V (b[6] * 20 mV)

    // Sentinel-gated, per the DATA_COND conditions in fineoffset_ws80.c:99-105.
    uint16_t light_raw = ((uint16_t)b[4] << 8) | b[5];
    if (light_raw != 0xFFFF) { r.light_wm2 = light_raw * 0.078925f; r.mark(F_LIGHT); }
    uint16_t temp_raw = (uint16_t)(((b[7] & 0x03) << 8) | b[8]);
    if (temp_raw != 0x3FF) { r.temp_c = (temp_raw - 400) * 0.1f; r.mark(F_TEMP); }
    if (b[9] != 0xFF && b[9] <= 100) { r.humidity = b[9]; r.mark(F_HUM); }
    uint16_t wind_raw = (uint16_t)(((b[7] & 0x10) << 4) | b[10]);
    if (wind_raw != 0x1FF) { r.wind_mps = wind_raw * 0.1f; r.mark(F_WIND); }
    uint16_t dir_raw = (uint16_t)(((b[7] & 0x20) << 3) | b[11]);
    if (dir_raw != 0x1FF && dir_raw < 360) { r.wind_dir_deg = dir_raw; r.mark(F_WDIR); }
    uint16_t gust_raw = (uint16_t)(((b[7] & 0x40) << 2) | b[12]);
    if (gust_raw != 0x1FF) { r.gust_mps = gust_raw * 0.1f; r.mark(F_GUST); }
    if (b[13] != 0xFF) { r.uv = (b[13] + 5) / 10; r.mark(F_UV); }
    r.valid = true;
    return true;
}

// WS85 ultrasonic array + piezo rain (GW1100 "wh85") — 28 bytes; a WS90 without
// temp/humidity/lux/uv. Ported from rtl_433 src/devices/fineoffset_ws85.c:62-144.
inline bool decode_ws85(const uint8_t *b, size_t n, Reading &r) {
    if (n < 28) return false;
    if (crc8(0x31, b, 26) != b[26]) return false;
    if (add_sum(b, 27) != b[27]) return false;

    r.family = Family::WS85;
    r.id = ((uint32_t)b[1] << 16) | ((uint32_t)b[2] << 8) | b[3];
    r.battery_low = (b[4] * 20) < 1400;          // mV — fineoffset_ws85.c:99-117
    r.battery_v = b[4] * 0.02f; r.mark(F_BATV);  // battery V (b[4] * 20 mV)
    r.rain_state = (b[12] & 0x10) != 0;          // rain-start flag
    r.rain_mm = (((uint16_t)b[15] << 8) | b[16]) * 0.1f;

    // Sentinel-gated, per the DATA_COND conditions in fineoffset_ws85.c:129-135.
    if (b[17] != 0xFF) { r.cap_v = (b[17] & 0x3f) * 0.1f; r.mark(F_CAPV); }
    uint16_t wind_raw = (uint16_t)(((b[5] & 0x10) << 4) | b[7]);
    if (wind_raw != 0x1FF) { r.wind_mps = wind_raw * 0.1f; r.mark(F_WIND); }
    uint16_t dir_raw = (uint16_t)(((b[5] & 0x20) << 3) | b[8]);
    if (dir_raw != 0x1FF && dir_raw < 360) { r.wind_dir_deg = dir_raw; r.mark(F_WDIR); }
    uint16_t gust_raw = (uint16_t)(((b[5] & 0x40) << 2) | b[9]);
    if (gust_raw != 0x1FF) { r.gust_mps = gust_raw * 0.1f; r.mark(F_GUST); }
    r.valid = true;
    return true;
}

// WS90 ultrasonic array + piezo rain + pressure (GW1100 "wh90") — 32 bytes.
// Ported from rtl_433 src/devices/fineoffset_ws90.c:67-153.
inline bool decode_ws90(const uint8_t *b, size_t n, Reading &r) {
    if (n < 32) return false;
    if (crc8(0x31, b, 31) != 0) return false;    // residue over b[0..30] (CRC byte b[30])
    if (add_sum(b, 31) != b[31]) return false;

    r.family = Family::WS90;
    r.id = ((uint32_t)b[1] << 16) | ((uint32_t)b[2] << 8) | b[3];
    r.battery_low = (b[6] * 20) < 1400;          // mV — fineoffset_ws90.c:105-106
    r.battery_v = b[6] * 0.02f; r.mark(F_BATV);  // battery V (b[6] * 20 mV)
    r.rain_state = (b[16] & 0x10) != 0;          // rain-start flag
    r.rain_mm = (((uint16_t)b[19] << 8) | b[20]) * 0.1f;

    // Every field below is gated on its all-ones sentinel, exactly as the DATA_COND
    // conditions in fineoffset_ws90.c:133-144. Dropping these was forwarding the WS90's
    // "sub-sensor invalid" markers to Home Assistant as real weather.
    uint16_t light_raw = ((uint16_t)b[4] << 8) | b[5];
    if (light_raw != 0xFFFF) { r.light_wm2 = light_raw * 0.078925f; r.mark(F_LIGHT); }
    if (b[21] != 0xFF) { r.cap_v = (b[21] & 0x3f) * 0.1f; r.mark(F_CAPV); }
    uint16_t temp_raw = (uint16_t)(((b[7] & 0x03) << 8) | b[8]);
    if (temp_raw != 0x3FF) { r.temp_c = (temp_raw - 400) * 0.1f; r.mark(F_TEMP); }
    if (b[9] != 0xFF && b[9] <= 100) { r.humidity = b[9]; r.mark(F_HUM); }
    uint16_t wind_raw = (uint16_t)(((b[7] & 0x10) << 4) | b[10]);
    if (wind_raw != 0x1FF) { r.wind_mps = wind_raw * 0.1f; r.mark(F_WIND); }
    uint16_t dir_raw = (uint16_t)(((b[7] & 0x20) << 3) | b[11]);
    if (dir_raw != 0x1FF && dir_raw < 360) { r.wind_dir_deg = dir_raw; r.mark(F_WDIR); }
    uint16_t gust_raw = (uint16_t)(((b[7] & 0x40) << 2) | b[12]);
    if (gust_raw != 0x1FF) { r.gust_mps = gust_raw * 0.1f; r.mark(F_GUST); }
    if (b[13] != 0xFF) { r.uv = (b[13] + 5) / 10; r.mark(F_UV); }

    // Pressure is a 14-BIT field: the sentinel is 0x3FFF and the top 2 bits of b[14] are
    // flags, so the full 16-bit word must be masked before the comparison. rtl_433
    // (fineoffset_ws90.c:115,135) compares the unmasked word and therefore leaks the
    // invalid marker whenever a flag bit is set — observed live on this station as
    // 0xBFFF -> 49151 hPa (recorder, 2026-08-15 21:07:36 and 2026-08-16 12:12:21).
    // The range check is belt-and-braces: the field is unscaled hPa.
    uint16_t p_raw = (uint16_t)(((b[14] & 0x3F) << 8) | b[15]);
    if (p_raw != 0x3FFF) {
        // The SCALING is genuinely ambiguous upstream: rtl_433 prints this field as whole
        // hPa, but 14 bits also comfortably fits 0.1 hPa units (1013.2 -> 10132), and the
        // field was labelled "aka pressure?" in earlier revisions. This unit's barometer
        // only ever reports the invalid sentinel, so there is no local sample to settle it
        // — accept either rather than guess, and range-check the result.
        float p = (p_raw >= 3000) ? p_raw * 0.1f : (float)p_raw;
        if (p >= 300.0f && p <= 1100.0f) {
            r.pressure_hpa = p;                  // ambient (station) pressure, hPa
            r.mark(F_PRESS);
        }
    }
    r.valid = true;
    return true;
}

// ---- UNVERIFIED stubs ----------------------------------------------------------
// These sensor families use known packet type codes but have
// NO rtl_433 reference decoder, so no layout is invented here.

// UNVERIFIED — no rtl_433 reference (WH26 rainfall, type 0x26).
inline bool decode_wh26(const uint8_t *, size_t, Reading &) { return false; }

// UNVERIFIED — no rtl_433 reference (WH35 leaf wetness, type 0x35).
inline bool decode_wh35(const uint8_t *, size_t, Reading &) { return false; }

// UNVERIFIED — no rtl_433 reference (WH54 leak, type 0x54).
inline bool decode_wh54(const uint8_t *, size_t, Reading &) { return false; }

// ---- Top-level dispatch -------------------------------------------------------
// b: payload (starting at the family/type byte). n: bytes available. Returns true if
// decoded and integrity-validated. WH25/WH32 carry the sensor ID in the type byte's
// low nibble, so those dispatch on the high nibble.
// WH2/WH5/WH1050/WH1080 are OOK legacy families — not decodable from the CMT2300A
// FSK packet path and intentionally omitted (fineoffset_wh1050.c/wh1080.c skipped
// per the brief).
inline bool decode(const uint8_t *b, size_t n, Reading &r) {
    if (n == 0) return false;
    if ((b[0] & 0xF0) == 0xD0 || (b[0] & 0xF0) == 0xE0)
        return decode_wh25(b, n, r);             // WH25 / WH32B / WH32
    switch (b[0]) {
        case 0x24: case 0x65: return decode_wh24(b, n, r);   // WH24 / WH65 / WS69
        case 0x30: case 0x37: return decode_wh31(b, n, r);   // WH31E / WH31B
        case 0x40:            return decode_wh40(b, n, r);   // WH40 rain
        case 0x20:            return decode_wn20(b, n, r);   // WN20 rain
        case 0x68:            return decode_ws68(b, n, r);   // WS68 ("wh68")
        case 0x41: case 0x42: return decode_wh0290(b, n, r); // WH0290 / WH41
        case 0x43:            return decode_wh43(b, n, r);   // WH43
        case 0x45:            return decode_wh45(b, n, r);   // WH45 CO2/PM
        case 0x46:            return decode_wh46(b, n, r);   // WH46
        case 0x51:            return decode_wh51(b, n, r);   // WH51 soil
        case 0xA2:            return decode_wh52(b, n, r);   // WH52 soil/EC
        case 0x55:            return decode_wh55(b, n, r);   // WH55 leak
        case 0x57:            return decode_wh57(b, n, r);   // WH57 lightning
        case 0x34: case 0x38: return decode_wn34(b, n, r);   // WN34 / WN38
        case 0x80:            return decode_ws80(b, n, r);   // WS80 ("wh80")
        case 0x85:            return decode_ws85(b, n, r);   // WS85 ("wh85")
        case 0x90:            return decode_ws90(b, n, r);   // WS90 ("wh90")
        case 0x26:            return decode_wh26(b, n, r);   // UNVERIFIED stub
        case 0x35:            return decode_wh35(b, n, r);   // UNVERIFIED stub
        case 0x54:            return decode_wh54(b, n, r);   // UNVERIFIED stub
        default:
            r.family = Family::Unknown;
            return false;
    }
}

}  // namespace ecowitt_rf
}  // namespace esphome
