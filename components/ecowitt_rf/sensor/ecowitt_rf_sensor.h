// SPDX-License-Identifier: GPL-3.0-only
// ESPHome custom sensor platform: GW1100 RF receiver + device-config control plane.
//
// Ties together cmt2300a.h (radio init) + fineoffset.h (decode) + ecowitt_upload.h
// (custom-server HTTP upload). Polls the CMT2300A RX FIFO, decodes a Fine Offset
// packet, applies calibration, gates by family-enable, publishes the values as
// ESPHome sensors, and uploads to the custom server.
//
// CONFIG GUI: runtime-editable settings are exposed in the YAML as `template`
// text/number/select/switch entities whose `on_value`/`on_turn_on`/`on_turn_off`
// automations call the setters below. Each setter persists to ESPHome preferences
// (FLASH-backed — see `esp8266: restore_from_flash: true` in the YAML) and applies
// immediately, so config survives power loss and is re-applied on boot. Two
// output-only text_sensors (sensor roster + device info) are owned here directly.
#pragma once

#include "esphome/core/component.h"
#include "esphome/core/preferences.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/switch/switch.h"
#include "esphome/components/text_sensor/text_sensor.h"
#include "esphome/components/time/real_time_clock.h"
#include "esphome/components/json/json_util.h"
#include "esphome/components/web_server_base/web_server_base.h"
#ifdef USE_MQTT
#include "esphome/components/mqtt/mqtt_client.h"
#endif

#include <cstdint>
#include <string>
#include <cstring>
#include <cstdio>
#include <algorithm>
#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <ESPAsyncWebServer.h>
#include <ctime>
#include <cstdlib>
#include <new>
#include "bounded_json.h"
#include "owned_response.h"
#include "rain_recovery.h"

#include "cmt2300a.h"
#include "fineoffset.h"
#include "remote_indoor.h"
#include "ecowitt_upload.h"
#include "indoor_sensor.h"
#include "dashboard_html.h"

namespace esphome {
namespace ecowitt_rf {

// Preference type tags (unique within this component). The ESP8266 flash preference sector
// is only 128 words (512 B); config is one blob and the server list a second, so they share
// it. Per-sensor overrides are RAM-only (no room) and re-learned on boot.
enum : uint32_t {
    PREF_RAIN_RTC = 0xE615, // separate RTC allocator; flash layout stays unchanged
    PREF_CFG = 0xE601,       // consolidated DeviceConfig blob
    PREF_RAIN = 0xE612,      // compact rain-aggregate state (first bucket)
    PREF_DISABLED = 0xE614,  // per-sensor disable list
    PREF_SERVERS = 0xE613,   // upload-server list blob
};

// Runtime device config. Defaults match the YAML (metric, 868 MHz, HA webhook path
// is supplied by YAML as `upload.path`; empty here means "not yet overridden").
struct DeviceConfig {
    char host[24] = {0};          // IP/hostname of the primary Ecowitt server (fits IPv4)
    char path[48] = {0};          // webhook path (HA /api/webhook/<32-hex> = 45 chars)
    char passkey[32] = {0};
    uint16_t port = 8123;
    uint16_t interval_s = 16;
    char timezone[24] = {0};      // empty -> keep the compile-time YAML timezone (IANA <=19)
    uint8_t band = 1;             // device_type: 0=433,1=868,2=915,3=920
    bool upload_enabled = true;
    bool cloud_enabled = false;   // Ecowitt cloud (OEM app) upload — OFF by default
    uint16_t family_enable = 0xFFFF;  // one bit per family group, all on by default

    float cal_temp = 0.0f;        // additive offsets / multiplicative gains
    float cal_hum = 0.0f;
    float cal_press = 0.0f;
    float cal_rain = 1.0f;
    float cal_wind = 1.0f;
    float cal_solar = 1.0f;
    float cal_uv = 1.0f;
};

// Family-enable bit indices (grouped families share one switch).
enum FamilyGroup : uint8_t {
    GRP_INDOOR = 0,       // WH25/WH32
    GRP_OUTDOOR_TH = 1,   // WH31
    GRP_SOIL_MOIST = 2,   // WH51
    GRP_SOIL_TEMP = 3,    // WN34/WN38
    GRP_SOIL_EC = 4,      // WH52
    GRP_ARRAY = 5,        // WH24/WH65/WS69
    GRP_RAIN = 6,         // WH40/WN20
    GRP_ULTRASONIC = 7,   // WS68/WS80/WS85/WS90
    GRP_AIRQ = 8,         // WH0290/WH43/WH45/WH46
    GRP_LIGHTNING = 9,    // WH57
    GRP_LEAK = 10,        // WH55/WH54
    GRP_LEAF = 11,        // WH35
};

// ESPHome's runtime set_timezone() accepts POSIX TZ strings, not IANA names. The YAML
// timezone select offers IANA names (human-readable); map them to POSIX here.
inline const char *iana_to_posix(const char *tz) {
    if (tz == nullptr || *tz == '\0') return nullptr;
    if (strcmp(tz, "Europe/London") == 0) return "GMT0BST,M3.5.0/1,M10.5.0";
    if (strcmp(tz, "Europe/Berlin") == 0 || strcmp(tz, "Europe/Paris") == 0 ||
        strcmp(tz, "Europe/Amsterdam") == 0 || strcmp(tz, "Europe/Madrid") == 0 ||
        strcmp(tz, "Europe/Rome") == 0) return "CET-1CEST,M3.5.0,M10.5.0/3";
    if (strcmp(tz, "UTC") == 0) return "UTC0";
    if (strcmp(tz, "America/New_York") == 0) return "EST5EDT,M3.2.0,M11.1.0";
    if (strcmp(tz, "America/Los_Angeles") == 0) return "PST8PDT,M3.2.0,M11.1.0";
    if (strcmp(tz, "Australia/Sydney") == 0) return "AEST-10AEDT,M10.1.0,M4.1.0/3";
    if (strcmp(tz, "Asia/Tokyo") == 0) return "JST-9";
    return "UTC0";
}

inline uint8_t family_group(Family f) {
    switch (f) {
        case Family::WH25: return GRP_INDOOR;
        case Family::WH31: return GRP_OUTDOOR_TH;
        case Family::WH51: return GRP_SOIL_MOIST;
        case Family::WN34:
        case Family::WN38: return GRP_SOIL_TEMP;
        case Family::WH52: return GRP_SOIL_EC;
        case Family::WH24:
        case Family::WH65:
        case Family::WS69: return GRP_ARRAY;
        case Family::WH40:
        case Family::WN20: return GRP_RAIN;
        case Family::WS68:
        case Family::WS80:
        case Family::WS85:
        case Family::WS90: return GRP_ULTRASONIC;
        case Family::WH0290:
        case Family::WH43:
        case Family::WH45:
        case Family::WH46: return GRP_AIRQ;
        case Family::WH57: return GRP_LIGHTNING;
        case Family::WH55:
        case Family::WH54: return GRP_LEAK;
        case Family::WH35: return GRP_LEAF;
        default: return GRP_ARRAY;  // unknown -> gated under array (never matches a decode)
    }
}

// Short canonical family tag (used as the dashboard roster `fam` + fam_en keys).
inline const char *family_tag(Family f) {
    switch (f) {
        case Family::WH25:   return "WH25";
        case Family::WH31:   return "WH31";
        case Family::WH51:   return "WH51";
        case Family::WH24:   return "WH24";
        case Family::WH65:   return "WH65";
        case Family::WH0290: return "WH41";
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
        default:             return "??";
    }
}

// Map a family tag back to its enable group (for the /config fam_en POST). -1 = unknown.
inline int8_t tag_to_group(const char *t) {
    if (t == nullptr) return -1;
    if (strcmp(t, "WH25") == 0) return GRP_INDOOR;
    if (strcmp(t, "WH31") == 0) return GRP_OUTDOOR_TH;
    if (strcmp(t, "WH51") == 0) return GRP_SOIL_MOIST;
    if (strcmp(t, "WN34") == 0 || strcmp(t, "WN38") == 0) return GRP_SOIL_TEMP;
    if (strcmp(t, "WH52") == 0) return GRP_SOIL_EC;
    if (strcmp(t, "WH24") == 0 || strcmp(t, "WH65") == 0 || strcmp(t, "WS69") == 0) return GRP_ARRAY;
    if (strcmp(t, "WH40") == 0 || strcmp(t, "WN20") == 0) return GRP_RAIN;
    if (strcmp(t, "WS68") == 0 || strcmp(t, "WS80") == 0 || strcmp(t, "WS85") == 0 || strcmp(t, "WS90") == 0) return GRP_ULTRASONIC;
    if (strcmp(t, "WH41") == 0 || strcmp(t, "WH43") == 0 || strcmp(t, "WH45") == 0 || strcmp(t, "WH46") == 0) return GRP_AIRQ;
    if (strcmp(t, "WH57") == 0) return GRP_LIGHTNING;
    if (strcmp(t, "WH55") == 0 || strcmp(t, "WH54") == 0) return GRP_LEAK;
    if (strcmp(t, "WH35") == 0) return GRP_LEAF;
    return -1;
}

// Map a family tag back to the Family enum (reverse of family_tag). Unknown = Unknown.
inline Family tag_to_family(const char *t) {
    if (t == nullptr) return Family::Unknown;
    if (strcmp(t, "WH25") == 0) return Family::WH25;
    if (strcmp(t, "WH31") == 0) return Family::WH31;
    if (strcmp(t, "WH51") == 0) return Family::WH51;
    if (strcmp(t, "WH24") == 0) return Family::WH24;
    if (strcmp(t, "WH65") == 0) return Family::WH65;
    if (strcmp(t, "WH41") == 0) return Family::WH0290;
    if (strcmp(t, "WS69") == 0) return Family::WS69;
    if (strcmp(t, "WH40") == 0) return Family::WH40;
    if (strcmp(t, "WN20") == 0) return Family::WN20;
    if (strcmp(t, "WS68") == 0) return Family::WS68;
    if (strcmp(t, "WH43") == 0) return Family::WH43;
    if (strcmp(t, "WH45") == 0) return Family::WH45;
    if (strcmp(t, "WH46") == 0) return Family::WH46;
    if (strcmp(t, "WH52") == 0) return Family::WH52;
    if (strcmp(t, "WH55") == 0) return Family::WH55;
    if (strcmp(t, "WH54") == 0) return Family::WH54;
    if (strcmp(t, "WH57") == 0) return Family::WH57;
    if (strcmp(t, "WN34") == 0) return Family::WN34;
    if (strcmp(t, "WN38") == 0) return Family::WN38;
    if (strcmp(t, "WS80") == 0) return Family::WS80;
    if (strcmp(t, "WS85") == 0) return Family::WS85;
    if (strcmp(t, "WS90") == 0) return Family::WS90;
    if (strcmp(t, "WH26") == 0) return Family::WH26;
    if (strcmp(t, "WH35") == 0) return Family::WH35;
    return Family::Unknown;
}

// Latest UI values (for the /state JSON) — one slot per dashboard field, keyed by a
// "seen" bitmask so a field not yet received is omitted (null → hidden card) rather
// than sent as 0 (measured zero → grayed card).
struct UiState {
    enum : uint8_t {
        T_IN = 0, H_IN, P, T_OUT, H_OUT, LEAF, WDIR, WSPD, WGUST, RAIN, RAIN_RATE,
        UV, SOLAR, SOIL_MOIST, SOIL_T, SOIL_EC, PM25, PM10, CO2, LIGHT_N, LIGHT_D, LEAK,
        BAT_V, CAP_V, MAX_GUST, WIND_AVG, RAIN_24H, RAIN_STATE, N
    };
    float v[N] = {0};
    uint32_t seen = 0;
    void set(uint8_t i, float x) { v[i] = x; seen |= (1u << i); }
    bool has(uint8_t i) const { return (seen & (1u << i)) != 0; }
};

// Persistent per-sensor config (flash): one slot per learned sensor, keyed by (family, id).
struct SensorSlot {
    uint8_t family;         // Family enum value
    uint8_t id0, id1, id2;  // 24-bit sensor ID, big-endian
    uint8_t channel;        // user-assigned channel (0-15)
    uint8_t enabled;        // 1 = enabled
    char name[16];          // user-assigned friendly name (empty = auto family name)
};
static constexpr uint8_t SENSOR_MAX = 24;

// Fixed-size mirror of a Server — std::string can't be memcpy'd, so the upload-server list
// is serialized to/from this POD. Sized so 3 servers + the config blob + the 13 template
// switches' restore_mode all fit the 512 B flash preference sector. Wunderground's host/path
// are hardcoded constants, so only Ecowitt IPs, ecowitt.net/MQTT paths and WU station IDs are
// stored (an extra-Ecowitt webhook path >24 chars would truncate — rare, the primary is HA).
static constexpr uint8_t SERVER_MAX = 3;
struct PersistedServer {
    uint8_t proto;       // Protocol enum value
    uint8_t enabled;
    uint16_t port;
    char host[16];
    char path[24];
    char key[16];
};
static constexpr size_t SERVERS_BLOB = 1 + SERVER_MAX * sizeof(PersistedServer);  // 1 + 3*60 = 181 B

// Compact persisted rain-aggregate state (the RainAggregator's first bucket, minus the
// short-lived hour_mm / last_rain_ms / last_sample_ms which reset naturally). Enough to keep
// the monthly/yearly totals correct across a reboot — the OEM's parity gap.
struct RainPersist {
    uint32_t key = 0;
    float last_counter = -1.0f;
    float event_mm = 0, day_mm = 0, week_mm = 0, month_mm = 0, year_mm = 0;
    uint16_t mark_doy = 0;
    uint8_t mark_month = 0;
    uint16_t mark_year = 0;
};

// Persisted per-sensor disable list. Only the exceptions are stored (the roster re-learns
// every sensor on boot and defaults to enabled), so this is just the (family, id) pairs the
// user has switched off. name/channel are not persisted — name is no longer exposed in the
// UI, and channel is re-derived from the decoder each time.
static constexpr uint8_t DISABLED_MAX = 8;
struct DisabledList {
    uint8_t count = 0;
    struct { uint8_t family; uint8_t id0, id1, id2; } s[DISABLED_MAX];
};

class EcowittRFSensor : public PollingComponent {
 public:
    // -- config (set by the Python schema / YAML) --
    uint8_t device_type = 1;         // 0=433, 1=868, 2=915, 3=920 MHz (compile default)
    UploadConfig upload;
    RemoteIndoorChannels remote_indoor_channels_;
    void set_remote_indoor_channel(uint32_t id, uint8_t channel) {
        remote_indoor_channels_.set(id, channel);
    }
    Server extra_servers_[SERVER_MAX];
    size_t extra_count_ = 0;
    int extra_code_[SERVER_MAX] = {0, 0, 0};
    i2c::I2CBus *i2c_bus_{nullptr};
    IndoorSensor indoor_;
    RainAggregator rain_;
    DisabledList disabled_;

    // The 13 config switches (1 upload + 12 family). The component owns their state in
    // cfg_ (upload_enabled / family_enable) and pushes it back on boot, so the templates
    // use restore_mode: DISABLED and no longer each burn a flash preference.
    switch_::Switch *upload_switch_{nullptr};
    switch_::Switch *family_switches_[12] = {nullptr};

    // Wind time-window state (RAM-only; feeds maxdailygust / winddir_avg10m).
    // Rolling rain is held per sensor by RainAggregator.
    float daily_max_gust_ = 0;     // m/s, max gust since midnight
    uint16_t gust_day_ = 0;        // local day-of-year at last reset
    struct DirSample { uint16_t deg; uint32_t ms; };
    DirSample dir_buf_[72];        // ~10 min of ~8.5 s wind readings
    uint8_t dir_head_ = 0, dir_count_ = 0;

    // -- exposed sensors (optional; one slot per family) --
    // temp/humidity/pressure are the BUILT-IN I2C sensor's, written only by update().
    sensor::Sensor *temp = nullptr;
    sensor::Sensor *humidity = nullptr;
    sensor::Sensor *pressure = nullptr;
    // ...and these are the primary outdoor array's, written only by publish(). Keeping them
    // apart is what stops the indoor entity alternating with the outdoor value.
    sensor::Sensor *outdoor_temp = nullptr;
    sensor::Sensor *outdoor_humidity = nullptr;
    sensor::Sensor *wind_speed = nullptr;
    sensor::Sensor *wind_dir = nullptr;
    sensor::Sensor *gust = nullptr;
    sensor::Sensor *rain = nullptr;
    sensor::Sensor *rain_rate = nullptr;
    sensor::Sensor *uv = nullptr;
    sensor::Sensor *light = nullptr;
    sensor::Sensor *soil_moisture = nullptr;
    sensor::Sensor *soil_temp = nullptr;
    sensor::Sensor *soil_ec = nullptr;
    sensor::Sensor *pm25 = nullptr;
    sensor::Sensor *pm10 = nullptr;
    sensor::Sensor *co2 = nullptr;
    sensor::Sensor *leaf_wetness = nullptr;
    sensor::Sensor *lightning_strikes = nullptr;
    sensor::Sensor *lightning_distance = nullptr;
    sensor::Sensor *leak = nullptr;

    // -- output-only text sensors --
    text_sensor::TextSensor *roster = nullptr;
    text_sensor::TextSensor *device_info = nullptr;

    // -- time clock (for runtime timezone apply) --
    time::RealTimeClock *time_clock = nullptr;

    // -- web dashboard state --
    UiState ui_;
    bool up_ok_ = false;
    uint32_t up_last_ = 0;      // unix seconds of last upload attempt
    int up_code_ = 0;
    int cloud_code_ = 0;        // Ecowitt cloud (OEM) last HTTP status / diagnostic
    bool web_registered_ = false;
    bool state_response_active_ = false;
    uint32_t state_rejected_ = 0;
    ESPPreferenceObject rain_rtc_pref_;
    RainCheckpoint rain_pending_;
    bool rain_restore_pending_ = false;
    bool rain_restored_ = false;
    bool rain_rtc_saved_ = false;

    // -- RF diagnostics (exposed on /state for live RX debugging) --
    int8_t rssi_dbm_ = 0;      // CMT2300A reg 0x70 - 128 (live RSSI, dBm)
    int8_t last_rssi_ = 0;     // RSSI latched at the most recent PKT_OK (reg 0x70 - 128)
    // Frames arriving below this are on the receiver's noise floor and are dropped before
    // their decode is acted on. A real sensor at a sane install is >= -100 dBm; the phantoms
    // this gateway accumulated were all -109..-114 dBm.
    static constexpr int8_t RSSI_FLOOR_DBM = -105;
    uint16_t rejected_rssi_ = 0;   // noise-floor frames dropped (diagnostic, /state)
    uint32_t last_rf_upload_ms_ = 0;  // rate limit for the BLOCKING HTTP upload path
    uint8_t reg01_ = 0;        // reg 0x01 part number (0x66 = CMT2300A, 0xB3 = CMT2219A)
    uint8_t reg02_ = 0;        // reg 0x02 part number low (0xB376 → 0x76)
    uint8_t reg03_ = 0;        // reg 0x03 PLL/freq (868 table = 0x15)
    uint8_t reg04_ = 0;        // reg 0x04 PLL/freq (868 table = 0x0D)
    uint8_t reg17_ = 0;        // reg 0x17 sync word (868 table = 0xD4)
    uint8_t reg18_ = 0;        // reg 0x18 freq (868 band table = 0x46)
    uint8_t reg6d_ = 0;        // reg 0x6D CUS_INT_FLAG (PKT_DONE etc.)
    uint8_t reg6e_ = 0;        // reg 0x6E CUS_FIFO_FLAG (RX_FIFO_NMTY etc.)
    uint8_t reg6f_ = 0;        // reg 0x6F CUS_RSSI_CODE (raw RSSI — nonzero = RF energy present)
    uint8_t reg61_ = 0;        // reg 0x61 CUS_MODE_STA (low nibble 0x05 = RX)
    uint8_t reg3f_ = 0;        // reg 0x3F INT_EN (init writes 0xFF)
    uint8_t reg3f_test_ = 0;   // write-read self-test: write 0x5A to 0x3F, read back
    uint8_t reg43_ = 0;        // reg 0x43 INT flag (bit0 = PKT_DONE — did a packet arrive?)
    uint8_t reg47_ = 0;        // reg 0x47 CMT2219A mode/status (read-back = OP_STATUS)
    uint8_t reg45_ = 0;        // reg 0x45 CMT2219A RSSI_VALUE (the REAL RSSI on 2219A)
    uint8_t rssi_min_ = 0, rssi_max_ = 0, rssi_changes_ = 0;  // RSSI fluctuation over 1.2s
    std::string last_hex_;     // hex of the most recent FIFO drain (first 24 bytes)
    std::string regdump_;      // full config register dump 0x00..0x5F (hex, for diff vs init)

    // -- setters (called by the Python schema / YAML template automations) --
    void set_device_type(uint8_t v) { device_type = v; }
    void set_upload_host(const std::string &v) { upload.host = v; }
    void set_upload_path(const std::string &v) { upload.path = v; }
    void set_upload_port(uint16_t v) { upload.port = v; }
    void set_upload_passkey(const std::string &v) { upload.passkey = v; }
    void set_upload_stationtype(const std::string &v) { upload.stationtype = v; }
    void set_upload_freq(const std::string &v) { upload.freq = v; }
    void set_time_clock(time::RealTimeClock *t) { time_clock = t; }
    void set_i2c_bus(i2c::I2CBus *bus) { i2c_bus_ = bus; }
    void set_upload_switch(switch_::Switch *s) { upload_switch_ = s; }
    void set_family_switch(uint8_t group, switch_::Switch *s) {
        if (group < 12) family_switches_[group] = s;
    }
    void set_roster(text_sensor::TextSensor *s) { roster = s; }
    void set_device_info(text_sensor::TextSensor *s) { device_info = s; }

    void set_temp(sensor::Sensor *s) { temp = s; }
    void set_humidity(sensor::Sensor *s) { humidity = s; }
    void set_pressure(sensor::Sensor *s) { pressure = s; }
    void set_outdoor_temp(sensor::Sensor *s) { outdoor_temp = s; }
    void set_outdoor_humidity(sensor::Sensor *s) { outdoor_humidity = s; }
    void set_wind_speed(sensor::Sensor *s) { wind_speed = s; }
    void set_wind_dir(sensor::Sensor *s) { wind_dir = s; }
    void set_gust(sensor::Sensor *s) { gust = s; }
    void set_rain(sensor::Sensor *s) { rain = s; }
    void set_rain_rate(sensor::Sensor *s) { rain_rate = s; }
    void set_uv(sensor::Sensor *s) { uv = s; }
    void set_light(sensor::Sensor *s) { light = s; }
    void set_soil_moisture(sensor::Sensor *s) { soil_moisture = s; }
    void set_soil_temp(sensor::Sensor *s) { soil_temp = s; }
    void set_soil_ec(sensor::Sensor *s) { soil_ec = s; }
    void set_pm25(sensor::Sensor *s) { pm25 = s; }
    void set_pm10(sensor::Sensor *s) { pm10 = s; }
    void set_co2(sensor::Sensor *s) { co2 = s; }
    void set_leaf_wetness(sensor::Sensor *s) { leaf_wetness = s; }
    void set_lightning_strikes(sensor::Sensor *s) { lightning_strikes = s; }
    void set_lightning_distance(sensor::Sensor *s) { lightning_distance = s; }
    void set_leak(sensor::Sensor *s) { leak = s; }

    // -- runtime config setters (persist + apply) --
    void set_cfg_host(const std::string &v) {
        strncpy(cfg_.host, v.c_str(), sizeof(cfg_.host) - 1);
        save_cfg();
        apply_cfg();
    }
    void set_cfg_path(const std::string &v) {
        strncpy(cfg_.path, v.c_str(), sizeof(cfg_.path) - 1);
        save_cfg();
        apply_cfg();
    }
    void set_cfg_passkey(const std::string &v) {
        strncpy(cfg_.passkey, v.c_str(), sizeof(cfg_.passkey) - 1);
        save_cfg();
        apply_cfg();
    }
    void set_cfg_port(uint16_t v) {
        cfg_.port = v;
        save_cfg();
        apply_cfg();
    }
    void set_cfg_interval(uint16_t v) {
        cfg_.interval_s = (v < 5) ? 5 : v;
        save_cfg();
        apply_cfg();
    }
    void set_cfg_timezone(const std::string &v) {
        strncpy(cfg_.timezone, v.c_str(), sizeof(cfg_.timezone) - 1);
        save_cfg();
        apply_cfg();
    }
    void set_cfg_band(uint8_t v) {   // applied on next boot (radio re-init)
        cfg_.band = (v > 3) ? 1 : v;
        save_cfg();
    }
    void set_cfg_upload_enabled(bool v) {
        cfg_.upload_enabled = v;
        save_cfg();
        apply_cfg();
        if (upload_switch_) upload_switch_->publish_state(v);
    }
    void set_cfg_cloud(bool v) {
        cfg_.cloud_enabled = v;
        save_cfg();
    }
    void set_family_enabled(uint8_t group, bool v) {
        if (group > 15)
            return;
        if (v)
            cfg_.family_enable |= (1u << group);
        else
            cfg_.family_enable &= ~(1u << group);
        save_cfg();
        if (group < 12 && family_switches_[group]) family_switches_[group]->publish_state(v);
    }
    void set_cal_temp(float v) { cfg_.cal_temp = v; save_cfg(); }
    void set_cal_hum(float v) { cfg_.cal_hum = v; save_cfg(); }
    void set_cal_press(float v) { cfg_.cal_press = v; save_cfg(); }
    void set_cal_rain(float v) { cfg_.cal_rain = v; save_cfg(); }
    void set_cal_wind(float v) { cfg_.cal_wind = v; save_cfg(); }
    void set_cal_solar(float v) { cfg_.cal_solar = v; save_cfg(); }
    void set_cal_uv(float v) { cfg_.cal_uv = v; save_cfg(); }

    // ---- dashboard: map a decoded reading into a UiState (family-aware keys) ----
    // Static so it can fill BOTH the shared latest-values map (ui_) and each roster entry's
    // own per-sensor readings — this is what lets a remote WH25 show its OWN T/H/P card
    // alongside the gateway's built-in indoor sensor.
    static void map_reading(const Reading &r, UiState &u) {
        switch (r.family) {
            case Family::WH25:
                if (r.has(F_TEMP)) u.set(UiState::T_IN, r.temp_c);
                if (r.has(F_HUM)) u.set(UiState::H_IN, r.humidity);
                if (r.has(F_PRESS)) u.set(UiState::P, r.pressure_hpa);
                break;
            case Family::WH31:
                if (r.has(F_TEMP)) u.set(UiState::T_OUT, r.temp_c);
                if (r.has(F_HUM)) u.set(UiState::H_OUT, r.humidity);
                break;
            case Family::WH24: case Family::WH65: case Family::WS69:
                if (r.has(F_WDIR)) u.set(UiState::WDIR, r.wind_dir_deg);
                if (r.has(F_WIND)) u.set(UiState::WSPD, r.wind_mps);
                if (r.has(F_GUST)) u.set(UiState::WGUST, r.gust_mps);
                if (r.rain_mm > 0) u.set(UiState::RAIN, r.rain_mm);
                u.set(UiState::RAIN_RATE, r.rain_rate_mmh);
                if (r.has(F_UV)) u.set(UiState::UV, r.uv);
                if (r.has(F_LIGHT)) u.set(UiState::SOLAR, r.light_wm2);
                if (r.has(F_TEMP)) u.set(UiState::T_OUT, r.temp_c);
                if (r.has(F_HUM)) u.set(UiState::H_OUT, r.humidity);
                // Pressure card is owned by the built-in barometer — see the
                // barometer-ownership note in ecowitt_upload.h.
                break;
            case Family::WS68: case Family::WS80: case Family::WS85: case Family::WS90:
                if (r.has(F_WDIR)) u.set(UiState::WDIR, r.wind_dir_deg);
                if (r.has(F_WIND)) u.set(UiState::WSPD, r.wind_mps);
                if (r.has(F_GUST)) u.set(UiState::WGUST, r.gust_mps);
                if (r.rain_mm > 0) u.set(UiState::RAIN, r.rain_mm);
                u.set(UiState::RAIN_RATE, r.rain_rate_mmh);
                if (r.has(F_UV)) u.set(UiState::UV, r.uv);
                if (r.has(F_LIGHT)) u.set(UiState::SOLAR, r.light_wm2);
                if (r.has(F_TEMP)) u.set(UiState::T_OUT, r.temp_c);
                if (r.has(F_HUM)) u.set(UiState::H_OUT, r.humidity);
                if (r.has(F_BATV)) u.set(UiState::BAT_V, r.battery_v);
                if (r.has(F_CAPV)) u.set(UiState::CAP_V, r.cap_v);
                if (r.daily_gust_mps > 0) u.set(UiState::MAX_GUST, r.daily_gust_mps);
                if (r.wind_dir_avg_deg > 0) u.set(UiState::WIND_AVG, r.wind_dir_avg_deg);
                if (r.last24h_rain_mm >= 0) u.set(UiState::RAIN_24H, r.last24h_rain_mm);
                u.set(UiState::RAIN_STATE, r.rain_state ? 1 : 0);
                break;
            case Family::WH40: case Family::WN20:
                if (r.rain_mm > 0) u.set(UiState::RAIN, r.rain_mm);
                break;
            case Family::WH51:
                if (r.moisture_pct) u.set(UiState::SOIL_MOIST, r.moisture_pct);
                break;
            case Family::WN34: case Family::WN38:
                if (r.soil_temp_c != 0) u.set(UiState::SOIL_T, r.soil_temp_c);
                break;
            case Family::WH52:
                if (r.moisture_pct) u.set(UiState::SOIL_MOIST, r.moisture_pct);
                if (r.soil_temp_c != 0) u.set(UiState::SOIL_T, r.soil_temp_c);
                if (r.soil_ec_uscm > 0) u.set(UiState::SOIL_EC, r.soil_ec_uscm);
                break;
            case Family::WH0290: case Family::WH43:
                if (r.pm25_ugm3) u.set(UiState::PM25, r.pm25_ugm3);
                if (r.pm10_ugm3) u.set(UiState::PM10, r.pm10_ugm3);
                break;
            case Family::WH45: case Family::WH46:
                if (r.pm25_ugm3) u.set(UiState::PM25, r.pm25_ugm3);
                if (r.co2_ppm) u.set(UiState::CO2, r.co2_ppm);
                break;
            case Family::WH57:
                if (r.strikes) u.set(UiState::LIGHT_N, r.strikes);
                if (r.lightning_dist_km > 0) u.set(UiState::LIGHT_D, r.lightning_dist_km);
                break;
            case Family::WH55: case Family::WH54:
                u.set(UiState::LEAK, r.leak ? 1 : 0);
                break;
            case Family::WH35:
                if (r.leaf_wetness) u.set(UiState::LEAF, r.leaf_wetness);
                break;
            default: break;
        }
    }

    // Fold into the SHARED latest-values map (dashboard fixed cards). The built-in I2C sensor
    // (read in update()) owns the indoor card, so a remote WH25 does NOT clobber it here —
    // but map_reading() still assigns the WH25's own T/H/P for its per-sensor roster card.
    void update_ui_state(const Reading &r) {
        if (r.family == Family::WH25)
            return;
        map_reading(r, ui_);
    }

    // ---- dashboard HTTP routes ----
    void register_web() {
        if (web_registered_)
            return;
        auto *base = web_server_base::global_web_server_base;
        if (base == nullptr)
            return;
        if (base->get_server() == nullptr)
            base->init();  // begin the shared server (captive_portal only inits it in AP mode)
        AsyncWebServer *srv = base->get_server();
        if (srv == nullptr)
            return;
        srv->on("/", HTTP_GET, [this](AsyncWebServerRequest *req) { handle_index(req); });
        srv->on("/state", HTTP_GET, [this](AsyncWebServerRequest *req) { handle_state(req); });
        srv->on("/health", HTTP_GET, [this](AsyncWebServerRequest *req) { handle_state(req, true); });
        srv->on("/config", HTTP_POST,
                [this](AsyncWebServerRequest *req) {
                    if (req->contentLength() > 2048) {
                        req->send(413, "application/json", "{\"ok\":false,\"err\":\"body too large\"}");
                    } else if (req->_tempObject == nullptr) {
                        req->send(503, "application/json", "{\"ok\":false,\"err\":\"body unavailable\"}");
                    } else {
                        handle_config(req, static_cast<const uint8_t *>(req->_tempObject), req->contentLength());
                    }
                }, nullptr,
                [](AsyncWebServerRequest *req, uint8_t *data, size_t len, size_t index, size_t total) {
                    if (total == 0 || total > 2048 || index > total || len > total - index) return;
                    if (index == 0 && ESP.getFreeHeap() >= total + 6000 &&
                        ESP.getMaxFreeBlockSize() >= total + 32)
                        req->_tempObject = std::malloc(total + 1);
                    if (req->_tempObject) {
                        auto *body = static_cast<uint8_t *>(req->_tempObject);
                        std::memcpy(body + index, data, len);
                        body[total] = 0;
                    }
                    // ESPAsyncWebServer frees _tempObject when this request is destroyed.
                });
        web_registered_ = true;
    }

    void handle_index(AsyncWebServerRequest *req) {
        req->send_P(200, "text/html; charset=utf-8", DASHBOARD_HTML);
    }

    void handle_state(AsyncWebServerRequest *req, bool health = false) {
        if (state_response_active_ || ESP.getFreeHeap() < 6500) {
            ++state_rejected_; req->send(503, "application/json", "{\"err\":\"busy or low heap\"}"); return;
        }
        const bool want_regs = !health && req->hasParam("regs");
        if (want_regs) {
            char hx[3]; regdump_.clear();
            for (uint8_t i = 0; i < 0x60; ++i) {
                std::snprintf(hx, sizeof(hx), "%02X", read_reg(i)); regdump_ += hx;
            }
        }
        BoundedJson measure;
        write_state(measure, health, want_regs);
        size_t capacity = measure.size() + 32;  // heap/uptime digit changes between passes
        if (!state_response_fits(capacity, ESP.getFreeHeap(), ESP.getMaxFreeBlockSize())) {
            ++state_rejected_; req->send(503, "application/json", "{\"err\":\"low heap\"}"); return;
        }
        char *body = static_cast<char *>(std::malloc(capacity));
        if (!body) { ++state_rejected_; req->send(503); return; }
        BoundedJson j(body, capacity);
        write_state(j, health, want_regs);
        if (!j.fits()) { std::free(body); req->send(503); return; }
        auto *response = new (std::nothrow) OwnedJsonResponse(body, j.size(), &state_response_active_);
        if (!response) { std::free(body); req->send(503); return; }
        state_response_active_ = true;
        req->send(response);
    }

    void write_state(BoundedJson &j, bool health, bool want_regs) {
        j.object(); j.object("dev");
        j.field("name", "GW1100");
        j.field("mac", WiFi.macAddress().c_str());
        j.field("ip", WiFi.localIP().toString().c_str());
        j.field("ssid", WiFi.SSID().c_str());
        j.field("wifi_rssi", WiFi.RSSI());
        j.field("heap", (uint32_t)ESP.getFreeHeap());
        j.field("uptime", (uint32_t)(millis() / 1000));
        j.field("fw", ESPHOME_VERSION);
        j.field("build", "rain-recovery-20261016");
        j.field("ts", (uint32_t)::time(nullptr));
        j.field("max_block", ESP.getMaxFreeBlockSize());
        j.field("reset_reason", ESP.getResetInfoPtr()->reason);
        j.field("reset_exception", ESP.getResetInfoPtr()->exccause);
        j.field("reset_pc", ESP.getResetInfoPtr()->epc1);
        j.end_object();
        j.object("up"); j.field("ok", up_ok_); j.field("last", up_last_);
        j.field("code", up_code_); j.field("cloud", cloud_code_); j.end_object();
        j.object("recovery"); j.field("rain_pending", rain_restore_pending_);
        j.field("rain_restored", rain_restored_); j.field("rain_rtc_saved", rain_rtc_saved_);
        j.field("state_rejected", state_rejected_); j.end_object();
        write_values(j, "r", ui_);
        if (!health) {
            j.object("diag");
        j.field("rssi", rssi_dbm_);
        j.field("reg01", reg01_);
        j.field("reg02", reg02_);
        j.field("reg03", reg03_);
        j.field("reg04", reg04_);
        j.field("reg17", reg17_);
        j.field("reg18", reg18_);
        j.field("reg6d", reg6d_);
        j.field("reg6e", reg6e_);
        j.field("reg6f", reg6f_);
        j.field("reg61", reg61_);
        if (want_regs) j.field("regdump", regdump_.c_str());
        j.field("rej_rssi", rejected_rssi_);
        j.field("reg3f", reg3f_);
        j.field("reg3f_test", reg3f_test_);
        j.field("reg43", reg43_);
        j.field("reg47", reg47_);
        j.field("reg45", reg45_);
        j.field("rssi_min", rssi_min_);
        j.field("rssi_max", rssi_max_);
        j.field("rssi_changes", rssi_changes_);
        j.field("last_hex", last_hex_.c_str());
            j.end_object();
            j.array("roster");
            uint32_t now_unix = (uint32_t)::time(nullptr);
            for (size_t i = 0; i < roster_count_; ++i) {
                j.object(); const auto &r = roster_[i];
                j.field("fam", family_tag(r.family)); char id[8];
                std::snprintf(id, sizeof(id), "%06X", r.id & 0xFFFFFF); j.field("id", id);
                int16_t idx = find_sensor((uint8_t)r.family, r.id);
                j.field("ch", idx >= 0 ? sensor_slots_[idx].channel : r.channel);
                j.field("en", idx >= 0 ? sensor_slots_[idx].enabled != 0 : true);
                j.field("name", idx >= 0 ? sensor_slots_[idx].name : "");
                j.field("rssi", r.rssi);
                uint32_t age = (millis() - r.last_ms) / 1000;
                j.field("last", now_unix >= age ? now_unix - age : 0);
                j.field("batt", r.battery_low); write_values(j, "v", r.v); j.end_object();
            }
            j.end_array(); write_config(j);
        }
        j.end_object();
    }

    static void write_values(BoundedJson &j, const char *key, const UiState &u) {
        static const char *const KEYS[UiState::N] = {
            "t_in", "h_in", "p", "t_out", "h_out", "leaf_wet", "wind_dir", "wind_spd",
            "wind_gust", "rain", "rain_rate", "uv", "solar", "soil_moist", "soil_t",
            "soil_ec", "pm25", "pm10", "co2", "light_cnt", "light_dist", "leak",
            "batt_v", "cap_v", "max_gust", "wind_avg", "rain_24h", "rain_state"};
        j.object(key);
        for (uint8_t i = 0; i < UiState::N; ++i) if (u.has(i)) j.field(KEYS[i], u.v[i]);
        j.end_object();
    }

    void write_config(BoundedJson &j) {
        j.object("cfg"); j.object("up");
        j.field("en", cfg_.upload_enabled); j.field("cloud", cfg_.cloud_enabled);
        j.field("host", cfg_.host); j.field("path", cfg_.path); j.field("port", cfg_.port);
        j.field("key", cfg_.passkey); j.field("int", cfg_.interval_s); j.end_object();
        j.array("servers");
        for (size_t i = 0; i < extra_count_; ++i) {
            const auto &s = extra_servers_[i]; j.object();
            j.field("proto", s.proto == Protocol::WUNDERGROUND ? "wunderground" :
                s.proto == Protocol::MQTT ? "mqtt" : "ecowitt");
            j.field("host", s.host.c_str()); j.field("port", s.port); j.field("path", s.path.c_str());
            j.field("key", s.key.c_str()); j.field("en", s.enabled); j.field("code", extra_code_[i]);
            j.end_object();
        }
        j.end_array(); j.field("tz", cfg_.timezone[0] ? cfg_.timezone : "Europe/London");
        j.field("rf", cfg_.band == 0 ? 433 : cfg_.band == 1 ? 868 : cfg_.band == 2 ? 915 : 920);
        j.object("cal"); j.field("t_off", cfg_.cal_temp); j.field("h_off", cfg_.cal_hum);
        j.field("p_off", cfg_.cal_press); j.field("rain_g", cfg_.cal_rain);
        j.field("wind_g", cfg_.cal_wind); j.field("solar_g", cfg_.cal_solar);
        j.field("uv_g", cfg_.cal_uv); j.end_object(); j.object("fam_en");
        static const char *const TAGS[] = {"WH25", "WH31", "WH51", "WN34", "WN38", "WH52", "WH24", "WH65", "WS69", "WH40",
            "WN20", "WS68", "WS80", "WS85", "WS90", "WH41", "WH43", "WH45", "WH46", "WH57", "WH55", "WH54", "WH35"};
        for (const char *t : TAGS) { int8_t g = tag_to_group(t); if (g >= 0) j.field(t, (cfg_.family_enable & (1u << g)) != 0); }
        j.end_object(); j.array("sensors");
        for (uint8_t i = 0; i < sensor_count_; ++i) {
            const auto &s = sensor_slots_[i]; j.object(); j.field("fam", family_tag((Family)s.family));
            char id[8]; uint32_t sid = ((uint32_t)s.id0 << 16) | ((uint32_t)s.id1 << 8) | s.id2;
            std::snprintf(id, sizeof(id), "%06X", sid); j.field("id", id);
            j.field("ch", s.channel); j.field("en", s.enabled != 0); j.end_object();
        }
        j.end_array(); j.end_object();
    }

    void handle_config(AsyncWebServerRequest *req, const uint8_t *body, size_t length) {
        bool ok = json::parse_json(body, length, [this](JsonObject root) -> bool {
            if (!root["rain_history"].isNull()) {
                if (root.size() != 1 || rain_.count() == 0) return false;
                JsonObject history = root["rain_history"].as<JsonObject>();
                const char *fam = history["fam"].as<const char *>();
                const char *id = history["id"].as<const char *>();
                if (!fam || !id || tag_to_family(fam) == Family::Unknown || std::strlen(id) != 6) return false;
                char *end = nullptr; uint32_t sid = (uint32_t)std::strtoul(id, &end, 16);
                if (!end || *end) return false;
                uint32_t key = ((uint32_t)tag_to_family(fam) << 24) | sid;
                auto &b = rain_.buckets()[0];
                if (key != b.key) return false;
                JsonArray samples = history["samples"].as<JsonArray>();
                if (samples.size() < 2 || samples.size() > 64) return false;
                RainHistoryPoint points[64]; size_t count = 0;
                for (JsonVariant sample : samples) {
                    JsonArray row = sample.as<JsonArray>();
                    if (row.size() != 2 || !row[0].is<uint32_t>() || !row[1].is<float>()) return false;
                    points[count++] = {row[0].as<uint32_t>(), row[1].as<float>()};
                }
                uint32_t now = (uint32_t)::time(nullptr);
                if (!restore_rain_history(points, count, b.last_counter, now, millis(), b.rolling, b.recent)) return false;
                rain_restore_pending_ = false; rain_restored_ = true; checkpoint_rain(now);
                return rain_rtc_saved_;
            }
            if (JsonObject up = root["up"].as<JsonObject>()) {
                if (!up["en"].isNull()) set_cfg_upload_enabled(up["en"].as<bool>());
                if (!up["cloud"].isNull()) set_cfg_cloud(up["cloud"].as<bool>());
                if (!up["host"].isNull()) set_cfg_host(up["host"].as<const char *>());
                if (!up["path"].isNull()) set_cfg_path(up["path"].as<const char *>());
                if (!up["key"].isNull()) set_cfg_passkey(up["key"].as<const char *>());
                if (!up["port"].isNull()) set_cfg_port((uint16_t)up["port"].as<int>());
                if (!up["int"].isNull()) set_cfg_interval((uint16_t)up["int"].as<int>());
            }
            if (!root["tz"].isNull()) set_cfg_timezone(root["tz"].as<const char *>());
            if (!root["rf"].isNull()) {
                int rf = root["rf"].as<int>();
                set_cfg_band((uint8_t)(rf == 433 ? 0 : rf == 868 ? 1 : rf == 915 ? 2 : 3));
            }
            // ★ One-shot purge of the learned-sensor list, the live roster and the
            // disabled list:  POST /config  {"clear_sensors":true}
            // Needed once after the anti-phantom fixes — the noise-minted WH25/WH32 entries
            // this gateway already accumulated live in flash, and nothing else evicts them.
            if (!root["clear_sensors"].isNull() && root["clear_sensors"].as<bool>()) {
                sensor_count_ = 0;
                roster_count_ = 0;
                disabled_.count = 0;
                disabled_pref_.save(&disabled_);
                save_sensors();
            }
            if (JsonObject cal = root["cal"].as<JsonObject>()) {
                if (!cal["t_off"].isNull()) set_cal_temp(cal["t_off"].as<float>());
                if (!cal["h_off"].isNull()) set_cal_hum(cal["h_off"].as<float>());
                if (!cal["p_off"].isNull()) set_cal_press(cal["p_off"].as<float>());
                if (!cal["rain_g"].isNull()) set_cal_rain(cal["rain_g"].as<float>());
                if (!cal["wind_g"].isNull()) set_cal_wind(cal["wind_g"].as<float>());
                if (!cal["solar_g"].isNull()) set_cal_solar(cal["solar_g"].as<float>());
                if (!cal["uv_g"].isNull()) set_cal_uv(cal["uv_g"].as<float>());
            }
            if (JsonObject fe = root["fam_en"].as<JsonObject>()) {
                for (auto kv : fe) {
                    int8_t g = tag_to_group(kv.key().c_str());
                    if (g >= 0)
                        set_family_enabled((uint8_t)g, kv.value().as<bool>());
                }
            }
            if (JsonArray sa = root["sensors"].as<JsonArray>()) {
                for (JsonVariant v : sa) {
                    JsonObject s = v.as<JsonObject>();
                    Family f = tag_to_family(s["fam"].as<const char *>());
                    if (f == Family::Unknown)
                        continue;
                    const char *idstr = s["id"].as<const char *>();
                    if (idstr == nullptr)
                        continue;
                    uint32_t id = (uint32_t)strtoul(idstr, nullptr, 16);
                    uint8_t ch = s["ch"].isNull() ? 0 : s["ch"].as<uint8_t>();
                    bool en = s["en"].isNull() ? true : s["en"].as<bool>();
                    // Enable-only toggle (no "ch" sent) must preserve the learned channel.
                    if (s["ch"].isNull()) {
                        int16_t ex = find_sensor((uint8_t)f, id);
                        ch = (ex >= 0) ? sensor_slots_[ex].channel : 0;
                    }
                    set_sensor((uint8_t)f, id, ch, en);
                    const char *nm = s["name"].as<const char *>();
                    if (nm)
                        set_sensor_name((uint8_t)f, id, nm);
                }
            }
            if (JsonArray srv = root["servers"].as<JsonArray>()) {
                extra_count_ = 0;
                for (JsonVariant v : srv) {
                    if (extra_count_ >= SERVER_MAX)
                        break;
                    JsonObject s = v.as<JsonObject>();
                    Server &d = extra_servers_[extra_count_];
                    const char *p = s["proto"].as<const char *>();
                    d.proto = (p && strcmp(p, "wunderground") == 0) ? Protocol::WUNDERGROUND
                            : (p && strcmp(p, "mqtt") == 0) ? Protocol::MQTT : Protocol::ECOWITT;
                    d.host = s["host"].as<const char *>() ? s["host"].as<const char *>() : "";
                    d.port = s["port"].isNull() ? 80 : (uint16_t)s["port"].as<int>();
                    d.path = s["path"].as<const char *>() ? s["path"].as<const char *>() : "";
                    d.key = s["key"].as<const char *>() ? s["key"].as<const char *>() : "";
                    d.enabled = s["en"].isNull() ? false : s["en"].as<bool>();
                    extra_count_++;
                }
                save_servers();
            }
            return true;
        });
        req->send(ok ? 200 : 400, "application/json", ok ? "{\"ok\":true}" : "{\"ok\":false}");
    }

    void setup() override {
        pinMode(ecowitt_rf::PIN_DOUT, INPUT);
        setup_prefs();
        load_cfg();
        load_servers();
        load_rain();
        load_disabled();
        apply_cfg();
        // Push the persisted config back to the 13 template switches (they use
        // restore_mode: DISABLED, so this component is their source of truth).
        if (upload_switch_)
            upload_switch_->publish_state(cfg_.upload_enabled);
        for (uint8_t g = 0; g < 12; g++)
            if (family_switches_[g])
                family_switches_[g]->publish_state((cfg_.family_enable & (1u << g)) != 0);
        ecowitt_rf::init(device_type);
        indoor_.set_bus(i2c_bus_);
        indoor_.probe();
        register_web();
    }

    void loop() override {
        register_web();  // retry until the shared HTTP server (captive_portal) is up

        // Fast RX poll — mirrors the stock's ~1 ms GPIO15 loop. The CMT2219A data-ready is
        // a per-byte FIFO_WBYTE pulse, so the old 16 s update() poll could never catch it.
        // Drain immediately on data-ready, then decode/publish/upload.
        uint8_t packet[69];
        size_t n = receive_packet(packet, sizeof(packet));
        if (n == 0)
            return;

        char hx[3];
        last_hex_.clear();
        for (size_t i = 0; i < n && i < 24; ++i) {
            std::snprintf(hx, sizeof(hx), "%02X", packet[i]);
            last_hex_ += hx;
        }

        ecowitt_rf::Reading r;
        if (!ecowitt_rf::decode(packet, n, r))
            return;
        r.rssi_dbm = last_rssi_;

        // ★ RSSI FLOOR — the cheapest and most effective anti-phantom defence. The real
        // sensors at this site sit around -77 dBm; all 21 phantom "WH25/WH32" units that
        // noise minted over 2.2 days sat at -109..-114 dBm, i.e. on the receiver's noise
        // floor. Rejecting sub-threshold frames HERE, before resolve_sensor(), also stops
        // them consuming a persistent sensor slot and triggering a flash write each time.
        if (r.rssi_dbm != 0 && r.rssi_dbm < RSSI_FLOOR_DBM) {
            rejected_rssi_++;
            return;
        }

        bool sen_enabled = resolve_sensor(r);   // per-sensor enable + assigned channel
        apply_calibration(r);

        // Fold rain-capable counters into the running aggregate (sampled for every heard
        // sensor so the delta baseline is always current, even for a disabled one).
        if (is_rain_family(r.family)) {
            RainAggregate agg;
            uint16_t doy = 0; uint8_t hour = 0, week = 0, month = 0; uint16_t year = 0;
            if (time_clock) {
                ESPTime t = time_clock->now();
                if (t.is_valid()) {
                    doy = t.day_of_year; hour = t.hour; month = t.month; year = t.year;
                    week = (uint8_t)((doy - 1) / 7);
                }
            }
            const uint32_t rain_key = ((uint32_t)r.family << 24) | (r.id & 0xFFFFFF);
            const uint32_t rain_now = (uint32_t)::time(nullptr);
            if (rain_restore_pending_ && rain_key == rain_pending_.key && year != 0) {
                auto *bucket = rain_.find(rain_key);
                rain_restored_ = rain_pending_.restore(rain_key, rain_now, millis(), bucket->rolling, bucket->recent);
                rain_restore_pending_ = false;
            }
            rain_.sample((uint8_t)r.family, r.id, r.rain_mm, doy, hour, week, month, year,
                         (uint32_t)millis(), agg);
            if (rain_restore_pending_ && rain_key == rain_pending_.key) {
                // NTP is not ready: unknown is safer than overwriting retained HA rain with zero.
                agg.last24h_mm = NAN; agg.rate_mmh = NAN;
            } else if (year != 0 && rain_.count() && rain_.buckets()[0].key == rain_key) {
                checkpoint_rain(rain_now);
            }
            r.rain_event_mm = agg.event_mm; r.rain_hour_mm = agg.hour_mm; r.rain_day_mm = agg.day_mm;
            r.rain_week_mm = agg.week_mm; r.rain_month_mm = agg.month_mm; r.rain_year_mm = agg.year_mm;
            r.rain_rate_mmh = agg.rate_mmh;
            r.last24h_rain_mm = agg.last24h_mm;
            if (rain_.take_dirty())
                save_rain();
        }

        apply_windows(r);
        bool was_seen = record_roster(r);   // after windows so the roster card carries max_gust/wind_avg/etc.

        if (!family_enabled(r.family) || !sen_enabled)
            return;  // discovered but disabled — still shown in the roster
        if (!was_seen)
            return;  // first sighting — don't upload until it repeats (phantom-entity guard)

        update_ui_state(r);
        publish(r);   // native API + MQTT: cheap and non-blocking, so every packet goes out

        // Each remote indoor unit gets its own MQTT identity and reports independently
        // of the outdoor array's HTTP rate limiter. The built-in reading uses model=0.
        if (is_remote_indoor(r)) {
            upload_reading(r);
            return;
        }

        // ★ The HTTP upload path uses BLOCKING sockets (one connect + read per target, and
        // there can be several). At one upload per received packet — a WS90 bursts every
        // ~8.8 s — that stalls loop() for seconds at a time and starves the fast RX poll
        // that actually drains the FIFO; the device logs "ecowitt_rf.sensor took a long time
        // for an operation (3052 ms)". So honour the "Upload Interval" number entity, which
        // until now was persisted and never enforced — despite its name it only ever set the
        // update() poll period.
        uint32_t now_ms = millis();
        if (last_rf_upload_ms_ == 0 ||
            (now_ms - last_rf_upload_ms_) >= (uint32_t)cfg_.interval_s * 1000UL) {
            last_rf_upload_ms_ = now_ms;
            upload_reading(r);
        }
    }

    void update() override {
        // Periodic (16 s): device info + register diagnostics + roster text.
        float it = 0, ih = 0, ip = 0;
        bool th_ok = indoor_.read_th(it, ih);
        bool p_ok = indoor_.read_pressure(ip);

        // Build the built-in reading FIRST, then calibrate it, then publish the CALIBRATED
        // values. This path used to skip apply_calibration() altogether, so cal_temp /
        // cal_hum / cal_press silently did nothing for the gateway's own I2C sensor even
        // though the YAML advertises them as "applied before publish + upload". It also
        // meant the built-in barometer and the RF sensors were offset from each other by
        // exactly cal_press. family=WH25 drives the reading_query field map (tempinc /
        // humidityin / baromrelhpa+baromabshpa); it is not an RF packet.
        Reading r;
        r.family = Family::WH25;
        if (th_ok) {
            r.temp_c = it; r.mark(F_TEMP);
            r.humidity = (int)(ih + 0.5f); r.mark(F_HUM);
        }
        if (p_ok) { r.pressure_hpa = ip; r.mark(F_PRESS); }
        apply_calibration(r);

        if (th_ok) {
            if (temp) temp->publish_state(r.temp_c);
            if (humidity) humidity->publish_state(r.humidity);
            ui_.set(UiState::T_IN, r.temp_c);   // built-in indoor sensor -> dashboard
            ui_.set(UiState::H_IN, r.humidity);
        }
        if (p_ok) {
            if (pressure) pressure->publish_state(r.pressure_hpa);
            ui_.set(UiState::P, r.pressure_hpa);
        }
        if (th_ok || p_ok) {
            record_builtin(r.temp_c, r.humidity, r.pressure_hpa);  // built-in roster card
            upload_reading(r);
        }
        publish_device_info();
        rssi_dbm_ = (int8_t)((int)read_reg(0x70) - 128);
        reg01_ = read_reg(0x01);
        reg02_ = read_reg(0x02);
        reg03_ = read_reg(0x03);
        reg04_ = read_reg(0x04);
        reg17_ = read_reg(0x17);
        reg18_ = read_reg(0x18);
        reg6d_ = read_reg(0x6D);
        reg6e_ = read_reg(0x6E);
        reg6f_ = read_reg(0x6F);
        // The 96-register config dump is no longer read here — it is built on demand in
        // handle_state() for /state?regs=1. Doing 96 bit-banged SPI reads every single
        // update() was pure overhead on the poll that also has to service the RX FIFO.
        reg61_ = read_reg(0x61);        // CUS_MODE_STA (low nibble 0x05 = RX)
        // NOTE: no 0x3F write canary — 0x3F is SYNC_VALUE<23:16>; writing it corrupts sync.
        reg43_ = read_reg(0x43);
        reg45_ = read_reg(0x45);
        reg47_ = read_reg(0x47);

        // Single-sample RSSI. The old 2 s blocking scan starved the fast RX poll in
        // loop(), which is what actually drains packets from the FIFO — a WH90 bursts
        // for only ~25 ms every ~8.8 s, so a 2 s freeze every 16 s was missing packets.
        uint8_t rnow = read_reg(0x70);
        rssi_min_ = rssi_max_ = rnow;
        rssi_changes_ = 0;

        publish_roster();
    }

 protected:
    DeviceConfig cfg_;
    // Preference objects are allocated ONCE in setup_prefs() and reused for load/save —
    // ESP8266's make_preference allocates a NEW flash offset on every call (no hash cache),
    // so re-creating them in each load/save would read/write different offsets.
    ESPPreferenceObject cfg_pref_;
    ESPPreferenceObject servers_pref_;
    ESPPreferenceObject rain_pref_;
    ESPPreferenceObject disabled_pref_;

    bool family_enabled(Family f) const {
        uint8_t g = family_group(f);
        return (cfg_.family_enable & (1u << g)) != 0;
    }

    // Gated on Reading::has() rather than on magic zeros, so a legitimate 0 °C / calm wind /
    // night-time UV is calibrated like any other reading instead of being treated as absent.
    // The humidity clamp is now only what it says it is — keeping an OFFSET inside 0..100.
    // It used to double as the last line of defence against un-sentinel-checked garbage,
    // which is why bad frames surfaced in HA as a suspiciously exact 100 %.
    void apply_calibration(Reading &r) const {
        if (r.has(F_TEMP)) r.temp_c += cfg_.cal_temp;
        if (r.has(F_HUM)) r.humidity = (int)std::clamp(r.humidity + (int)cfg_.cal_hum, 0, 100);
        if (r.has(F_PRESS)) r.pressure_hpa += cfg_.cal_press;
        if (r.rain_mm > 0) r.rain_mm *= cfg_.cal_rain;
        if (r.rain_rate_mmh > 0) r.rain_rate_mmh *= cfg_.cal_rain;
        if (r.has(F_WIND)) r.wind_mps *= cfg_.cal_wind;
        if (r.has(F_GUST)) r.gust_mps *= cfg_.cal_wind;
        if (r.has(F_LIGHT)) r.light_wm2 *= cfg_.cal_solar;
        if (r.has(F_UV)) r.uv = (int)std::lroundf(r.uv * cfg_.cal_uv);
    }

    // The families that ARE the station's primary outdoor array. Only these drive the shared
    // outdoor_temperature / outdoor_humidity sensors: multi-channel families (WH31) and
    // remote indoor units (WH25) publish through their own per-channel upload fields and
    // must not fight over a single entity.
    static bool is_primary_array(Family f) {
        switch (f) {
            case Family::WH24: case Family::WH65: case Family::WS69:
            case Family::WS68: case Family::WS80: case Family::WS85: case Family::WS90:
                return true;
            default:
                return false;
        }
    }

    // Derive the time-window fields: max daily gust, 10-min circular wind-direction
    // average. All RAM-only; rainfall windows live in the per-sensor aggregator.
    void apply_windows(Reading &r) {
        uint16_t doy = 1;
        if (time_clock) {
            ESPTime t = time_clock->now();
            if (t.is_valid()) doy = t.day_of_year;
        }
        uint32_t now = millis();

        // Max gust since local midnight.
        if (doy != gust_day_) { gust_day_ = doy; daily_max_gust_ = 0; }
        if (r.has(F_GUST) && r.gust_mps > daily_max_gust_) daily_max_gust_ = r.gust_mps;
        r.daily_gust_mps = daily_max_gust_;

        // 10-min circular-average wind direction (vector sum over a sliding window).
        // Gated on has() — due north is 0°, which the old `> 0` test silently discarded.
        if (r.has(F_WDIR)) {
            while (dir_count_ && now - dir_buf_[dir_head_].ms > 600000UL) {
                dir_head_ = (uint8_t)((dir_head_ + 1) % 72); dir_count_--;
            }
            uint8_t tail = (uint8_t)((dir_head_ + dir_count_) % 72);
            dir_buf_[tail].deg = (uint16_t)r.wind_dir_deg;
            dir_buf_[tail].ms = now;
            if (dir_count_ < 72) dir_count_++;
            else dir_head_ = (uint8_t)((dir_head_ + 1) % 72);
            float s = 0, c = 0;
            for (uint8_t i = 0; i < dir_count_; i++) {
                float a = dir_buf_[(uint8_t)((dir_head_ + i) % 72)].deg * 0.0174532925f;
                s += sinf(a); c += cosf(a);
            }
            float avg = atan2f(s, c) * 57.295779513f;
            if (avg < 0) avg += 360.0f;
            r.wind_dir_avg_deg = (int)(avg + 0.5f) % 360;
        }

    }

    // Poll GPIO15 (data-ready), burst-read the RX FIFO, re-arm. The decode dispatches on
    // buf[0] (sensor-type). Returns 0 if nothing was ready.
    size_t receive_packet(uint8_t *buf, size_t max) {
        if (!ecowitt_rf::rx_ready())
            return 0;
        // 0x70 is latched at PKT_OK; read it now (before re-arm clears the latch) so the
        // packet's actual signal strength is captured, not the stale 16 s diag value.
        last_rssi_ = (int8_t)((int)read_reg(0x70) - 128);
        size_t plen = ecowitt_rf::fifo_packet_len();  // 69 (CMT2300A) or 32 (CMT2219A)
        size_t n = (max < plen) ? max : plen;
        ecowitt_rf::fifo_read(buf, n);
        ecowitt_rf::rearm_rx();
        return n;
    }

    void publish(const ecowitt_rf::Reading &r) {
        // ★ The indoor temp/humidity/pressure sensors belong to the BUILT-IN I2C sensor and
        // are published from update(). An RF sensor must never write them. Until this split,
        // every RF packet wrote the same three Sensor objects the I2C sensor wrote, so
        // "GW1100 Indoor Humidity" was a 50/50 interleave of the console (52 %) and the WS90
        // outdoor array (77 %) — likewise 24.9 °C indoors alternating with 16.1 °C outdoors.
        // Outdoor T/H now has its own pair of sensors, driven only by the primary array.
        if (is_primary_array(r.family)) {
            if (outdoor_temp && r.has(F_TEMP))        outdoor_temp->publish_state(r.temp_c);
            if (outdoor_humidity && r.has(F_HUM))     outdoor_humidity->publish_state(r.humidity);
        }
        // Publish only the fields this family actually carried (see Reading::has()).
        if (wind_speed && r.has(F_WIND))              wind_speed->publish_state(r.wind_mps);
        if (wind_dir && r.has(F_WDIR))                wind_dir->publish_state(r.wind_dir_deg);
        if (gust && r.has(F_GUST))                    gust->publish_state(r.gust_mps);
        if (is_rain_family(r.family)) {
            if (rain) rain->publish_state(r.rain_mm);
            if (rain_rate) rain_rate->publish_state(r.rain_rate_mmh);
        }
        if (uv && r.has(F_UV))                        uv->publish_state(r.uv);
        if (light && r.has(F_LIGHT))                  light->publish_state(r.light_wm2);
        if (soil_moisture && r.moisture_pct)          soil_moisture->publish_state(r.moisture_pct);
        if (soil_temp && r.soil_temp_c != 0)          soil_temp->publish_state(r.soil_temp_c);
        if (soil_ec && r.soil_ec_uscm > 0)            soil_ec->publish_state(r.soil_ec_uscm);
        if (pm25 && r.pm25_ugm3)                      pm25->publish_state(r.pm25_ugm3);
        if (pm10 && r.pm10_ugm3)                      pm10->publish_state(r.pm10_ugm3);
        if (co2 && r.co2_ppm)                         co2->publish_state(r.co2_ppm);
        if (leaf_wetness && r.leaf_wetness)           leaf_wetness->publish_state(r.leaf_wetness);
        if (lightning_strikes && r.strikes)           lightning_strikes->publish_state(r.strikes);
        if (lightning_distance && r.lightning_dist_km > 0) lightning_distance->publish_state(r.lightning_dist_km);
        if (leak)                                  leak->publish_state(r.leak ? 1.0f : 0.0f);
    }

    static bool is_remote_indoor(const ecowitt_rf::Reading &r) {
        return r.family == Family::WH25 && r.model != 0;
    }

    void upload_reading(const ecowitt_rf::Reading &r) {
        if (!cfg_.upload_enabled)
            return;
        // Remote T/H/P stays separate on MQTT. Explicit mappings also send T/H
        // to reserved cloud extra channels; never overwrite the one indoor slot.
        if (is_remote_indoor(r) ||
            (r.family == Family::WH31 && remote_indoor_channels_.reserved(r.channel))) {
            for (size_t i = 0; i < extra_count_; ++i) {
                const Server &s = extra_servers_[i];
                if (s.enabled && s.proto == Protocol::MQTT)
                    mqtt_upload_reading(s, r, i);
            }
            Reading cloud_reading;
            if (remote_indoor_channels_.map(r, cloud_reading))
                upload_cloud_reading(cloud_reading);
            else if (r.family == Family::WH31)
                ESP_LOGW("ecowitt_rf", "WH31 channel %u reserved for remote indoor sensor; MQTT only", r.channel);
            return;
        }
        // Primary Ecowitt server (YAML / config entity, backward compatible).
        if (!upload.host.empty()) {
            std::string q = ecowitt_rf::reading_query(upload, r);
            int code = ecowitt_rf::http_request(true, upload.host, upload.port, upload.path, q);
            up_ok_ = (code == 200);
            up_code_ = code;
            up_last_ = (uint32_t)::time(nullptr);
        }
        upload_cloud_reading(r);
        // Additional servers (any protocol, any count up to 4).
        for (size_t i = 0; i < extra_count_; ++i) {
            Server &s = extra_servers_[i];
            if (!s.enabled)
                continue;
            if (s.proto == Protocol::MQTT) {
                mqtt_upload_reading(s, r, i);
                continue;
            }
            if (s.host.empty())
                continue;
            if (s.proto == Protocol::WUNDERGROUND) {
                // Wunderground's host/path are fixed constants (raw-socket HTTP, so the legacy
                // rtupdate endpoint, not the HTTPS one); only the station ID (key) varies.
                std::string q = ecowitt_rf::wunderground_query(s.key, r);
                extra_code_[i] = ecowitt_rf::http_request(false, "rtupdate.wunderground.com", 80,
                                                          "/weatherstation/updateweatherstation.php", q);
            } else {  // ECOWITT (or unknown) — POST like the primary
                UploadConfig tmp;
                tmp.host = s.host; tmp.port = s.port; tmp.path = s.path; tmp.passkey = s.key;
                std::string q = ecowitt_rf::reading_query(tmp, r);
                extra_code_[i] = ecowitt_rf::http_request(true, s.host, s.port, s.path, q);
            }
        }
    }

    void upload_cloud_reading(const ecowitt_rf::Reading &r) {
        // Ecowitt CLOUD (the OEM upload path) — the stock GW1100 posts here so the WS View /
        // Ecowitt app sees live data. Same PASSKEY + field set as the primary, plus dateutc
        // (the cloud rejects a stale timestamp). Fixed endpoint, gated by cfg_.cloud_enabled.
        if (cfg_.cloud_enabled) {
            std::string q = ecowitt_rf::reading_query(upload, r);
            time_t now = ::time(nullptr);
            struct tm *t = ::gmtime(&now);
            if (t) {
                char buf[32];
                std::snprintf(buf, sizeof(buf), "dateutc=%04d-%02d-%02d+%02d:%02d:%02d",
                              t->tm_year + 1900, t->tm_mon + 1, t->tm_mday, t->tm_hour, t->tm_min, t->tm_sec);
                if (!q.empty()) q += '&';
                q += buf;
            }
            cloud_code_ = ecowitt_rf::http_request(true, "cdnrtpdate.ecowitt.net", 80, "/data/report/", q);
        }
    }

    // Publish one decoded reading to MQTT (the "MQTT" upload protocol). The broker is the
    // global ESPHome mqtt: component (see YAML), so the Server's host/port are ignored;
    // `path` is the topic prefix (default "gw1100"), and the full topic is
    // "<prefix>/<FAM>[/sensor-ID][/chN]" (ID for remote indoor units). Payload has identity + every
    // non-zero measurement this family carries. extra_code_[slot]:
    //   1 = published, -4 = publish failed, -5 = broker not connected, -6 = MQTT not built.
    void mqtt_upload_reading(const Server &s, const ecowitt_rf::Reading &r, size_t slot) {
#ifdef USE_MQTT
        if (mqtt::global_mqtt_client == nullptr || !mqtt::global_mqtt_client->is_connected()) {
            extra_code_[slot] = -5;
            return;
        }
        std::string topic = s.path.empty() ? std::string("gw1100") : std::string(s.path);
        topic += "/";
        topic += family_tag(r.family);
        if (is_remote_indoor(r)) {
            char sensor_id[8];
            std::snprintf(sensor_id, sizeof(sensor_id), "/%06X", r.id & 0xFFFFFF);
            topic += sensor_id;
        }
        if (r.channel > 0) {
            char cbuf[10];
            std::snprintf(cbuf, sizeof(cbuf), "/ch%d", r.channel);
            topic += cbuf;
        }
        bool ok = mqtt::global_mqtt_client->publish_json(topic, [&r](JsonObject o) {
            o["fam"] = family_tag(r.family);
            char id[8];
            std::snprintf(id, sizeof(id), "%06X", r.id & 0xFFFFFF);
            o["id"] = id;
            o["ch"] = r.channel;
            o["rssi"] = r.rssi_dbm;
            o["batt"] = r.battery_low ? 0 : 1;
            o["ts"] = (uint32_t)::time(nullptr);
            if (r.has(F_TEMP)) o["temp_c"] = r.temp_c;
            if (r.has(F_HUM)) o["humidity"] = r.humidity;
            if (r.has(F_PRESS)) o["pressure_hpa"] = r.pressure_hpa;
            if (r.has(F_WIND)) o["wind_mps"] = r.wind_mps;
            if (r.has(F_GUST)) o["gust_mps"] = r.gust_mps;
            if (r.has(F_WDIR)) o["wind_dir_deg"] = r.wind_dir_deg;
            if (is_rain_family(r.family)) {
                o["rain_mm"] = r.rain_mm;
                o["rain_rate_mmh"] = r.rain_rate_mmh;
                o["rain_event_mm"] = r.rain_event_mm;
                o["rain_hour_mm"] = r.rain_hour_mm;
                o["rain_day_mm"] = r.rain_day_mm;
                o["rain_week_mm"] = r.rain_week_mm;
                o["rain_month_mm"] = r.rain_month_mm;
                o["rain_year_mm"] = r.rain_year_mm;
                o["last24h_rain_mm"] = r.last24h_rain_mm;
            }
            if (r.has(F_UV)) o["uv"] = r.uv;
            if (r.has(F_LIGHT)) o["solar_wm2"] = r.light_wm2;
            if (r.moisture_pct) o["soil_moist_pct"] = r.moisture_pct;
            if (r.soil_temp_c != 0) o["soil_temp_c"] = r.soil_temp_c;
            if (r.soil_ec_uscm > 0) o["soil_ec_uscm"] = r.soil_ec_uscm;
            if (r.pm25_ugm3) o["pm25_ugm3"] = r.pm25_ugm3;
            if (r.pm10_ugm3) o["pm10_ugm3"] = r.pm10_ugm3;
            if (r.co2_ppm) o["co2_ppm"] = r.co2_ppm;
            if (r.leaf_wetness) o["leaf_wetness"] = r.leaf_wetness;
            if (r.strikes) o["strikes"] = r.strikes;
            if (r.lightning_dist_km > 0) o["lightning_dist_km"] = r.lightning_dist_km;
            if (r.leak) o["leak"] = 1;
        });
        extra_code_[slot] = ok ? 1 : -4;
#else
        (void)s; (void)r;
        extra_code_[slot] = -6;
#endif
    }

    // ---- persistence (ESPHome flash preferences, 512 B sector) ----
    // Config is stored as ONE blob (16 individual prefs would each carry a checksum word and
    // blow the sector). The upload-server list is a second compact blob. Per-sensor overrides
    // (name/channel/enable) are RAM-only and re-learned on boot — no flash room left.
    //
    // The preference objects are allocated ONCE (setup_prefs) and reused: ESP8266's
    // make_preference() allocates a NEW flash offset on every call (no hash cache), so calling
    // it in each load/save would read/write different offsets and silently drop data.
    void setup_prefs() {
        cfg_pref_ = global_preferences->make_preference<uint8_t[sizeof(DeviceConfig)]>(PREF_CFG, true);
        servers_pref_ = global_preferences->make_preference<uint8_t[SERVERS_BLOB]>(PREF_SERVERS, true);
        rain_pref_ = global_preferences->make_preference<uint8_t[sizeof(RainPersist)]>(PREF_RAIN, true);
        disabled_pref_ = global_preferences->make_preference<uint8_t[sizeof(DisabledList)]>(PREF_DISABLED, true);
        // ESPHome2026.7.4 reserves normal preference offsets0..77; the next
        // region is eboot's scratch space. Never let this checkpoint spill there.
        constexpr size_t rain_words = (sizeof(RainCheckpoint) + 3) / 4 + 1;
        if (global_preferences->current_offset + rain_words <= 78)
            rain_rtc_pref_ = global_preferences->make_preference<RainCheckpoint>(PREF_RAIN_RTC, false);
        else
            ESP_LOGE("ecowitt_rf", "No safe RTC space for rain checkpoint");
        rain_restore_pending_ = rain_rtc_pref_.load(&rain_pending_) && rain_pending_.valid();
    }

    void load_cfg() {
        uint8_t blob[sizeof(DeviceConfig)];
        if (cfg_pref_.load(&blob))
            memcpy(&cfg_, blob, sizeof(DeviceConfig));
    }

    void save_cfg() {
        uint8_t blob[sizeof(DeviceConfig)];
        memcpy(blob, &cfg_, sizeof(DeviceConfig));
        cfg_pref_.save(&blob);
        global_preferences->sync();
    }

    void load_servers() {
        uint8_t blob[SERVERS_BLOB];
        if (servers_pref_.load(&blob)) {
            extra_count_ = (blob[0] <= SERVER_MAX) ? blob[0] : 0;
            for (uint8_t i = 0; i < extra_count_; i++) {
                PersistedServer ps;
                memcpy(&ps, blob + 1 + (size_t)i * sizeof(PersistedServer), sizeof(ps));
                Server &s = extra_servers_[i];
                s.proto = (ps.proto == (uint8_t)Protocol::WUNDERGROUND) ? Protocol::WUNDERGROUND
                         : (ps.proto == (uint8_t)Protocol::MQTT) ? Protocol::MQTT : Protocol::ECOWITT;
                s.enabled = ps.enabled != 0;
                s.port = ps.port;
                s.host.assign(ps.host);
                s.path.assign(ps.path);
                s.key.assign(ps.key);
            }
        }
    }

    void save_servers() {
        uint8_t blob[SERVERS_BLOB];
        memset(blob, 0, sizeof(blob));
        blob[0] = extra_count_;
        for (uint8_t i = 0; i < extra_count_; i++) {
            PersistedServer ps;
            memset(&ps, 0, sizeof(ps));
            ps.proto = (uint8_t)extra_servers_[i].proto;
            ps.enabled = extra_servers_[i].enabled ? 1 : 0;
            ps.port = extra_servers_[i].port;
            strncpy(ps.host, extra_servers_[i].host.c_str(), sizeof(ps.host) - 1);
            strncpy(ps.path, extra_servers_[i].path.c_str(), sizeof(ps.path) - 1);
            strncpy(ps.key, extra_servers_[i].key.c_str(), sizeof(ps.key) - 1);
            memcpy(blob + 1 + (size_t)i * sizeof(PersistedServer), &ps, sizeof(ps));
        }
        servers_pref_.save(&blob);
        global_preferences->sync();
    }

    void checkpoint_rain(uint32_t now_unix) {
        if (rain_.count() == 0 || now_unix < 1704067200UL) return;
        const auto &b = rain_.buckets()[0];
        RainCheckpoint saved;
        saved.key = b.key; saved.unix_time = now_unix;
        saved.rolling = b.rolling.state(); saved.recent = b.recent.state();
        rain_rtc_saved_ = rain_rtc_pref_.save(&saved);
    }

    // Rain-aggregate state (the first bucket). Persisted so monthly/yearly totals survive
    // reboot — the OEM's parity gap. Loaded in setup(), saved when the aggregator is dirty.
    void load_rain() {
        RainPersist rp;
        if (rain_pref_.load(&rp) && rp.last_counter >= 0.0f) {
            rain_.set_count(1);
            RainAggregator::Bucket &b = rain_.buckets()[0];
            b.key = rp.key;
            b.last_counter = rp.last_counter;
            b.event_mm = rp.event_mm; b.day_mm = rp.day_mm; b.week_mm = rp.week_mm;
            b.month_mm = rp.month_mm; b.year_mm = rp.year_mm;
            b.restore_calendar(rp.mark_doy, rp.mark_month, rp.mark_year);
        }
    }

    void save_rain() {
        if (rain_.count() == 0)
            return;
        RainAggregator::Bucket &b = rain_.buckets()[0];
        RainPersist rp;
        rp.key = b.key;
        rp.last_counter = b.last_counter;
        rp.event_mm = b.event_mm; rp.day_mm = b.day_mm; rp.week_mm = b.week_mm;
        rp.month_mm = b.month_mm; rp.year_mm = b.year_mm;
        rp.mark_doy = b.mark_doy; rp.mark_month = b.mark_month; rp.mark_year = b.mark_year;
        rain_pref_.save(&rp);
        global_preferences->sync();
    }

    // Per-sensor disable list (the only user-editable override that must survive reboot).
    void load_disabled() {
        DisabledList dl;
        if (disabled_pref_.load(&dl) && dl.count <= DISABLED_MAX)
            disabled_ = dl;
    }

    bool is_disabled(uint8_t family, uint32_t id) const {
        for (uint8_t i = 0; i < disabled_.count; i++) {
            uint32_t sid = ((uint32_t)disabled_.s[i].id0 << 16) | ((uint32_t)disabled_.s[i].id1 << 8) | disabled_.s[i].id2;
            if (disabled_.s[i].family == family && sid == id)
                return true;
        }
        return false;
    }

    void set_disabled(uint8_t family, uint32_t id, bool dis) {
        // Remove any existing entry, shifting left.
        uint8_t w = 0;
        for (uint8_t i = 0; i < disabled_.count; i++) {
            uint32_t sid = ((uint32_t)disabled_.s[i].id0 << 16) | ((uint32_t)disabled_.s[i].id1 << 8) | disabled_.s[i].id2;
            if (!(disabled_.s[i].family == family && sid == id))
                disabled_.s[w++] = disabled_.s[i];
        }
        disabled_.count = w;
        // Add back if disabling.
        if (dis && disabled_.count < DISABLED_MAX) {
            disabled_.s[disabled_.count].family = family;
            disabled_.s[disabled_.count].id0 = (id >> 16) & 0xFF;
            disabled_.s[disabled_.count].id1 = (id >> 8) & 0xFF;
            disabled_.s[disabled_.count].id2 = id & 0xFF;
            disabled_.count++;
        }
        disabled_pref_.save(&disabled_);
        global_preferences->sync();
    }

    void save_sensors() {}  // kept for the setter call sites; enable is handled by set_disabled

    // Push persisted config into the live upload/poll state.
    void apply_cfg() {
        if (cfg_.host[0]) upload.host = cfg_.host;
        if (cfg_.path[0]) upload.path = cfg_.path;
        upload.port = cfg_.port;
        if (cfg_.passkey[0]) upload.passkey = cfg_.passkey;
        if (cfg_.band) device_type = cfg_.band;
        set_update_interval((uint32_t)cfg_.interval_s * 1000);
        if (time_clock && cfg_.timezone[0]) {
            const char *posix = iana_to_posix(cfg_.timezone);
            if (posix)
                time_clock->set_timezone(posix);
        }
    }

    // ---- per-sensor config (flash-persisted: enable + channel per sensor id) ----
    SensorSlot sensor_slots_[SENSOR_MAX];
    uint8_t sensor_count_ = 0;

    int16_t find_sensor(uint8_t family, uint32_t id) const {
        for (uint8_t i = 0; i < sensor_count_; i++) {
            uint32_t sid = ((uint32_t)sensor_slots_[i].id0 << 16) | ((uint32_t)sensor_slots_[i].id1 << 8) | sensor_slots_[i].id2;
            if (sensor_slots_[i].family == family && sid == id)
                return i;
        }
        return -1;
    }

    // Ensure the sensor has a slot; apply its persisted channel; return whether enabled.
    bool resolve_sensor(Reading &r) {
        int16_t idx = find_sensor((uint8_t)r.family, r.id);
        if (idx >= 0) {
            r.channel = sensor_slots_[idx].channel;
            return sensor_slots_[idx].enabled != 0;
        }
        if (sensor_count_ < SENSOR_MAX) {
            SensorSlot &s = sensor_slots_[sensor_count_++];
            s.family = (uint8_t)r.family;
            s.id0 = (r.id >> 16) & 0xFF;
            s.id1 = (r.id >> 8) & 0xFF;
            s.id2 = r.id & 0xFF;
            s.channel = r.channel;
            s.enabled = is_disabled((uint8_t)r.family, r.id) ? 0 : 1;  // apply persisted disable
            save_sensors();
        }
        return true;
    }

    void set_sensor(uint8_t family, uint32_t id, uint8_t channel, bool enabled) {
        int16_t idx = find_sensor(family, id);
        if (idx < 0) {
            if (sensor_count_ >= SENSOR_MAX)
                return;
            idx = sensor_count_++;
            sensor_slots_[idx].family = family;
            sensor_slots_[idx].id0 = (id >> 16) & 0xFF;
            sensor_slots_[idx].id1 = (id >> 8) & 0xFF;
            sensor_slots_[idx].id2 = id & 0xFF;
        }
        sensor_slots_[idx].channel = channel;
        sensor_slots_[idx].enabled = enabled ? 1 : 0;
        set_disabled(family, id, !enabled);
        save_sensors();
    }
    void set_sensor_name(uint8_t family, uint32_t id, const char *name) {
        int16_t idx = find_sensor(family, id);
        if (idx < 0)
            return;
        strncpy(sensor_slots_[idx].name, name, sizeof(sensor_slots_[idx].name) - 1);
        sensor_slots_[idx].name[sizeof(sensor_slots_[idx].name) - 1] = 0;
        save_sensors();
    }

    // ---- learned-sensor roster (RAM-only live state, not persisted) ----
    struct RosterEntry {
        Family family = Family::Unknown;
        uint32_t id = 0;
        uint8_t channel = 0;
        int8_t rssi = 0;
        uint32_t last_ms = 0;
        bool battery_low = false;
        UiState v;             // this sensor's own latest readings (per-sensor, not shared)
    };
    RosterEntry roster_[32];
    size_t roster_count_ = 0;

    // Returns true if the sensor was already in the roster AND was heard RECENTLY. A
    // first-sighting false is used to skip the upload, so a one-off stray decode (noise that
    // passes the checksum) can't mint a phantom entity in HA.
    //
    // ★ The recency window matters. The WH25 ID is only 8 bits wide, so with ~20 stale
    // phantom IDs resident a fresh noise frame had roughly a 1-in-12 chance of colliding
    // with one of them and being waved through as "seen before" — which is exactly how
    // garbage temp/humidity/pressure reached HA about once a day. A genuine sensor reports
    // at most ~70 s apart, so anything that has been silent for an hour is not a repeat
    // sighting; it is a new frame that happens to share an ID.
    static constexpr uint32_t ROSTER_SEEN_WINDOW_MS = 3600000UL;  // 1 h

    bool record_roster(const Reading &r) {
        for (size_t i = 0; i < roster_count_; i++) {
            if (roster_[i].family == r.family && roster_[i].id == r.id) {
                uint32_t prev_ms = roster_[i].last_ms;
                roster_[i].channel = r.channel;
                roster_[i].rssi = r.rssi_dbm;
                roster_[i].last_ms = millis();
                roster_[i].battery_low = r.battery_low;
                map_reading(r, roster_[i].v);
                const uint32_t window = is_remote_indoor(r) ? 180000UL : ROSTER_SEEN_WINDOW_MS;
                return (millis() - prev_ms) <= window;
            }
        }
        if (roster_count_ < sizeof(roster_) / sizeof(roster_[0])) {
            RosterEntry &e = roster_[roster_count_++];
            e.family = r.family; e.id = r.id; e.channel = r.channel;
            e.rssi = r.rssi_dbm; e.last_ms = millis(); e.battery_low = r.battery_low;
            map_reading(r, e.v);
        }
        return false;
    }

    // The gateway's own built-in indoor T/H/P (I2C, not RF) gets its own roster card so it
    // appears alongside the RF sensors. family=Builtin, id=0 (no RF id).
    void record_builtin(float temp_c, float humidity, float pressure_hpa) {
        for (size_t i = 0; i < roster_count_; i++) {
            if (roster_[i].family == Family::Builtin) {
                roster_[i].last_ms = millis();
                if (temp_c != 0) roster_[i].v.set(UiState::T_IN, temp_c);
                if (humidity) roster_[i].v.set(UiState::H_IN, humidity);
                if (pressure_hpa > 0) roster_[i].v.set(UiState::P, pressure_hpa);
                return;
            }
        }
        if (roster_count_ < sizeof(roster_) / sizeof(roster_[0])) {
            RosterEntry &e = roster_[roster_count_++];
            e.family = Family::Builtin; e.id = 0; e.channel = 0;
            e.last_ms = millis();
            if (temp_c != 0) e.v.set(UiState::T_IN, temp_c);
            if (humidity) e.v.set(UiState::H_IN, humidity);
            if (pressure_hpa > 0) e.v.set(UiState::P, pressure_hpa);
        }
    }

    void publish_roster() {
        if (!roster)
            return;
        std::string s;
        char line[96];
        uint32_t now = millis();
        for (size_t i = 0; i < roster_count_; i++) {
            uint32_t ago_s = (now - roster_[i].last_ms) / 1000;
            std::snprintf(line, sizeof(line), "%s id=0x%06X ch=%u rssi=%ddBm %us ago\n",
                          family_name(roster_[i].family), roster_[i].id,
                          roster_[i].channel, roster_[i].rssi, (unsigned)ago_s);
            s += line;
        }
        if (roster_count_ == 0)
            s = "no sensors heard yet";
        roster->publish_state(s);
    }

    void publish_device_info() {
        if (!device_info)
            return;
        char buf[160];
        std::snprintf(buf, sizeof(buf), "MAC=%s IP=%s SSID=%s RSSI=%ddBm heap=%uk",
                      WiFi.macAddress().c_str(), WiFi.localIP().toString().c_str(),
                      WiFi.SSID().c_str(), WiFi.RSSI(), (unsigned)(ESP.getFreeHeap() / 1024));
        device_info->publish_state(buf);
    }
};

} // namespace ecowitt_rf
} // namespace esphome
