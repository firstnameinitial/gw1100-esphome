// SPDX-License-Identifier: GPL-3.0-only
// Ecowitt "Customized server" upload — the GW1000/WH2650 HTTP protocol that Home
// Assistant's `ecowitt` integration (via aioecowitt) consumes. Sends decoded sensor
// readings to a user-configured server as an HTTP GET with query params.
//
// Field names are the authoritative aioecowitt keys (aioecowitt), so the
// receiver parses every family. The Ecowitt CLOUD upload is intentionally NOT implemented
// — custom-server only, matching the GW1100 "Customized" slot.
//
// ENTITY-CONTINUITY (critical): HA's ecowitt integration keys every entity as
//   unique_id = "<PASSKEY>-<fieldkey>"
// The stock GW1100 sends PASSKEY = MD5(WiFi.macAddress()) UPPERCASED and METRIC field
// names. To take over an existing station's entities (and their history) without forking
// a new set, this firmware must send the SAME PASSKEY and SAME metric keys. default_passkey()
// recomputes that MD5 from the MAC at runtime; the YAML `passkey` overrides it only if set.
#pragma once
#include <cstdint>
#include <string>
#include <cstdio>
#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <MD5Builder.h>

#include "fineoffset.h"

namespace esphome {
namespace ecowitt_rf {

struct UploadConfig {
    std::string host;
    uint16_t port = 8123;
    std::string path = "/api/webhook/";   // webhook id appended by the YAML path value
    std::string passkey;                  // empty -> use default_passkey() (MD5 of MAC)
    std::string stationtype = "GW1100A";
    std::string freq = "868M";
};

// Upload transport protocol for a target server.
enum class Protocol : uint8_t { ECOWITT = 0, WUNDERGROUND = 1, MQTT = 2 };

// One upload target. Ecowitt uses host/port/path + key=PASSKEY; Wunderground uses
// host/port/path + key=station ID. MQTT is a global toggle (broker from ESPHome's mqtt).
struct Server {
    Protocol proto = Protocol::ECOWITT;
    std::string host;
    uint16_t port = 80;
    std::string path;
    std::string key;
    bool enabled = false;
};

// Generic raw-socket HTTP. use_body=true -> POST the query as a form body (aioecowitt);
// use_body=false -> GET with the query in the URL (Wunderground).
// Returns the HTTP status code, or -2 (not configured/net down), -1 (connect failed), -3 (no response).
inline int http_request(bool use_body, const std::string &host, uint16_t port,
                        const std::string &path, const std::string &query) {
    if (host.empty() || WiFi.status() != WL_CONNECTED) return -2;
    WiFiClient client;
    // 3000 ms was long enough for one slow target to blow the component's 2550 ms
    // loop-time budget on its own. The receivers here are on the LAN.
    client.setTimeout(1200);
    // connect(host, port) resolves both IP literals and hostnames (DNS) — required for
    // Wunderground's "rtupdate.wunderground.com", which is not an IP.
    if (!client.connect(host.c_str(), port)) return -1;
    std::string req = std::string(use_body ? "POST " : "GET ") + path +
                      (use_body ? std::string(" HTTP/1.0") : ("?" + query + " HTTP/1.0"));
    req += "\r\nHost: " + host + "\r\n";
    if (use_body) {
        req += "Content-Type: application/x-www-form-urlencoded\r\n";
        req += "Content-Length: " + std::to_string(query.size()) + "\r\n";
    }
    req += "Connection: close\r\n\r\n";
    if (use_body) req += query;
    client.write(req.c_str(), req.size());
    client.flush();
    String status = client.readStringUntil('\n');
    client.stop();
    int sp = status.indexOf(' ');
    return sp < 0 ? -3 : status.substring(sp + 1, sp + 4).toInt();
}

// PASSKEY the stock GW1100 uses: uppercase MD5 of the MAC (colon-separated, uppercase),
// e.g. MD5("AA:BB:CC:DD:EE:FF") = "<MD5-of-MAC>".
inline std::string default_passkey() {
    MD5Builder md5;
    md5.begin();
    md5.add(WiFi.macAddress().c_str());   // "AA:BB:CC:DD:EE:FF"
    md5.calculate();
    std::string s = md5.toString().c_str();  // lowercase hex
    for (char &c : s) c = toupper((unsigned char)c);
    return s;
}

inline void add_field(std::string &url, const char *name, float value) {
    if (!url.empty()) url += '&';
    url += name; url += '=';
    char buf[24];
    std::snprintf(buf, sizeof(buf), "%.1f", value);
    url += buf;
}
inline void add_field_prec(std::string &url, const char *name, float value, int prec) {
    if (!url.empty()) url += '&';
    url += name; url += '=';
    char buf[24];
    std::snprintf(buf, sizeof(buf), "%.*f", prec, value);
    url += buf;
}
inline void add_field_i(std::string &url, const char *name, int value) {
    if (!url.empty()) url += '&';
    url += name; url += '=';
    char buf[16];
    std::snprintf(buf, sizeof(buf), "%d", value);
    url += buf;
}
inline void add_field(std::string &url, const char *name, const std::string &value) {
    if (!url.empty()) url += '&';
    url += name; url += '=';
    url += value;
}

// Vapour pressure deficit, in inHg (the aioecowitt `vpd` key is VPD_INHG). Magnus formula.
inline float vpd_inhg(float temp_c, int humidity) {
    if (temp_c <= -50.0f || humidity <= 0 || humidity > 100)
        return 0.0f;
    float es = 0.6108f * expf(17.27f * temp_c / (temp_c + 237.3f));  // kPa
    float vpd = es * (1.0f - humidity / 100.0f);
    return (vpd < 0 ? 0.0f : vpd) * 0.2953f;  // kPa -> inHg
}

// Wunderground "PWS upload" field names (imperial, per the WU protocol).
inline std::string wunderground_query(const std::string &station_id, const Reading &r) {
    std::string q;
    add_field(q, "ID", station_id);
    add_field(q, "PASSWORD", "password");
    add_field(q, "dateutc", "now");
    add_field(q, "action", "updateraw");
    if (r.has(F_TEMP)) add_field(q, "tempf", r.temp_c * 9.0f / 5.0f + 32.0f);
    if (r.has(F_HUM)) add_field_i(q, "humidity", r.humidity);
    if (r.has(F_WDIR)) add_field_i(q, "winddir", r.wind_dir_deg);
    if (r.has(F_WIND)) add_field(q, "windspeedmph", r.wind_mps * 2.23694f);
    if (r.has(F_GUST)) add_field(q, "windgustmph", r.gust_mps * 2.23694f);
    if (r.rain_mm > 0) add_field(q, "rainin", r.rain_mm / 25.4f);
    if (is_rain_family(r.family)) add_field(q, "rainratein", r.rain_rate_mmh / 25.4f);
    if (r.has(F_LIGHT)) add_field(q, "solarradiation", r.light_wm2);
    if (r.has(F_UV)) add_field(q, "UV", (float)r.uv);
    if (r.has(F_PRESS)) add_field(q, "baromin", r.pressure_hpa * 0.02953f);
    return q;
}

// Send one HTTP POST to the custom server. Raw-socket HTTP (no Arduino HTTPClient) to
// keep the dependency surface small. HA's aioecowitt webhook accepts POST (returns 200);
// it rejects GET with 405, so we POST the query as an x-www-form-urlencoded body.
// Returns the HTTP status code, or a negative diagnostic:
//   -2 = not configured / no network, -1 = connect failed, -3 = no response line.
inline int upload(const UploadConfig &cfg, const std::string &query) {
    if (cfg.host.empty() || WiFi.status() != WL_CONNECTED) return -2;
    WiFiClient client;
    // 3000 ms was long enough for one slow target to blow the component's 2550 ms
    // loop-time budget on its own. The receivers here are on the LAN.
    client.setTimeout(1200);
    if (!client.connect(cfg.host.c_str(), cfg.port)) return -1;  // IP or hostname (DNS)
    std::string req = "POST " + cfg.path + " HTTP/1.0\r\n";
    req += "Host: " + cfg.host + "\r\n";
    req += "Content-Type: application/x-www-form-urlencoded\r\n";
    req += "Content-Length: " + std::to_string(query.size()) + "\r\n";
    req += "Connection: close\r\n\r\n";
    req += query;
    client.write(req.c_str(), req.size());
    client.flush();
    String status = client.readStringUntil('\n');  // "HTTP/1.0 200 OK"
    client.stop();
    int sp = status.indexOf(' ');
    if (sp < 0) return -3;
    return status.substring(sp + 1, sp + 4).toInt();  // the 3-digit code
}

// Common station fields (auth + identity), METRIC.
inline std::string station_fields(const UploadConfig &cfg) {
    std::string q;
    std::string pk = cfg.passkey.empty() ? default_passkey() : cfg.passkey;
    add_field(q, "PASSKEY", pk);
    add_field(q, "stationtype", cfg.stationtype);
    add_field(q, "freq", cfg.freq);
    add_field(q, "model", cfg.stationtype);
    return q;
}

// Build the query string for one decoded reading (METRIC field names), matching the
// aioecowitt key set so HA maps onto the existing station's entities.
inline std::string reading_query(const UploadConfig &cfg, const Reading &r) {
    std::string q = station_fields(cfg);

    switch (r.family) {
        case Family::WH25:   // indoor T/H/P (WH32/WN32B) — metric
            if (r.has(F_TEMP)) {
                add_field(q, "tempinc", r.temp_c);
                add_field(q, "tempinf", r.temp_c * 9.0f / 5.0f + 32.0f);  // the Ecowitt app reads the imperial field for indoor
            }
            if (r.has(F_HUM)) add_field_i(q, "humidityin", r.humidity);
            if (r.has(F_PRESS)) {
                add_field(q, "baromrelhpa", r.pressure_hpa);
                add_field(q, "baromabshpa", r.pressure_hpa);
                // aioecowitt's calc.py OVERWRITES baromrelhpa with
                // round(baromrelin * 33.86, 1). Sending baromrelin at the OEM-style "%.1f"
                // (0.1 inHg = 3.4 hPa) therefore quantised the pressure to 3.4 hPa steps and
                // froze the HA entity (e.g. 1016.5 -> 30.0 inHg -> 1015.8 for hours). Send it
                // at the exact inverse of the library's 33.86 factor and 0.001 inHg precision
                // so the round-trip is lossless and the entity tracks the metric value.
                add_field_prec(q, "baromrelin", r.pressure_hpa / 33.86f, 3);
                add_field_prec(q, "baromabsin", r.pressure_hpa / 33.86f, 3);
            }
            if (r.battery_low) add_field_i(q, "wh25batt", 0);
            break;

        case Family::WH31: { // outdoor multi-channel T/H (8 ch)
            char k[16];
            if (r.has(F_TEMP)) {
                std::snprintf(k, sizeof(k), "temp%dc", r.channel);
                add_field(q, k, r.temp_c);
                std::snprintf(k, sizeof(k), "temp%df", r.channel);
                add_field(q, k, r.temp_c * 9.0f / 5.0f + 32.0f);
            }
            if (r.has(F_HUM)) {
                std::snprintf(k, sizeof(k), "humidity%d", r.channel);
                add_field_i(q, k, r.humidity);
            }
            std::snprintf(k, sizeof(k), "batt%d", r.channel);
            add_field_i(q, k, r.battery_low ? 1 : 0);  // Ecowitt binary battery: 1 = low
            break;
        }

        case Family::WH51: { // soil moisture (16 ch)
            char k[16];
            std::snprintf(k, sizeof(k), "soilmoisture%d", r.channel);
            add_field_i(q, k, r.moisture_pct);
            std::snprintf(k, sizeof(k), "soilbatt%d", r.channel);
            add_field_i(q, k, r.battery_low ? 0 : 1);
            break;
        }

        case Family::WH24:
        case Family::WH65:
        case Family::WS69: { // outdoor weather array (tipping-bucket rain)
            if (r.has(F_WDIR)) add_field_i(q, "winddir", r.wind_dir_deg);
            if (r.has(F_WIND)) {
                add_field(q, "windspeedkmh", r.wind_mps * 3.6f);
                add_field(q, "windspeedmph", r.wind_mps * 2.23694f);   // imperial — calc.py derives windchill/feels-like from these
            }
            if (r.has(F_GUST)) {
                add_field(q, "windgustkmh", r.gust_mps * 3.6f);
                add_field(q, "windgustmph", r.gust_mps * 2.23694f);
            }
            add_field(q, "maxdailygust", r.daily_gust_mps * 2.23694f);
            add_field_i(q, "winddir_avg10m", r.wind_dir_avg_deg);
            add_field(q, "rainratemm", r.rain_rate_mmh);
            add_field(q, "eventrainmm", r.rain_event_mm);
            add_field(q, "hourlyrainmm", r.rain_hour_mm);
            add_field(q, "dailyrainmm", r.rain_day_mm);
            add_field(q, "weeklyrainmm", r.rain_week_mm);
            add_field(q, "monthlyrainmm", r.rain_month_mm);
            add_field(q, "yearlyrainmm", r.rain_year_mm);
            add_field(q, "totalrainmm", r.rain_mm);
            if (r.has(F_LIGHT)) add_field(q, "solarradiation", r.light_wm2);
            if (r.has(F_UV)) add_field_i(q, "uv", r.uv);
            if (r.has(F_TEMP)) {
                add_field(q, "tempc", r.temp_c);
                add_field(q, "tempf", r.temp_c * 9.0f / 5.0f + 32.0f);  // aioecowitt metric-temp bug workaround
            }
            if (r.has(F_HUM)) add_field_i(q, "humidity", r.humidity);
            if (r.has(F_TEMP) && r.has(F_HUM)) add_field(q, "vpd", vpd_inhg(r.temp_c, r.humidity));
            // NOTE: the WS69 pressure extension is deliberately NOT uploaded — see the
            // barometer-ownership comment in the WS68/80/85/90 case below.
            add_field_i(q, "wh65batt", r.battery_low ? 0 : 1);
            break;
        }

        case Family::WH0290: // PM2.5/PM10 (WH41 family)
            add_field_i(q, "pm25_ch1", r.pm25_ugm3);
            add_field_i(q, "pm25batt1", r.battery_low ? 0 : 1);
            break;

        case Family::WH43:
            add_field_i(q, "pm25_ch1", r.pm25_ugm3);
            add_field_i(q, "pm25batt1", r.battery_low ? 0 : 1);
            break;

        case Family::WH45:
            add_field_i(q, "pm25_ch1", r.pm25_ugm3);
            add_field_i(q, "co2", r.co2_ppm);
            add_field_i(q, "co2_batt", r.battery_low ? 0 : 1);
            break;

        case Family::WH46:
            add_field_i(q, "pm25_ch1", r.pm25_ugm3);
            add_field_i(q, "co2", r.co2_ppm);
            add_field_i(q, "co2_batt", r.battery_low ? 0 : 1);
            break;

        case Family::WN34:   // soil temp (8 ch)
        case Family::WN38: {
            char k[16];
            std::snprintf(k, sizeof(k), "tf_ch%d", r.channel);
            add_field(q, k, r.soil_temp_c);
            std::snprintf(k, sizeof(k), "tf_batt%d", r.channel);
            add_field_i(q, k, r.battery_low ? 0 : 1);
            break;
        }

        case Family::WH40:   // rain gauge
        case Family::WN20:
            add_field(q, "eventrainmm", r.rain_event_mm);
            add_field(q, "hourlyrainmm", r.rain_hour_mm);
            add_field(q, "dailyrainmm", r.rain_day_mm);
            add_field(q, "weeklyrainmm", r.rain_week_mm);
            add_field(q, "monthlyrainmm", r.rain_month_mm);
            add_field(q, "yearlyrainmm", r.rain_year_mm);
            add_field(q, "totalrainmm", r.rain_mm);
            add_field(q, (r.family == Family::WN20) ? "wn20batt" : "wh40batt", r.battery_v);  // battery V
            break;

        case Family::WH55: { // leak (4 ch)
            char k[16];
            std::snprintf(k, sizeof(k), "leak_ch%d", r.channel);
            add_field_i(q, k, r.leak ? 1 : 0);
            std::snprintf(k, sizeof(k), "leakbatt%d", r.channel);
            add_field_i(q, k, r.battery_low ? 0 : 1);
            break;
        }

        case Family::WH57:   // lightning
            add_field_i(q, "lightning", r.lightning_dist_km > 0 ? 1 : 0);
            add_field(q, "lightning_mi", r.lightning_dist_km * 0.621371f);
            add_field_i(q, "lightning_num", (int)r.strikes);
            add_field_i(q, "wh57batt", r.battery_low ? 0 : 1);
            break;

        case Family::WS68:
        case Family::WS80:
        case Family::WS85:
        case Family::WS90: { // ultrasonic arrays (WH90 = piezo rain)
            if (r.has(F_WDIR)) add_field_i(q, "winddir", r.wind_dir_deg);
            if (r.has(F_WIND)) {
                add_field(q, "windspeedkmh", r.wind_mps * 3.6f);
                add_field(q, "windspeedmph", r.wind_mps * 2.23694f);   // imperial — calc.py derives windchill/feels-like from these
            }
            if (r.has(F_GUST)) {
                add_field(q, "windgustkmh", r.gust_mps * 3.6f);
                add_field(q, "windgustmph", r.gust_mps * 2.23694f);
            }
            add_field(q, "maxdailygust", r.daily_gust_mps * 2.23694f);
            add_field_i(q, "winddir_avg10m", r.wind_dir_avg_deg);
            if (r.has(F_LIGHT)) add_field(q, "solarradiation", r.light_wm2);
            if (r.has(F_UV)) add_field_i(q, "uv", r.uv);
            // ★ BAROMETER OWNERSHIP. The WS90 carries its own barometer, but so does the
            // gateway (HP203B, uploaded every update() as the WH25 slot) — and both used to
            // write baromrelhpa under the SAME PASSKEY, so HA's single relative-pressure
            // entity was driven alternately by two sensors at two cadences (16 s vs ~8.8 s)
            // reading a few hPa apart. The gateway barometer is the station barometer, as on
            // the OEM console, so the array's pressure is intentionally NOT uploaded.
            if (r.has(F_TEMP)) {
                add_field(q, "tempc", r.temp_c);
                add_field(q, "tempf", r.temp_c * 9.0f / 5.0f + 32.0f);  // aioecowitt metric-temp bug workaround
            }
            if (r.has(F_HUM)) add_field_i(q, "humidity", r.humidity);
            if (r.has(F_TEMP) && r.has(F_HUM)) add_field(q, "vpd", vpd_inhg(r.temp_c, r.humidity));
            // piezo rain (WS85/WS90): running hourly/daily/weekly/monthly/yearly totals
            // + event rain + rate (all derived by RainAggregator from the cumulative counter).
            add_field(q, "erain_piezomm", r.rain_event_mm);
            add_field(q, "hrain_piezomm", r.rain_hour_mm);
            add_field(q, "drain_piezomm", r.rain_day_mm);
            add_field(q, "wrain_piezomm", r.rain_week_mm);
            add_field(q, "mrain_piezomm", r.rain_month_mm);
            add_field(q, "yrain_piezomm", r.rain_year_mm);
            add_field(q, "last24hrain_piezomm", r.last24h_rain_mm);
            add_field(q, "rrain_piezomm", r.rain_rate_mmh);
            add_field(q, "totalrainmm", r.rain_mm);   // lifetime counter — the WS90's one historic value
            add_field_i(q, "srain_piezo", r.rain_state ? 1 : 0);  // rain-start flag (RAIN_STATE)
            // battery + supercap are VOLTAGES (aioecowitt BATTERY_VOLTAGE / VOLTAGE), not 0/1.
            if (r.family == Family::WS68) {
                add_field_i(q, "wh68batt", r.battery_low ? 0 : 1);
            } else if (r.family == Family::WS80) {
                add_field(q, "wh80batt", r.battery_v);
            } else if (r.family == Family::WS85) {
                add_field(q, "wh85batt", r.battery_v);
                if (r.has(F_CAPV)) add_field(q, "ws85cap_volt", r.cap_v);
            } else {  // WS90
                add_field(q, "wh90batt", r.battery_v);
                if (r.has(F_CAPV)) add_field(q, "ws90cap_volt", r.cap_v);
            }
            break;
        }

        case Family::WH52: { // soil moisture/temp/EC
            char k[16];
            std::snprintf(k, sizeof(k), "soilmoisture%d", r.channel);
            add_field_i(q, k, r.moisture_pct);
            std::snprintf(k, sizeof(k), "soil_ec_temp%d", r.channel);
            add_field(q, k, r.soil_temp_c);
            std::snprintf(k, sizeof(k), "soil_ec%d", r.channel);
            add_field(q, k, r.soil_ec_uscm);
            std::snprintf(k, sizeof(k), "soil_ec_batt%d", r.channel);
            add_field_i(q, k, r.battery_low ? 0 : 1);
            break;
        }

        default:
            break;
    }
    return q;
}

// Convenience: WH25/WH32 indoor reading (kept for backward compatibility).
inline std::string wh25_query(const UploadConfig &cfg, const Reading &r) {
    return reading_query(cfg, r);
}

}  // namespace ecowitt_rf
}  // namespace esphome
