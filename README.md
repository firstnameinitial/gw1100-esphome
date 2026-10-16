# GW1100 ESPHome firmware

**Beta. Initial installation requires opening the case and serial flashing;
this can damage or brick your device. No warranty.**
Independent project, not affiliated with or endorsed by Ecowitt or Fine Offset.

Replacement firmware for the Ecowitt GW1100 (ESP8266). Receives multiple Ecowitt /
Fine Offset sensors, keeps their readings separate, and sends them to Home Assistant
through ESPHome, MQTT or the Ecowitt integration. An on-device web interface shows
the sensors and edits upload settings. Ecowitt cloud uploads are optional.

The main reason for this firmware is multi-sensor support, including independent
remote indoor sensors instead of replacing the gateway's own temperature, humidity
and pressure readings. MQTT preserves individual sensor identities; Ecowitt uploads
remain subject to that protocol's channel limits.

## Build

Use ESPHome 2026.7.4. Copy `secrets.example.yaml` to `secrets.yaml`, enter your own
settings, then run `esphome compile gw1100.yaml`. After initial installation, update
with `esphome upload gw1100.yaml --device <device-ip>`.

## Serial pads

Use a **3.3 V UART adapter** and common ground. The following pad labels were
confirmed during serial recovery on a GW1100 board in August 2026:

| Connection | Board pad |
|---|---|
| Adapter TX → device RX | TP2 |
| Adapter RX ← device TX | TP13 |
| Ground | ESP module metal shield / GND |
| Boot strap (GPIO0) | TP15; ground during reset, then release |

Back up the entire 2 MB flash before writing. The original vendor OTA route is not
an ESPHome installation method. Pause weather uploads during ESPHome OTA updates.

## Notes

`/health` provides compact upload, heap, reset-reason and rain-recovery diagnostics.
Full `/state` responses use bounded memory; concurrent or oversized requests return
HTTP 503 instead of risking a reset. Diagnostics and outbound HTTP uploads share
a memory gate; the dashboard retries busy reads on its next poll. Configuration
bodies are limited to 2 KB.

The primary rain sensor's rolling 24-hour and ten-minute windows are checkpointed
to RTC memory and aged against the clock after a restart or OTA. This does not
write flash or change existing settings. RTC storage can be lost when power is
removed. For migration or a cold start, a retained recorder can restore dated rain
counter observations through `/config`; see [rain recovery](docs/rain-recovery.md).

Keep your own stock backup and a serial recovery route.
The web interface has no authentication: use it only on a trusted local network.
Optional cloud uploads use Ecowitt's service and are subject to its terms.

Source is licensed under GPL-3.0-only, with compatible third-party material listed
in [THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md). Run `tests/run.sh` for host checks.
