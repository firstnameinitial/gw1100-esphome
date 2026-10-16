# Rain recovery

The WS85/WS90 raw wet/start flag is independent of measured rainfall. Alerts should
use the derived measured rate; the raw flag is retained for diagnostics.

The primary rain sensor's calendar totals retain their existing flash layout.
Rolling 24-hour and ten-minute windows now use a separate checksummed RTC
preference, saved after each valid dated sample. ESPHome allocates the RTC region;
the eboot and crash-handler reservations are untouched. Restoration waits for a
valid clock and ages both windows by the elapsed wall-clock time. Rain outside a
window expires instead of being renewed by a restart. Before clock sync, pending
window values are unknown and omitted from HTTP uploads.

This checkpoint covers the first rain sensor, matching calendar persistence.
Other rain sensors retain independent in-memory windows. RTC survives ordinary
restarts and OTA, but is not power-loss storage. A cold start or upgrade from older
firmware needs retained external history to recover prior rolling observations.

An explicit recovery request can replay up to 64 dated cumulative counters:

```json
{
  "rain_history": {
    "fam": "WS90",
    "id": "123456",
    "samples": [[1792100000, 100.0], [1792103600, 100.3]]
  }
}
```

POST this object alone to `/config`. Include a baseline preceding the rolling
window, then all counter changes within it. Points must be strictly ordered, no
more than 25 hours old, and not in the future. The final counter must match the
current primary sensor counter. Counter resets contribute no negative rainfall.
Malformed, mismatched or oversized requests are rejected. Successful replay
replaces only the rolling windows and updates their RTC checkpoint; it does not
change calendar or lifetime totals. The next RF reading publishes the recovered
values. An incomplete input cannot be detected automatically: use authoritative
retained history, not guessed amounts. The 2 KB body limit still applies.

`/health` reports `recovery.rain_pending`, `rain_restored`, `rain_rtc_saved` and
`state_rejected`. Reset diagnostics include the SDK reason, exception cause and
program counter. A read-only diagnostic rejection does not prove a device fault;
retry the compact endpoint after the active response finishes.
