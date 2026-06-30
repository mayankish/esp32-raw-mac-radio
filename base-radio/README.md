# base-radio

ESP-IDF v5.x firmware for the ESP32 that sits on the "base station" side
of the system — the only ESP32 in this project that joins a real Wi-Fi
network. One of two firmware images in
[`esp32-raw-mac-radio`](../README.md) — see that file for how base-radio
and `bot-radio` relate, and the "why raw frames" design rationale.
See [`DATA_CONTRACT.md`](../DATA_CONTRACT.md) for the wire format.

## What this firmware does

```
                          raw 802.11, ch 6                UDP :5005 (broadcast)
  bot-radio  <===========================>  base-radio  ===================>  dashboard / app
                                                          <===================
                                                          UDP :5006 (unicast)
                                              |
                                         Wi-Fi STA (real AP) -- IP + mDNS (lidarbase.local)
```

- Joins the Wi-Fi network configured in `main/wifi_credentials.h` (see
  "Configuration" below), gets an IP via DHCP.
- Advertises itself over mDNS as **`lidarbase.local`**.
- Runs `raw_link_rx_start()` (promiscuous mode, same OUI-tagged
  vendor-action-frame scheme as bot-radio) and re-emits **every** frame it
  receives over the raw link as a UDP broadcast datagram on **port
  5005** — see "Why broadcast everything" below.
- Listens for `control_command` frames as UDP unicast datagrams on
  **port 5006** and forwards them to bot-radio over the raw link.
- Tracks the data contract's `seq` field across all frames received over
  the raw link and logs (`ESP_LOGW`) any gap as a count of probably-lost
  frames — see `track_sequence()` in `main.c`.

## Why broadcast everything on :5005, not just telemetry

This project's data flow names `scan_sample`/`scan_complete`/`health_status`
as "telemetry" and `control_ack` as part of the control path "flowing
back" — but all five packet types share one wire format and one `type`
field, and base-radio has no reason to special-case any of them: it
isn't a application-aware router, it's a link bridge. Broadcasting every
frame type it receives over the raw link onto UDP :5005 means the
dashboard and Android app (which already have to dispatch on `type` to
handle `scan_sample` vs `scan_complete` vs `health_status` distinctly)
just handle `control_ack` the same way, as one more frame type on the
same socket — no second control-ack-specific channel needed. This is a
documented, deliberate simplification, not an oversight; see the
"resolved ambiguity" note in
[`DATA_CONTRACT.md`](../DATA_CONTRACT.md).

## Configuration

```sh
cp main/wifi_credentials.h.example main/wifi_credentials.h
# edit main/wifi_credentials.h with your SSID/password
```

`wifi_credentials.h` is gitignored — never commit real credentials.

**Channel-lock caveat (read this if the raw link to bot-radio seems
dead):** once base-radio associates with your AP, the ESP32's single
radio locks onto the AP's channel, which then governs promiscuous-mode
RX too — overriding whatever `RAW_LINK_CHANNEL` (`main/raw_link.h`,
default 6) was set to before association. **Configure your AP's channel
to match `RAW_LINK_CHANNEL` in your router's admin settings.** Both
firmwares log their effective channel at boot so a mismatch is visible
in the serial monitor immediately rather than failing silently.

## Toolchain

ESP-IDF v5.x — same as bot-radio:

```sh
. $IDF_PATH/export.sh
```

## Build & flash

```sh
idf.py set-target esp32
idf.py build
idf.py -p /dev/ttyUSB0 flash monitor
```

## API surface for clients (dashboard / Android app)

| What | How |
|---|---|
| Discover base-radio | mDNS hostname `lidarbase.local`, or manual IP fallback |
| Receive telemetry + ack | UDP, bind/listen on **0.0.0.0:5005**, expect 16-byte `lb_frame_t` wire datagrams (broadcast) |
| Send control commands | UDP unicast to `lidarbase.local:5006` (or manual IP), 16-byte `control_command` wire datagram |

## File structure

```
base-radio/
├── CMakeLists.txt
├── sdkconfig.defaults
├── main/
│   ├── CMakeLists.txt
│   ├── main.c                      Wi-Fi STA, mDNS, UDP sockets, raw link, seq tracking
│   ├── data_contract.h/.c          byte-identical copy of the canonical wire format
│   ├── raw_link.h/.c               raw 802.11 TX/RX helpers (shared design with bot-radio)
│   ├── wifi_credentials.h.example  template (tracked)
│   └── wifi_credentials.h          your real credentials (gitignored)
├── .gitignore / LICENSE
```

## Data contract types touched

base-radio passes every frame type through unmodified, in both
directions: all five types when relayed from radio to UDP :5005, and
`control_command` (`type=0x10`) specifically when relayed from UDP :5006
to radio (other inbound UDP datagram types/lengths are logged and
dropped — see `control_listener_task` in `main.c`).

## Known limitations

- **No retransmission**: corrupt or out-of-sequence frames are logged
  (CRC failures, `track_sequence()`'s gap warnings, malformed control
  datagrams) but never retried.
- **`SO_BROADCAST` to `255.255.255.255`**, not a subnet-specific broadcast
  address — works on the vast majority of home/lab networks but may be
  filtered by some router configurations; documented here rather than
  silently assumed to always work.
- Wi-Fi STA reconnect logic (`WIFI_EVENT_STA_DISCONNECTED` handler) is a
  simple immediate-retry, no backoff — acceptable for a small lab/demo
  network, not tuned for a flaky production AP.
- Written against the documented ESP-IDF v5.x Wi-Fi/mDNS/lwIP-socket
  APIs but **not run on physical hardware** as part of producing this
  repo (no ESP32 boards available yet) —
  see "Testing" in `../README.md` for the full verification statement.

## License

MIT — see [`LICENSE`](LICENSE).
