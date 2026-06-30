# bot-radio

ESP-IDF v5.x firmware for the ESP32 mounted on the bot, alongside the
STM32 Nucleo board ([`stm32-lidar-firmware`](https://github.com/mayankish/stm32-lidar-firmware)).
This is one of two firmware images in
[`esp32-raw-mac-radio`](../README.md) — see that file for how bot-radio
and `base-radio` relate, and the "why raw frames" design rationale.
See [`DATA_CONTRACT.md`](../DATA_CONTRACT.md) for the wire format.

## What this firmware does

A dumb, byte-verbatim bridge between the STM32's UART1 and the raw
802.11 link to base-radio:

```
STM32 USART1  <-- UART1 (115200 8N1) -->  bot-radio  <-- raw 802.11, ch 6 -->  base-radio
```

- **UART1 -> radio**: reads bytes from the STM32, finds frame boundaries
  by watching for a valid SOF byte (`0xAA` telemetry / `0xAB` control),
  collects the following 16-byte wire frame, and forwards it **verbatim**
  over the raw link via `esp_wifi_80211_tx()`. It does perform a CRC
  check, but only to count and log corrupt frames (`uart_rx_task` in
  `main.c`) — it never re-encodes or modifies a frame it forwards.
- **radio -> UART1**: any raw frame received from base-radio (control
  commands) is queued from the promiscuous-mode callback and written
  straight to UART1 toward the STM32, again byte-verbatim.

It never joins a Wi-Fi network, never gets an IP address, and runs no
sockets — see `../README.md` for why.

## Hardware / wiring

| ESP32 pin | Function | Connects to |
|---|---|---|
| GPIO17 | UART TX | STM32 PA10 (USART1_RX) |
| GPIO16 | UART RX | STM32 PA9 (USART1_TX) |
| GND | — | STM32 GND (common ground, required) |

Baud rate is **115200 8N1**, hardcoded in `main.c` (`LB_UART_BAUD`) and
must match `stm32-lidar-firmware`'s `UART1_BAUD` in `Src/main.c` exactly.

## Toolchain

ESP-IDF v5.x (tested against the v5.x release line; any v5.x point
release should work — nothing here uses a version-specific API). Follow
Espressif's standard setup:

```sh
. $IDF_PATH/export.sh   # or install_idf.sh equivalent for your OS
```

## Build & flash

```sh
idf.py set-target esp32
idf.py build
idf.py -p /dev/ttyUSB0 flash monitor    # adjust port for your OS
```

## File structure

```
bot-radio/
├── CMakeLists.txt          top-level ESP-IDF project file
├── sdkconfig.defaults      seeds menuconfig with project-relevant defaults
├── main/
│   ├── CMakeLists.txt
│   ├── main.c              UART<->radio bridge, two FreeRTOS tasks
│   ├── data_contract.h/.c  byte-identical copy of the canonical wire format
│   └── raw_link.h/.c       raw 802.11 TX/RX helpers (shared design with base-radio)
├── .gitignore / LICENSE
```

## Data contract types touched

bot-radio is a pure pipe — it parses just enough of each frame to find
its 16-byte boundary and verify CRC for logging (`lb_unpack`), but never
inspects `type` or payload contents. All five packet types pass through
unmodified in whichever direction they're flowing.

## Known limitations

- **No retransmission**: dropped/corrupt UART or radio frames are logged
  (`uart_rx_task`'s CRC-failure counter, `radio_rx_queue`-full warnings)
  but never retried — loss recovery, if any, happens at the application
  layer (dashboard/app), not here.
- **Single fixed channel** (`RAW_LINK_CHANNEL` in `raw_link.h`, default 6)
  — no channel scanning or negotiation.
- The raw-802.11 header building/parsing in `raw_link.c` was written
  against the documented ESP-IDF v5.x promiscuous-mode and
  `esp_wifi_80211_tx()` APIs but has **not been run on physical hardware**
  as part of producing this repo (no ESP32 boards available yet) —
  see `../README.md` "Testing" for the full
  verification statement covering both bot-radio and base-radio.

## License

MIT — see [`LICENSE`](LICENSE).
