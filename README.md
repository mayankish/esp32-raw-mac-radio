# esp32-raw-mac-radio

Two ESP-IDF v5.x firmware images that form the wireless hop between the
bot's STM32 ([`stm32-lidar-firmware`](https://github.com/mayankish/stm32-lidar-firmware))
and the base-station clients ([`lidar-android-app`](https://github.com/mayankish/lidar-android-app),
[`lidar-slam-dashboard`](https://github.com/mayankish/lidar-slam-dashboard)).
This is **Project 2** of a small multi-repo lidar-mapping robot project
I've been building in my spare time. See
[`DATA_CONTRACT.md`](DATA_CONTRACT.md) for the canonical wire format
both images speak.

![Architecture: bot-radio bridges STM32 UART to base-radio over raw 802.11 frames; base-radio bridges that link to UDP/mDNS for the dashboard and Android app](docs/esp32_bridge_architecture.png)

```
[bot-radio]                                              [base-radio]
  UART1 <-> STM32                                          Wi-Fi STA -> AP -> IP, mDNS lidarbase.local
      |                                                         |
      +------------------ raw 802.11, fixed channel ------------+
                        (esp_wifi_80211_tx / promiscuous RX,
                         no association, no IP on this hop)
                                                                 |
                                                          UDP :5005 broadcast (out)
                                                          UDP :5006 unicast  (in)
```

| | bot-radio | base-radio |
|---|---|---|
| Role | Lives next to the STM32, on the bot | Lives at the base station |
| Joins a Wi-Fi network? | No | Yes (needs an IP for UDP/mDNS) |
| Talks to the STM32? | Yes, UART1 | No |
| Talks to dashboard/app? | No | Yes, UDP |
| README | [`bot-radio/README.md`](bot-radio/README.md) | [`base-radio/README.md`](base-radio/README.md) |

## Why raw 802.11 frames instead of Wi-Fi sockets or ESP-NOW

bot-radio has nowhere to get an IP address from in the field and no
reason to wait through AP association/DHCP just to move bytes a few
meters to base-radio. ESP-NOW would avoid needing an AP, but it brings
its own framing, peer-table and (optional) encryption machinery for a
payload that `data_contract.h` already frames, checksums, and sequences
— adopting it would mean paying ESP-NOW's overhead to re-solve a problem
already solved one layer up. `esp_wifi_80211_tx()` plus promiscuous-mode
RX is the thinnest layer the ESP32 Wi-Fi radio exposes that still moves
bytes over the air, unassociated, on a fixed channel: "be a wire, but
wireless." The cost of this choice is the channel-lock caveat below,
which is a real operational requirement, not a hidden bug.

Both firmwares tag their raw frames with a locally-administered,
project-specific OUI (`common/raw_link.h`) inside a vendor-specific
802.11 Action frame, so the promiscuous RX callback can cheaply ignore
all the normal Wi-Fi traffic (beacons, probe requests, other devices)
sharing the channel before doing any real parsing.

## Channel-lock caveat

base-radio is the only one of the two that joins a real AP. Once it
associates, the ESP32's single radio locks onto the AP's channel — which
also governs the raw link's promiscuous RX, regardless of what
`RAW_LINK_CHANNEL` (`*/main/raw_link.h`, default channel 6) was set to
before association. **Fix: configure your Wi-Fi AP's channel (in its own
admin settings) to match `RAW_LINK_CHANNEL` exactly.** Both firmwares log
their effective channel at boot, so a mismatch shows up immediately in
the serial monitor instead of failing silently. See
[`base-radio/README.md`](base-radio/README.md) for more detail.

## Shared code, duplicated on purpose

`common/data_contract.h` / `.c` and `common/raw_link.h` / `.c` are the
canonical versions of code shared between bot-radio and base-radio. They
are duplicated byte-for-byte into `bot-radio/main/` and `base-radio/main/`
rather than referenced via a shared include path — each is an
independent ESP-IDF project meant to build standalone. If you change either file, update all three
copies (`common/`, `bot-radio/main/`, `base-radio/main/`).

## File structure

```
esp32-raw-mac-radio/
├── README.md              (this file)
├── docs/                  architecture diagram
├── common/                canonical data_contract + raw_link (see above)
├── bot-radio/              ESP-IDF project: UART<->raw-link bridge
│   └── README.md
└── base-radio/             ESP-IDF project: raw-link<->UDP/mDNS bridge
    └── README.md
```

## Testing

Neither firmware has been run on physical ESP32 hardware yet
(no ESP32 boards available at this stage of development).
Both are written against the documented ESP-IDF v5.x APIs
(`esp_wifi_80211_tx`, promiscuous-mode RX, UART driver, lwIP sockets,
`mdns_*`) as they would be for real hardware, with every hardware-facing
assumption (UART pins/baud, the hand-rolled 802.11 header layout, channel
behavior) called out explicitly in the relevant README rather than left
implicit. See each subfolder's README "Known limitations" for the
itemized list.

## License

MIT — see [`LICENSE`](LICENSE). Each subfolder also carries its own copy
for independent-repo use later.
