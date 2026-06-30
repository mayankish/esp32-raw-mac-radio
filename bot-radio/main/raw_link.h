/*
 * raw_link.h -- shared raw-802.11 link layer used by both bot-radio and
 * base-radio to exchange lidar-bot-system wire frames (LB_WIRE_LEN = 16
 * bytes each) WITHOUT joining a Wi-Fi network or using ESP-NOW.
 *
 * This is a byte-for-byte duplicate in bot-radio/main/ and base-radio/
 * main/ (same rationale as data_contract.h: each ESP-IDF project must
 * build standalone). If you change the framing here, update both copies
 * AND the "why raw frames" section of esp32-raw-mac-radio/README.md.
 *
 * ---------------------------------------------------------------------
 * WHY RAW 802.11 INSTEAD OF ESP-NOW OR NORMAL WI-FI SOCKETS:
 *
 * bot-radio never associates with an access point -- it has nowhere to
 * get an IP address from in the field, and association/DHCP adds seconds
 * of latency and a dependency on AP availability that the bot-to-base
 * telemetry link shouldn't have. ESP-NOW would solve "no AP" but pulls in
 * its own framing, encryption and peer-table machinery for a payload that
 * is already framed, checksummed and sequenced by the data contract --
 * using it would mean paying ESP-NOW's overhead to re-solve a problem
 * data_contract.h already solves. Raw esp_wifi_80211_tx()/promiscuous-RX
 * is the thinnest layer available on the ESP32 Wi-Fi radio that still
 * gets bytes over the air on a fixed channel with no association, which
 * is exactly "be a wire, but wireless" -- the design goal for this hop.
 * The cost is the channel-lock caveat documented below and in both
 * subproject READMEs.
 * ---------------------------------------------------------------------
 * CHANNEL-LOCK CAVEAT (read this before debugging "no packets arrive"):
 *
 * base-radio ALSO joins a normal Wi-Fi AP (for its own IP address, so it
 * can run mDNS + UDP sockets for the dashboard/app). On the ESP32, a
 * single radio can only listen on one channel at a time. Once base-radio
 * associates with an AP, the Wi-Fi driver locks the radio to that AP's
 * channel -- promiscuous-mode RX (used to receive bot-radio's raw frames)
 * is then confined to that same channel too, whatever RAW_LINK_CHANNEL
 * below says.
 *
 * Workaround used by this project (not hidden, not "fixed" -- a
 * documented operational requirement): configure your Wi-Fi AP's channel
 * in its own admin settings to match RAW_LINK_CHANNEL exactly. Both
 * firmwares log their effective channel at boot so a mismatch is visible
 * immediately in the serial monitor rather than failing silently.
 * ---------------------------------------------------------------------
 */
#ifndef RAW_LINK_H
#define RAW_LINK_H

#include <stdint.h>
#include <stddef.h>
#include "esp_err.h"

/* Fixed channel for the raw link. MUST match the local Wi-Fi AP's
 * configured channel (see caveat above) and MUST be identical in both
 * bot-radio and base-radio's sdkconfig/build -- there is no channel
 * negotiation in v1. 2.4GHz channel 6 chosen as a common default least
 * likely to require AP reconfiguration on a typical home router, but any
 * channel 1-13 works as long as both firmwares and the AP agree. */
#define RAW_LINK_CHANNEL 6

/* Locally-administered OUI (first octet 0x02 sets the "locally
 * administered" bit, so this can never collide with a real IEEE-assigned
 * OUI) used to tag our vendor-specific action frames, so the
 * promiscuous-RX callback can cheaply reject beacons/probes/other
 * traffic before doing any real parsing. */
#define RAW_LINK_OUI { 0x02, 0xFE, 0xC0 }

/* Fixed pseudo-BSSID used in the 802.11 header's addr3 field. This is not
 * a real access point's BSSID -- it's just a fixed value both ends
 * recognize as "this frame belongs to lidar-bot-system", since we never
 * associate with anything on this link. */
#define RAW_LINK_BSSID { 0x02, 0xFE, 0xC0, 0x4C, 0x42, 0x01 }

#define RAW_LINK_MAX_PAYLOAD 16u /* == LB_WIRE_LEN */

/* Called from the promiscuous-mode RX path (NOT from the Wi-Fi task's own
 * context with much stack, and NOT safe to block in -- queue the data and
 * return, same convention as any IDF promiscuous callback). `payload` is
 * valid only for the duration of the call. */
typedef void (*raw_link_rx_cb_t)(const uint8_t *payload, size_t len);

/* Brings the Wi-Fi driver up far enough to transmit/receive raw 802.11
 * frames: esp_netif_init + default event loop + esp_wifi_init +
 * esp_wifi_set_mode(WIFI_MODE_STA) + esp_wifi_start(), WITHOUT calling
 * esp_wifi_connect(). Sets the channel to RAW_LINK_CHANNEL.
 *
 * Used by bot-radio, which never associates with an AP, so this one call
 * is its entire Wi-Fi bring-up. base-radio does NOT call this function --
 * it needs a real esp_netif STA object (via
 * esp_netif_create_default_wifi_sta()) wired up before esp_wifi_init() so
 * it can receive IP events and run mDNS, which this helper doesn't set
 * up. base-radio instead performs its own full STA bring-up (see its
 * main.c) and then calls raw_link_rx_start()/raw_link_send() directly --
 * both of those only touch the already-running Wi-Fi driver and don't
 * care how it was brought up. This is also where the channel-lock caveat
 * above actually bites: base-radio's own esp_wifi_connect() to a real AP
 * happens after Wi-Fi start, and the resulting association can move the
 * radio off RAW_LINK_CHANNEL onto whatever channel the AP uses. */
esp_err_t raw_link_radio_init(void);

/* Registers `cb` and enables promiscuous mode filtered to management
 * frames only (our raw frames are vendor-specific Action frames, which
 * are management-type). Internally validates the OUI marker and frame
 * length before ever calling `cb`, so `cb` only ever sees frames that are
 * actually ours. */
esp_err_t raw_link_rx_start(raw_link_rx_cb_t cb);

/* Wraps `payload` (must be exactly RAW_LINK_MAX_PAYLOAD bytes -- this
 * project only ever sends whole lb_frame_t wire buffers, never partial
 * data) in the fixed 802.11 Action-frame header described in raw_link.c
 * and transmits it via esp_wifi_80211_tx() on WIFI_IF_STA. Broadcast
 * destination, no ACK expected or waited for -- loss is handled at the
 * data-contract layer (sequence numbers + CRC), not here. */
esp_err_t raw_link_send(const uint8_t *payload, size_t len);

#endif /* RAW_LINK_H */
