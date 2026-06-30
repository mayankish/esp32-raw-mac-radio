/*
 * raw_link.c -- see raw_link.h for the design rationale ("why raw
 * frames") and the channel-lock caveat. This file builds/parses a fixed,
 * minimal IEEE 802.11 Action-frame header by hand; it is not using
 * ESP-NOW or any IDF Wi-Fi-frame helper because none of those expose
 * "send these exact bytes, unassociated, on a fixed channel" any more
 * directly than esp_wifi_80211_tx() already does.
 *
 * Frame layout this code transmits/expects (45 bytes total: 29-byte
 * header + 16-byte lidar-bot-system wire frame):
 *
 *   offset  size  field
 *   0       2     Frame Control: 0xD0,0x00 (Management/Action, no flags)
 *   2       2     Duration: 0x0000 (no airtime reservation)
 *   4       6     Addr1 (RA/DA): broadcast FF:FF:FF:FF:FF:FF
 *   10      6     Addr2 (TA/SA): this radio's own STA MAC
 *   16      6     Addr3 (BSSID): fixed pseudo-BSSID, RAW_LINK_BSSID
 *   22      2     Seq/Frag control: 0x0000 (lb_frame_t.seq already
 *                 sequences these packets at the data-contract layer;
 *                 see raw_link.h)
 *   24      1     Category: 0x7F (127 = vendor-specific)
 *   25      1     Action field: 0x00 (only one action subtype defined here)
 *   26      3     OUI: RAW_LINK_OUI
 *   29      16    Payload: a complete lb_frame_t wire buffer, verbatim
 */
#include "raw_link.h"

#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_log.h"
#include <string.h>

static const char *TAG = "raw_link";

#define HDR_LEN 29u
#define FRAME_LEN (HDR_LEN + RAW_LINK_MAX_PAYLOAD)

static const uint8_t kOui[3] = RAW_LINK_OUI;
static const uint8_t kBssid[6] = RAW_LINK_BSSID;
static const uint8_t kBroadcast[6] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };

static raw_link_rx_cb_t s_user_cb;

esp_err_t raw_link_radio_init(void) {
    esp_err_t err;

    err = esp_netif_init();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) return err; /* INVALID_STATE = already initialized by caller (base-radio does this for its own STA netif) */

    err = esp_event_loop_create_default();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) return err;

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    err = esp_wifi_init(&cfg);
    if (err != ESP_OK && err != ESP_ERR_WIFI_INIT_STATE) return err;

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_set_channel(RAW_LINK_CHANNEL, WIFI_SECOND_CHAN_NONE));

    ESP_LOGI(TAG, "raw link radio up on channel %d", RAW_LINK_CHANNEL);
    return ESP_OK;
}

static void IRAM_ATTR promisc_rx_cb(void *buf, wifi_promiscuous_pkt_type_t type) {
    if (type != WIFI_PKT_MGMT) {
        return;
    }
    wifi_promiscuous_pkt_t *pkt = (wifi_promiscuous_pkt_t *)buf;
    const uint8_t *p = pkt->payload;
    uint16_t len = pkt->rx_ctrl.sig_len;

    if (len < FRAME_LEN) {
        return; /* too short to be one of ours -- not an error, just noise */
    }
    /* Subtype check: high nibble of byte 0 == 0xD (Action). We deliberately
     * don't also require the flags byte (offset 1) to be exactly 0x00,
     * since some link layers reuse reserved flag bits in ways that aren't
     * worth being strict about here -- the OUI check below is the real
     * filter. */
    if ((p[0] & 0xF0) != 0xD0) {
        return;
    }
    if (p[24] != 0x7Fu) { /* category: vendor-specific */
        return;
    }
    if (memcmp(&p[26], kOui, 3) != 0) {
        return; /* someone else's vendor action frame, or noise -- not ours */
    }

    if (s_user_cb) {
        s_user_cb(&p[HDR_LEN], RAW_LINK_MAX_PAYLOAD);
    }
}

esp_err_t raw_link_rx_start(raw_link_rx_cb_t cb) {
    s_user_cb = cb;

    wifi_promiscuous_filter_t filter = {
        .filter_mask = WIFI_PROMIS_FILTER_MASK_MGMT,
    };
    ESP_ERROR_CHECK(esp_wifi_set_promiscuous_filter(&filter));
    ESP_ERROR_CHECK(esp_wifi_set_promiscuous_rx_cb(promisc_rx_cb));
    ESP_ERROR_CHECK(esp_wifi_set_promiscuous(true));

    ESP_LOGI(TAG, "raw link promiscuous RX enabled (mgmt frames, OUI-filtered)");
    return ESP_OK;
}

esp_err_t raw_link_send(const uint8_t *payload, size_t len) {
    if (len != RAW_LINK_MAX_PAYLOAD) {
        return ESP_ERR_INVALID_SIZE;
    }

    uint8_t frame[FRAME_LEN];
    memset(frame, 0, sizeof(frame));

    frame[0] = 0xD0; /* FC: Management, subtype Action */
    frame[1] = 0x00; /* flags: none */
    /* bytes 2-3 duration: left zero */

    memcpy(&frame[4], kBroadcast, 6);   /* addr1 */

    uint8_t own_mac[6];
    if (esp_wifi_get_mac(WIFI_IF_STA, own_mac) == ESP_OK) {
        memcpy(&frame[10], own_mac, 6); /* addr2 */
    }
    /* If esp_wifi_get_mac fails (radio not up yet), addr2 stays
     * 00:00:00:00:00:00 -- the receiver doesn't use addr2 for anything,
     * it's informational only, so this is a soft failure, not fatal. */

    memcpy(&frame[16], kBssid, 6);      /* addr3 */
    /* bytes 22-23 seq/frag control: left zero, see raw_link.h */

    frame[24] = 0x7F; /* category: vendor-specific */
    frame[25] = 0x00; /* action field */
    memcpy(&frame[26], kOui, 3);

    memcpy(&frame[HDR_LEN], payload, RAW_LINK_MAX_PAYLOAD);

    /* en_sys_seq = false: we supply our own framing/sequencing via
     * lb_frame_t.seq, see raw_link.h "why raw frames". */
    return esp_wifi_80211_tx(WIFI_IF_STA, frame, (int)sizeof(frame), false);
}
