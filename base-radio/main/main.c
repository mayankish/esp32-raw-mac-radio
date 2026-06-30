/*
 * base-radio main.c
 *
 * Joins a normal Wi-Fi network (for an IP address + mDNS), bridges the
 * raw 802.11 link to/from bot-radio, and exposes the result to the
 * dashboard/app over UDP:
 *
 *   radio -> UDP broadcast :5005   (every frame received from bot-radio,
 *                                   telemetry AND control_ack alike --
 *                                   see ../README.md "why broadcast
 *                                   everything" for the reasoning)
 *   UDP unicast :5006 -> radio     (control_command frames from the
 *                                   dashboard/app, forwarded to bot-radio)
 *
 * Advertises itself via mDNS as lidarbase.local so clients don't need a
 * manual IP (the Android app and dashboard both fall back to a manual-IP
 * field if mDNS resolution fails on their network).
 *
 * See ../README.md for the channel-lock caveat this firmware is subject
 * to: associating with the AP below can move the radio off
 * RAW_LINK_CHANNEL (main/raw_link.h), so the AP's own channel must be
 * fixed to match.
 */
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"

#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "mdns.h"

#include "lwip/sockets.h"
#include "lwip/inet.h"

#include "data_contract.h"
#include "raw_link.h"
#include "wifi_credentials.h"

static const char *TAG = "base-radio";

#define TELEMETRY_UDP_PORT 5005
#define CONTROL_UDP_PORT   5006
#define MDNS_HOSTNAME      "lidarbase"

static EventGroupHandle_t s_wifi_event_group;
#define WIFI_CONNECTED_BIT BIT0

/* Frames received over the raw link, queued from the (non-blocking)
 * promiscuous callback for a normal task to drain and UDP-broadcast.
 * Depth 8, same backpressure reasoning as bot-radio's radio_rx_queue. */
static QueueHandle_t s_radio_rx_queue;

/* ---------------- sequence-gap drop counter ----------------
 * bot-radio's USART1 link uses ONE shared, monotonically increasing seq
 * space across every frame type (see stm32-lidar-firmware/Src/uart.c,
 * uart1_send_frame()) -- which means base-radio can track loss the same
 * simple way: one running "expected next seq" counter across everything
 * it receives over the raw link, regardless of frame type. A gap means
 * frames were lost somewhere between the STM32 and here (UART RX
 * overrun, raw-link RX miss, CRC failure -- any of those). This does NOT
 * distinguish which cause, only that *something* was lost, which is
 * logged, not silently swallowed. */
static int s_seq_initialized;
static uint16_t s_expected_seq;
static uint32_t s_total_lost;

static void track_sequence(uint16_t seq) {
    if (!s_seq_initialized) {
        s_seq_initialized = 1;
        s_expected_seq = (uint16_t)(seq + 1u);
        return;
    }
    if (seq != s_expected_seq) {
        uint16_t gap = (uint16_t)(seq - s_expected_seq); /* wraps correctly mod 65536 */
        s_total_lost += gap;
        ESP_LOGW(TAG, "sequence gap: expected %u, got %u (lost %u, total lost %u)",
                  s_expected_seq, seq, gap, (unsigned)s_total_lost);
    }
    s_expected_seq = (uint16_t)(seq + 1u);
}

/* ---------------- Wi-Fi STA bring-up ---------------- */

static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                                 int32_t event_id, void *event_data) {
    (void)arg; (void)event_data;
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        ESP_LOGW(TAG, "STA disconnected, retrying...");
        esp_wifi_connect();
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *evt = (ip_event_got_ip_t *)event_data;
        ESP_LOGI(TAG, "got IP: " IPSTR, IP2STR(&evt->ip_info.ip));
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

static void wifi_sta_init(void) {
    s_wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL));

    wifi_config_t wifi_config = { 0 };
    strncpy((char *)wifi_config.sta.ssid, WIFI_SSID, sizeof(wifi_config.sta.ssid) - 1);
    strncpy((char *)wifi_config.sta.password, WIFI_PASS, sizeof(wifi_config.sta.password) - 1);

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    /* Best-effort: set our preferred raw-link channel before association.
     * Once esp_wifi_connect() succeeds (triggered from the event handler
     * above on WIFI_EVENT_STA_START), the AP dictates the real channel --
     * this is the channel-lock caveat from raw_link.h/README.md, made
     * concrete: if the two don't match, this call's effect is overridden
     * the moment association completes. */
    esp_wifi_set_channel(RAW_LINK_CHANNEL, WIFI_SECOND_CHAN_NONE);

    ESP_LOGI(TAG, "connecting to SSID '%s'...", WIFI_SSID);
    xEventGroupWaitBits(s_wifi_event_group, WIFI_CONNECTED_BIT, pdFALSE, pdTRUE, portMAX_DELAY);
}

/* ---------------- mDNS ---------------- */

static void mdns_setup(void) {
    ESP_ERROR_CHECK(mdns_init());
    ESP_ERROR_CHECK(mdns_hostname_set(MDNS_HOSTNAME));
    ESP_ERROR_CHECK(mdns_instance_name_set("lidar-bot-system base radio"));
    ESP_LOGI(TAG, "mDNS hostname set: %s.local", MDNS_HOSTNAME);
}

/* ---------------- radio -> UDP broadcast :5005 ---------------- */

static void on_raw_frame_from_bot(const uint8_t *payload, size_t len) {
    if (len != LB_WIRE_LEN) {
        return;
    }
    uint8_t copy[LB_WIRE_LEN];
    memcpy(copy, payload, LB_WIRE_LEN);
    if (xQueueSend(s_radio_rx_queue, copy, 0) != pdTRUE) {
        ESP_LOGW(TAG, "radio_rx_queue full, dropping one telemetry frame");
    }
}

static void telemetry_broadcast_task(void *arg) {
    (void)arg;

    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (sock < 0) {
        ESP_LOGE(TAG, "failed to create broadcast socket: errno %d", errno);
        vTaskDelete(NULL);
        return;
    }
    int broadcast_enable = 1;
    setsockopt(sock, SOL_SOCKET, SO_BROADCAST, &broadcast_enable, sizeof(broadcast_enable));

    struct sockaddr_in dest = { 0 };
    dest.sin_family = AF_INET;
    dest.sin_port = htons(TELEMETRY_UDP_PORT);
    dest.sin_addr.s_addr = htonl(INADDR_BROADCAST); /* 255.255.255.255 */

    uint8_t wire[LB_WIRE_LEN];
    for (;;) {
        if (xQueueReceive(s_radio_rx_queue, wire, portMAX_DELAY) == pdTRUE) {
            lb_frame_t f;
            if (!lb_unpack(wire, &f)) {
                ESP_LOGW(TAG, "dropped raw-link frame with bad CRC");
                continue; /* not counted in the sequence-gap tracker --
                             a corrupt frame's seq field isn't trustworthy */
            }
            track_sequence(f.seq);

            int sent = sendto(sock, wire, LB_WIRE_LEN, 0,
                                (struct sockaddr *)&dest, sizeof(dest));
            if (sent < 0) {
                ESP_LOGW(TAG, "UDP broadcast sendto failed: errno %d", errno);
            }
        }
    }
}

/* ---------------- UDP unicast :5006 -> radio ---------------- */

static void control_listener_task(void *arg) {
    (void)arg;

    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (sock < 0) {
        ESP_LOGE(TAG, "failed to create control socket: errno %d", errno);
        vTaskDelete(NULL);
        return;
    }
    int reuse = 1;
    setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));

    struct sockaddr_in bind_addr = { 0 };
    bind_addr.sin_family = AF_INET;
    bind_addr.sin_port = htons(CONTROL_UDP_PORT);
    bind_addr.sin_addr.s_addr = htonl(INADDR_ANY);

    if (bind(sock, (struct sockaddr *)&bind_addr, sizeof(bind_addr)) < 0) {
        ESP_LOGE(TAG, "failed to bind control socket to port %d: errno %d", CONTROL_UDP_PORT, errno);
        vTaskDelete(NULL);
        return;
    }
    ESP_LOGI(TAG, "control listener bound on UDP :%d", CONTROL_UDP_PORT);

    uint8_t buf[LB_WIRE_LEN];
    for (;;) {
        struct sockaddr_in src;
        socklen_t src_len = sizeof(src);
        int n = recvfrom(sock, buf, sizeof(buf), 0, (struct sockaddr *)&src, &src_len);
        if (n != (int)LB_WIRE_LEN) {
            if (n > 0) {
                ESP_LOGW(TAG, "dropped control datagram with wrong length %d (expected %u)", n, LB_WIRE_LEN);
            }
            continue;
        }

        lb_frame_t f;
        if (!lb_unpack(buf, &f) || f.sof != LB_SOF_CONTROL || f.type != LB_TYPE_CONTROL_COMMAND) {
            ESP_LOGW(TAG, "dropped invalid/non-control_command UDP datagram on :%d", CONTROL_UDP_PORT);
            continue;
        }

        esp_err_t err = raw_link_send(buf, LB_WIRE_LEN);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "raw_link_send (control_command) failed: %s", esp_err_to_name(err));
        }
    }
}

void app_main(void) {
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    s_radio_rx_queue = xQueueCreate(8, LB_WIRE_LEN);

    wifi_sta_init();      /* blocks until associated + IP obtained */
    mdns_setup();
    ESP_ERROR_CHECK(raw_link_rx_start(on_raw_frame_from_bot));

    xTaskCreate(telemetry_broadcast_task, "telem_bcast", 4096, NULL, 5, NULL);
    xTaskCreate(control_listener_task,    "ctrl_listen", 4096, NULL, 5, NULL);

    ESP_LOGI(TAG, "base-radio up: lidarbase.local, UDP :%d broadcast / :%d control",
              TELEMETRY_UDP_PORT, CONTROL_UDP_PORT);
}
