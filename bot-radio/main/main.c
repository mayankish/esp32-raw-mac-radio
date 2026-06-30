/*
 * bot-radio main.c
 *
 * Bridges STM32 UART1 <-> the raw 802.11 link (see ../../common/raw_link.h
 * for "why raw frames"). This firmware is a dumb pipe by design: it does
 * not interpret packet contents beyond the framing needed to find frame
 * boundaries and a defensive CRC check used only for drop counting/logging
 * -- it never re-encodes a frame, it forwards the exact 16 bytes it
 * received in both directions ("byte-verbatim").
 *
 * Two independent data flows, each its own task:
 *   uart_rx_task  : STM32 UART1 -> (SOF-anchored framing) -> raw_link_send()
 *   radio_rx_task : promiscuous RX queue -> UART1 TX (back to STM32)
 *
 * Baud rate (115200 8N1) and UART pins MUST match stm32-lidar-firmware's
 * USART1 config exactly -- documented in both READMEs.
 */
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "driver/uart.h"
#include "esp_log.h"
#include "nvs_flash.h"

#include "data_contract.h"
#include "raw_link.h"

static const char *TAG = "bot-radio";

/* ---- UART1 link to the STM32 (see README "Pin map") ---- */
#define LB_UART_PORT     UART_NUM_2
#define LB_UART_TX_GPIO  17
#define LB_UART_RX_GPIO  16
#define LB_UART_BAUD     115200
#define LB_UART_BUF_SIZE 256

/* Frames arriving over the radio, queued from the (non-blocking)
 * promiscuous callback for the UART-TX task to drain. Depth 8 mirrors the
 * STM32 telemetry queue's reasoning: bursts shouldn't be dropped just
 * because the consumer is a tick behind. */
static QueueHandle_t s_radio_rx_queue;

static uint32_t s_uart_rx_crc_drops; /* defensive-check failures, logged */

/* ---------------- radio -> UART1 (control commands inbound) ---------------- */

static void on_raw_frame_from_base(const uint8_t *payload, size_t len) {
    if (len != LB_WIRE_LEN) {
        return; /* raw_link.c already guarantees this; defensive only */
    }
    uint8_t copy[LB_WIRE_LEN];
    memcpy(copy, payload, LB_WIRE_LEN);
    /* Non-blocking: this runs in the Wi-Fi driver's task context, not a
     * true ISR, but it must not stall waiting on a full queue either. */
    if (xQueueSend(s_radio_rx_queue, copy, 0) != pdTRUE) {
        ESP_LOGW(TAG, "radio_rx_queue full, dropping one inbound control frame");
    }
}

static void radio_rx_task(void *arg) {
    (void)arg;
    uint8_t wire[LB_WIRE_LEN];

    for (;;) {
        if (xQueueReceive(s_radio_rx_queue, wire, portMAX_DELAY) == pdTRUE) {
            /* Byte-verbatim forward to the STM32 -- no re-encoding. */
            uart_write_bytes(LB_UART_PORT, (const char *)wire, LB_WIRE_LEN);
        }
    }
}

/* ---------------- UART1 -> radio (telemetry/ack outbound) ---------------- */

static void uart_rx_task(void *arg) {
    (void)arg;
    uint8_t buf[LB_WIRE_LEN];
    int idx = 0;
    int collecting = 0;

    for (;;) {
        uint8_t byte;
        int n = uart_read_bytes(LB_UART_PORT, &byte, 1, pdMS_TO_TICKS(20));
        if (n <= 0) {
            continue;
        }

        if (!collecting) {
            if (byte == LB_SOF_TELEMETRY || byte == LB_SOF_CONTROL) {
                buf[0] = byte;
                idx = 1;
                collecting = 1;
            }
            continue;
        }

        buf[idx++] = byte;
        if (idx == LB_WIRE_LEN) {
            lb_frame_t f;
            if (lb_unpack(buf, &f)) {
                esp_err_t err = raw_link_send(buf, LB_WIRE_LEN);
                if (err != ESP_OK) {
                    ESP_LOGW(TAG, "raw_link_send failed: %s", esp_err_to_name(err));
                }
            } else {
                s_uart_rx_crc_drops++;
                ESP_LOGW(TAG, "dropped UART frame with bad CRC (total drops: %u)",
                         (unsigned)s_uart_rx_crc_drops);
            }
            idx = 0;
            collecting = 0;
        }
    }
}

static void uart1_init(void) {
    uart_config_t cfg = {
        .baud_rate = LB_UART_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_ERROR_CHECK(uart_driver_install(LB_UART_PORT, LB_UART_BUF_SIZE * 2, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(LB_UART_PORT, &cfg));
    ESP_ERROR_CHECK(uart_set_pin(LB_UART_PORT, LB_UART_TX_GPIO, LB_UART_RX_GPIO,
                                  UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
}

void app_main(void) {
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    uart1_init();

    s_radio_rx_queue = xQueueCreate(8, LB_WIRE_LEN);

    ESP_ERROR_CHECK(raw_link_radio_init());
    ESP_ERROR_CHECK(raw_link_rx_start(on_raw_frame_from_base));

    xTaskCreate(uart_rx_task,  "uart_rx",  4096, NULL, 5, NULL);
    xTaskCreate(radio_rx_task, "radio_rx", 4096, NULL, 5, NULL);

    ESP_LOGI(TAG, "bot-radio up: UART1 <-> raw link bridge running");
}
