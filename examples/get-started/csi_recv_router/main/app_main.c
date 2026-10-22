/*
 * SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD
 * SPDX-License-Identifier: Apache-2.0
 *
 * 1000 Hz ICMP traffic generator and CSI collector for ESP32-C5.
 */

#include <errno.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#include "driver/uart.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "nvs_flash.h"

#include "lwip/icmp.h"
#include "lwip/inet.h"
#include "lwip/inet_chksum.h"
#include "lwip/ip4.h"
#include "lwip/prot/ip4.h"
#include "lwip/sockets.h"

#include "protocol_examples_common.h"

#define PING_FREQUENCY_HZ          1000U
#define MEASUREMENT_DURATION_SEC   10U
#define PING_PERIOD_US             (1000000U / PING_FREQUENCY_HZ)
#define PING_COUNT                 (PING_FREQUENCY_HZ * MEASUREMENT_DURATION_SEC)
#define PING_ID                    0x4353U
#define PING_PAYLOAD_SIZE          8U
#define PING_REPLY_GRACE_MS        100U

#define CSI_MAX_LEN                512U
#define CSI_POOL_SIZE              64U

#define CSI_UART                   UART_NUM_0
#define CSI_UART_BAUD_RATE         4000000U
#define CSI_UART_TX_BUFFER_SIZE    32768

#define PROTOCOL_VERSION           1U
#define CSI_FRAME_TYPE             1U
#define RESULT_FRAME_TYPE          2U

#if CONFIG_FREERTOS_HZ != 1000
#error "CONFIG_FREERTOS_HZ must be 1000 for a 1 ms ping schedule"
#endif

static const char *TAG = "csi_1000hz";

typedef struct {
    uint32_t sequence;
    uint32_t timestamp_us;
    uint16_t length;
    int8_t rssi;
    uint8_t channel;
    uint8_t rx_format;
    uint8_t flags;
    int8_t data[CSI_MAX_LEN];
} csi_slot_t;

typedef struct __attribute__((packed)) {
    uint8_t magic[2];
    uint8_t version;
    uint8_t type;
    uint32_t sequence;
    uint32_t timestamp_us;
    uint16_t length;
    int8_t rssi;
    uint8_t channel;
    uint8_t rx_format;
    uint8_t flags;
} csi_frame_header_t;

typedef struct __attribute__((packed)) {
    uint8_t magic[2];
    uint8_t version;
    uint8_t type;
    uint32_t ping_expected;
    uint32_t ping_attempted;
    uint32_t ping_sent;
    uint32_t ping_send_failed;
    uint32_t ping_send_enomem;
    uint32_t ping_send_eagain;
    uint32_t ping_send_other;
    uint32_t ping_reply_unique;
    uint32_t ping_timeout;
    uint32_t ping_duration_us;
    uint32_t ping_interval_avg_us;
    uint32_t ping_interval_min_us;
    uint32_t ping_interval_max_us;
    uint32_t csi_received;
    uint32_t csi_processed;
    uint32_t csi_dropped;
    uint32_t csi_interval_avg_us;
    uint32_t csi_interval_min_us;
    uint32_t csi_interval_max_us;
} result_frame_t;

_Static_assert(sizeof(csi_frame_header_t) == 18, "CSI frame header size changed");
_Static_assert(sizeof(result_frame_t) == 80, "Result frame size changed");

static csi_slot_t s_csi_pool[CSI_POOL_SIZE];
static QueueHandle_t s_free_slots;
static QueueHandle_t s_ready_slots;
static TaskHandle_t s_result_task;

static int s_ping_socket = -1;
static struct sockaddr_in s_ping_target;
static uint8_t s_ping_reply_seen[(PING_COUNT + 7U) / 8U];

static volatile bool s_measurement_active;
static volatile bool s_receiver_active;
static volatile bool s_writer_busy;

static volatile uint32_t s_ping_attempted;
static volatile uint32_t s_ping_sent;
static volatile uint32_t s_ping_send_failed;
static volatile uint32_t s_ping_send_enomem;
static volatile uint32_t s_ping_send_eagain;
static volatile uint32_t s_ping_send_other;
static volatile uint32_t s_ping_reply_unique;
static volatile uint32_t s_ping_duration_us;
static volatile uint64_t s_ping_interval_total_us;
static volatile uint32_t s_ping_interval_count;
static volatile uint32_t s_ping_interval_min_us;
static volatile uint32_t s_ping_interval_max_us;

static volatile uint32_t s_csi_received;
static volatile uint32_t s_csi_processed;
static volatile uint32_t s_csi_dropped;
static volatile uint64_t s_csi_interval_total_us;
static volatile uint32_t s_csi_interval_count;
static volatile uint32_t s_csi_interval_min_us;
static volatile uint32_t s_csi_interval_max_us;
static volatile uint32_t s_last_csi_timestamp;

static inline void update_interval_stats(uint32_t interval,
                                         volatile uint64_t *total,
                                         volatile uint32_t *count,
                                         volatile uint32_t *minimum,
                                         volatile uint32_t *maximum)
{
    *total += interval;
    (*count)++;
    if (interval < *minimum) {
        *minimum = interval;
    }
    if (interval > *maximum) {
        *maximum = interval;
    }
}

static void reset_measurement_counters(void)
{
    s_ping_attempted = 0;
    s_ping_sent = 0;
    s_ping_send_failed = 0;
    s_ping_send_enomem = 0;
    s_ping_send_eagain = 0;
    s_ping_send_other = 0;
    s_ping_reply_unique = 0;
    s_ping_duration_us = 0;
    s_ping_interval_total_us = 0;
    s_ping_interval_count = 0;
    s_ping_interval_min_us = UINT32_MAX;
    s_ping_interval_max_us = 0;

    s_csi_received = 0;
    s_csi_processed = 0;
    s_csi_dropped = 0;
    s_csi_interval_total_us = 0;
    s_csi_interval_count = 0;
    s_csi_interval_min_us = UINT32_MAX;
    s_csi_interval_max_us = 0;
    s_last_csi_timestamp = 0;
    memset(s_ping_reply_seen, 0, sizeof(s_ping_reply_seen));
}

static void csi_rx_callback(void *ctx, wifi_csi_info_t *info)
{
    if (!s_measurement_active || info == NULL || info->buf == NULL) {
        return;
    }
    if (memcmp(info->mac, ctx, 6) != 0) {
        return;
    }

    const wifi_pkt_rx_ctrl_t *rx_ctrl = &info->rx_ctrl;
    const uint32_t sequence = s_csi_received++;
    const uint32_t timestamp = rx_ctrl->timestamp;

    if (s_last_csi_timestamp != 0) {
        update_interval_stats(timestamp - s_last_csi_timestamp,
                              &s_csi_interval_total_us,
                              &s_csi_interval_count,
                              &s_csi_interval_min_us,
                              &s_csi_interval_max_us);
    }
    s_last_csi_timestamp = timestamp;

    uint8_t slot_index;
    if (xQueueReceive(s_free_slots, &slot_index, 0) != pdTRUE) {
        s_csi_dropped++;
        return;
    }

    csi_slot_t *slot = &s_csi_pool[slot_index];
    slot->sequence = sequence;
    slot->timestamp_us = timestamp;
    slot->length = info->len > CSI_MAX_LEN ? CSI_MAX_LEN : info->len;
    slot->rssi = rx_ctrl->rssi;
    slot->channel = rx_ctrl->channel;
    slot->rx_format = rx_ctrl->cur_bb_format;
    slot->flags = info->first_word_invalid ? 1U : 0U;
    memcpy(slot->data, info->buf, slot->length);

    if (xQueueSend(s_ready_slots, &slot_index, 0) != pdTRUE) {
        s_csi_dropped++;
        xQueueSend(s_free_slots, &slot_index, 0);
    }
}

static void csi_writer_task(void *arg)
{
    uint8_t slot_index;
    while (true) {
        if (xQueueReceive(s_ready_slots, &slot_index, portMAX_DELAY) != pdTRUE) {
            continue;
        }

        s_writer_busy = true;
        csi_slot_t *slot = &s_csi_pool[slot_index];
        const csi_frame_header_t header = {
            .magic = {0xAA, 0x55},
            .version = PROTOCOL_VERSION,
            .type = CSI_FRAME_TYPE,
            .sequence = slot->sequence,
            .timestamp_us = slot->timestamp_us,
            .length = slot->length,
            .rssi = slot->rssi,
            .channel = slot->channel,
            .rx_format = slot->rx_format,
            .flags = slot->flags,
        };

        int header_bytes = uart_write_bytes(CSI_UART, &header, sizeof(header));
        int data_bytes = uart_write_bytes(CSI_UART, slot->data, slot->length);
        if (header_bytes == sizeof(header) && data_bytes == slot->length) {
            s_csi_processed++;
        } else {
            s_csi_dropped++;
        }

        xQueueSend(s_free_slots, &slot_index, portMAX_DELAY);
        s_writer_busy = false;
    }
}

static void wifi_csi_init(void)
{
    wifi_csi_config_t config = {
        .enable = true,
        .acquire_csi_legacy = false,
        .acquire_csi_force_lltf = false,
        .acquire_csi_ht20 = true,
        .acquire_csi_ht40 = false,
        .acquire_csi_vht = false,
        .acquire_csi_su = false,
        .acquire_csi_mu = false,
        .acquire_csi_dcm = false,
        .acquire_csi_beamformed = false,
        .acquire_csi_he_stbc_mode = 2,
        .val_scale_cfg = 0,
        .dump_ack_en = false,
        .reserved = false,
    };

    static wifi_ap_record_t ap_info;
    ESP_ERROR_CHECK(esp_wifi_sta_get_ap_info(&ap_info));
    ESP_ERROR_CHECK(esp_wifi_set_csi_config(&config));
    ESP_ERROR_CHECK(esp_wifi_set_csi_rx_cb(csi_rx_callback, ap_info.bssid));
    ESP_ERROR_CHECK(esp_wifi_set_csi(true));
}

static esp_err_t warm_up_gateway(const esp_ip4_addr_t *gateway)
{
    struct sockaddr_in destination = {
        .sin_family = AF_INET,
        .sin_port = htons(9),
        .sin_addr.s_addr = gateway->addr,
    };
    const uint8_t byte = 0;
    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    ESP_RETURN_ON_FALSE(sock >= 0, ESP_FAIL, TAG, "warm-up socket failed");

    ssize_t sent = sendto(sock, &byte, sizeof(byte), 0,
                          (const struct sockaddr *)&destination,
                          sizeof(destination));
    close(sock);
    ESP_RETURN_ON_FALSE(sent == sizeof(byte), ESP_FAIL, TAG, "warm-up send failed");
    vTaskDelay(pdMS_TO_TICKS(200));
    return ESP_OK;
}

static esp_err_t create_ping_socket(const esp_ip4_addr_t *gateway)
{
    s_ping_socket = socket(AF_INET, SOCK_RAW, IPPROTO_ICMP);
    ESP_RETURN_ON_FALSE(s_ping_socket >= 0, ESP_FAIL, TAG, "raw ICMP socket failed");

    int buffer_size = 65535;
    setsockopt(s_ping_socket, SOL_SOCKET, SO_RCVBUF, &buffer_size, sizeof(buffer_size));
    setsockopt(s_ping_socket, SOL_SOCKET, SO_SNDBUF, &buffer_size, sizeof(buffer_size));

    memset(&s_ping_target, 0, sizeof(s_ping_target));
    s_ping_target.sin_family = AF_INET;
    s_ping_target.sin_addr.s_addr = gateway->addr;
    return ESP_OK;
}

static bool mark_unique_reply(uint16_t sequence)
{
    if (sequence >= PING_COUNT) {
        return false;
    }
    const uint32_t byte_index = sequence / 8U;
    const uint8_t mask = (uint8_t)(1U << (sequence % 8U));
    if ((s_ping_reply_seen[byte_index] & mask) != 0) {
        return false;
    }
    s_ping_reply_seen[byte_index] |= mask;
    return true;
}

static void ping_receiver_task(void *arg)
{
    uint8_t buffer[128];
    while (s_receiver_active) {
        ssize_t received = recvfrom(s_ping_socket, buffer, sizeof(buffer), MSG_DONTWAIT,
                                    NULL, NULL);
        if (received < (ssize_t)(sizeof(struct ip_hdr) + sizeof(struct icmp_echo_hdr))) {
            vTaskDelay(pdMS_TO_TICKS(1));
            continue;
        }

        const struct ip_hdr *ip_header = (const struct ip_hdr *)buffer;
        const uint16_t ip_header_size = IPH_HL_BYTES(ip_header);
        if (received < (ssize_t)(ip_header_size + sizeof(struct icmp_echo_hdr))) {
            continue;
        }

        const struct icmp_echo_hdr *icmp =
            (const struct icmp_echo_hdr *)(buffer + ip_header_size);
        if (ICMPH_TYPE(icmp) != ICMP_ER || lwip_ntohs(icmp->id) != PING_ID) {
            continue;
        }

        uint16_t sequence = lwip_ntohs(icmp->seqno);
        if (mark_unique_reply(sequence)) {
            s_ping_reply_unique++;
        }
    }
    vTaskDelete(NULL);
}

static void prepare_echo_packet(uint8_t *packet, uint16_t sequence, uint32_t timestamp_us)
{
    const size_t packet_size = sizeof(struct icmp_echo_hdr) + PING_PAYLOAD_SIZE;
    memset(packet, 0, packet_size);

    struct icmp_echo_hdr *header = (struct icmp_echo_hdr *)packet;
    ICMPH_TYPE_SET(header, ICMP_ECHO);
    ICMPH_CODE_SET(header, 0);
    header->id = lwip_htons(PING_ID);
    header->seqno = lwip_htons(sequence);

    uint32_t *payload = (uint32_t *)(packet + sizeof(struct icmp_echo_hdr));
    payload[0] = timestamp_us;
    payload[1] = sequence;
    header->chksum = inet_chksum(packet, packet_size);
}

static void ping_sender_task(void *arg)
{
    uint8_t packet[sizeof(struct icmp_echo_hdr) + PING_PAYLOAD_SIZE];
    TickType_t last_wake = xTaskGetTickCount();
    int64_t measurement_start = esp_timer_get_time();
    int64_t previous_send_time = 0;

    reset_measurement_counters();
    s_receiver_active = true;
    s_measurement_active = true;

    for (uint32_t sequence = 0; sequence < PING_COUNT; sequence++) {
        if (sequence > 0) {
            vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(1));
        }

        int64_t now = esp_timer_get_time();
        if (previous_send_time != 0) {
            update_interval_stats((uint32_t)(now - previous_send_time),
                                  &s_ping_interval_total_us,
                                  &s_ping_interval_count,
                                  &s_ping_interval_min_us,
                                  &s_ping_interval_max_us);
        }
        previous_send_time = now;

        prepare_echo_packet(packet, (uint16_t)sequence,
                            (uint32_t)(now - measurement_start));
        s_ping_attempted++;
        ssize_t sent = sendto(s_ping_socket, packet, sizeof(packet), 0,
                              (const struct sockaddr *)&s_ping_target,
                              sizeof(s_ping_target));
        if (sent == sizeof(packet)) {
            s_ping_sent++;
        } else {
            s_ping_send_failed++;
            if (errno == ENOMEM) {
                s_ping_send_enomem++;
            } else if (errno == EAGAIN || errno == EWOULDBLOCK) {
                s_ping_send_eagain++;
            } else {
                s_ping_send_other++;
            }
        }
    }

    vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(1));
    s_measurement_active = false;
    s_ping_duration_us = (uint32_t)(esp_timer_get_time() - measurement_start);

    vTaskDelay(pdMS_TO_TICKS(PING_REPLY_GRACE_MS));
    s_receiver_active = false;
    xTaskNotifyGive(s_result_task);
    vTaskDelete(NULL);
}

static void result_task(void *arg)
{
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    TickType_t wait_start = xTaskGetTickCount();
    while ((uxQueueMessagesWaiting(s_ready_slots) > 0 || s_writer_busy) &&
           (xTaskGetTickCount() - wait_start) < pdMS_TO_TICKS(2000)) {
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    uart_wait_tx_done(CSI_UART, pdMS_TO_TICKS(2000));

    result_frame_t result = {
        .magic = {0xAA, 0x56},
        .version = PROTOCOL_VERSION,
        .type = RESULT_FRAME_TYPE,
        .ping_expected = PING_COUNT,
        .ping_attempted = s_ping_attempted,
        .ping_sent = s_ping_sent,
        .ping_send_failed = s_ping_send_failed,
        .ping_send_enomem = s_ping_send_enomem,
        .ping_send_eagain = s_ping_send_eagain,
        .ping_send_other = s_ping_send_other,
        .ping_reply_unique = s_ping_reply_unique,
        .ping_timeout = s_ping_sent - s_ping_reply_unique,
        .ping_duration_us = s_ping_duration_us,
        .ping_interval_avg_us = s_ping_interval_count
                                    ? (uint32_t)(s_ping_interval_total_us / s_ping_interval_count) : 0,
        .ping_interval_min_us = s_ping_interval_count ? s_ping_interval_min_us : 0,
        .ping_interval_max_us = s_ping_interval_max_us,
        .csi_received = s_csi_received,
        .csi_processed = s_csi_processed,
        .csi_dropped = s_csi_dropped,
        .csi_interval_avg_us = s_csi_interval_count
                                   ? (uint32_t)(s_csi_interval_total_us / s_csi_interval_count) : 0,
        .csi_interval_min_us = s_csi_interval_count ? s_csi_interval_min_us : 0,
        .csi_interval_max_us = s_csi_interval_max_us,
    };

    uart_write_bytes(CSI_UART, &result, sizeof(result));
    uart_wait_tx_done(CSI_UART, pdMS_TO_TICKS(2000));
    vTaskDelete(NULL);
}

static void init_uart(void)
{
    if (!uart_is_driver_installed(CSI_UART)) {
        const uart_config_t config = {
            .baud_rate = CSI_UART_BAUD_RATE,
            .data_bits = UART_DATA_8_BITS,
            .parity = UART_PARITY_DISABLE,
            .stop_bits = UART_STOP_BITS_1,
            .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
            .source_clk = UART_SCLK_DEFAULT,
        };
        ESP_ERROR_CHECK(uart_param_config(CSI_UART, &config));
        ESP_ERROR_CHECK(uart_driver_install(CSI_UART, 4096, CSI_UART_TX_BUFFER_SIZE,
                                            0, NULL, 0));
    }
    ESP_ERROR_CHECK(uart_set_baudrate(CSI_UART, CSI_UART_BAUD_RATE));
}

static void init_csi_pool(void)
{
    s_free_slots = xQueueCreate(CSI_POOL_SIZE, sizeof(uint8_t));
    s_ready_slots = xQueueCreate(CSI_POOL_SIZE, sizeof(uint8_t));
    ESP_ERROR_CHECK(s_free_slots && s_ready_slots ? ESP_OK : ESP_ERR_NO_MEM);

    for (uint8_t i = 0; i < CSI_POOL_SIZE; i++) {
        ESP_ERROR_CHECK(xQueueSend(s_free_slots, &i, 0) == pdTRUE
                            ? ESP_OK : ESP_ERR_NO_MEM);
    }
}

void app_main(void)
{
    esp_err_t nvs_error = nvs_flash_init();
    if (nvs_error == ESP_ERR_NVS_NO_FREE_PAGES ||
        nvs_error == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        nvs_error = nvs_flash_init();
    }
    ESP_ERROR_CHECK(nvs_error);
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    init_uart();

    ESP_ERROR_CHECK(example_connect());
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));

    esp_netif_t *sta = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
    ESP_ERROR_CHECK(sta ? ESP_OK : ESP_ERR_NOT_FOUND);
    esp_netif_ip_info_t ip_info;
    ESP_ERROR_CHECK(esp_netif_get_ip_info(sta, &ip_info));
    ESP_LOGI(TAG, "IP " IPSTR ", gateway " IPSTR,
             IP2STR(&ip_info.ip), IP2STR(&ip_info.gw));

    init_csi_pool();
    ESP_ERROR_CHECK(xTaskCreate(csi_writer_task, "csi_writer", 4096, NULL, 8, NULL)
                        == pdPASS ? ESP_OK : ESP_ERR_NO_MEM);
    ESP_ERROR_CHECK(xTaskCreate(result_task, "result", 3072, NULL, 6, &s_result_task)
                        == pdPASS ? ESP_OK : ESP_ERR_NO_MEM);

    wifi_csi_init();
    ESP_ERROR_CHECK(warm_up_gateway(&ip_info.gw));
    ESP_ERROR_CHECK(create_ping_socket(&ip_info.gw));

    /* No text logs may be emitted once the binary measurement stream starts. */
    esp_log_level_set("*", ESP_LOG_NONE);

    s_receiver_active = true;
    ESP_ERROR_CHECK(xTaskCreate(ping_receiver_task, "ping_rx", 3072, NULL, 10, NULL)
                        == pdPASS ? ESP_OK : ESP_ERR_NO_MEM);
    ESP_ERROR_CHECK(xTaskCreate(ping_sender_task, "ping_tx", 4096, NULL, 12, NULL)
                        == pdPASS ? ESP_OK : ESP_ERR_NO_MEM);
}
