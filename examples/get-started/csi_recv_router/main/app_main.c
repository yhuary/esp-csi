/*
 * SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
/* Get recv router csi

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"
#include "freertos/semphr.h"

#include "driver/uart.h"

#include "nvs_flash.h"

#include "esp_mac.h"
#include "rom/ets_sys.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_netif.h"
#include "esp_now.h"
#include "esp_event.h"
#include "esp_err.h"

#include "lwip/inet.h"
#include "lwip/netdb.h"
#include "lwip/sockets.h"
#include "esp_timer.h"
#include "lwip/def.h"
#include <errno.h>

#include "protocol_examples_common.h"
#include "esp_csi_gain_ctrl.h"

#define CONFIG_SEND_FREQUENCY 800// target ICMP Echo Requests per second
#if CONFIG_IDF_TARGET_ESP32C5 || CONFIG_IDF_TARGET_ESP32C61
#define CSI_FORCE_LLTF 0
#endif
#define CONFIG_FORCE_GAIN 0

#if CONFIG_IDF_TARGET_ESP32S3 || CONFIG_IDF_TARGET_ESP32C3 || CONFIG_IDF_TARGET_ESP32C5 || CONFIG_IDF_TARGET_ESP32C6 || CONFIG_IDF_TARGET_ESP32C61
#define CONFIG_GAIN_CONTROL 0
#endif

#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(6, 0, 0)
#define ESP_IF_WIFI_STA ESP_MAC_WIFI_STA
#endif

static const char *TAG = "csi_recv_router";
/*キューに入れる箱の形*/
#define CSI_MAX_LEN 512
#define CSI_QUEUE_SIZE 64

typedef struct
{
    int seq;            // CSIの通し番号(シーケンス番号)
    int len;            // raw CSI配列の要素数
    int8_t rssi;        // 受信状況確認(ほんとになる？)
    uint32_t timestamp; // CSI受信時のタイムスタンプ(マイクロ秒)

    uint8_t channel;           // CSIを受信したWi-Fiのプライマリチャネル番号
    uint8_t rx_format;         // 受信したWi-FiフレームのPHY形式（例：HT、VHT、HEなど）
    uint8_t secondary_channel; // セカンダリチャネルの位置（なし／上側／下側）
    uint8_t stbc;              // 空間時間ブロック符号の使用状況
    bool first_word_invalid;   // 最初の4バイトが無効かどうか

    int8_t data[CSI_MAX_LEN];
} csi_queue_item_t;


#define WIFI_MAXIMUM_RETRY 3

#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1

static EventGroupHandle_t s_wifi_event_group = NULL;
static int s_retry_num = 0;

// 後始末中に再接続しないためのフラグ
static bool s_wifi_shutting_down = false;

static QueueHandle_t csi_queue = NULL;


static uint32_t ping_reply_count = 0;
static uint32_t ping_timeout_count = 0;
static uint32_t csi_received_count = 0;
static uint32_t csi_processed_count = 0;
static uint32_t csi_dropped_count = 0;

/* QueueにCSIが来るまで待って、来たら item に取り出す*/
static void csi_process_task(void *arg)
{
    (void)arg;

    csi_queue_item_t item;
    uint8_t frame[8 + CSI_MAX_LEN];
    frame[0] = 0xAA;
    frame[1] = 0x55;

    while (1) {
        if (xQueueReceive(csi_queue, &item, portMAX_DELAY) == pdTRUE) {
            const uint32_t ts = item.timestamp;
            const uint16_t len = (uint16_t)item.len;
            // Explicit little-endian encoding, matching Python int.from_bytes(..., "little")
            frame[2] = (uint8_t)ts;
            frame[3] = (uint8_t)(ts >> 8);
            frame[4] = (uint8_t)(ts >> 16);
            frame[5] = (uint8_t)(ts >> 24);
            frame[6] = (uint8_t)len;
            frame[7] = (uint8_t)(len >> 8);
            memcpy(frame + 8, item.data, len);
            const int written = uart_write_bytes(UART_NUM_0, frame, 8 + len);
            if (written == (int)(8 + len)) {
                csi_processed_count++; // accepted by UART driver, not proof of PC receipt
            }
        }
    }
}

/*
 * 60秒後にCSIの受信・処理・ドロップ数を表示するTask
 */
static void __attribute__((unused)) csi_stats_task(void *arg)
{
    (void)arg;

    /*
     * 60秒待つ
     */
    vTaskDelay(pdMS_TO_TICKS(20000));

    /*
     * 60秒間の結果を1回だけ表示
     */
    ESP_LOGI(TAG, "===== 20 SEC RESULT =====");


    ESP_LOGI(TAG, "CSI received  : %lu",
             (unsigned long)csi_received_count);

    ESP_LOGI(TAG, "CSI processed : %lu",
             (unsigned long)csi_processed_count);

    ESP_LOGI(TAG, "CSI dropped   : %lu",
             (unsigned long)csi_dropped_count);

    ESP_LOGI(TAG, "Ping reply     : %lu",
            (unsigned long)ping_reply_count);

    ESP_LOGI(TAG, "Ping timeout   : %lu",
         (unsigned long)ping_timeout_count);

    ESP_LOGI(TAG, "Ping total     : %lu",
         (unsigned long)(ping_reply_count + ping_timeout_count));
         

    ESP_LOGI(TAG, "=========================");

    /*
     * このTask自身を終了
     */
    vTaskDelete(NULL);
}

static void wifi_csi_rx_cb(void *ctx, wifi_csi_info_t *info)//ctx 
{

    if (!info || !info->buf)
    {
        ESP_LOGW(TAG, "<%s> wifi_csi_cb", esp_err_to_name(ESP_ERR_INVALID_ARG));
        return;
    }

    if (memcmp(info->mac, ctx, 6))
    {
        return;
    }

    const wifi_pkt_rx_ctrl_t *rx_ctrl = &info->rx_ctrl;
    static int s_count = 0;
#if CONFIG_GAIN_CONTROL
    float compensate_gain = 1.0f;
    static uint8_t agc_gain = 0;
    static int8_t fft_gain = 0;
    static uint8_t agc_gain_baseline = 0;
    static int8_t fft_gain_baseline = 0;
    esp_csi_gain_ctrl_get_rx_gain(rx_ctrl, &agc_gain, &fft_gain);
    if (s_count < 100)
    {
        esp_csi_gain_ctrl_record_rx_gain(agc_gain, fft_gain);
    }
    else if (s_count == 100)
    {
        esp_csi_gain_ctrl_get_rx_gain_baseline(&agc_gain_baseline, &fft_gain_baseline);
#if CONFIG_FORCE_GAIN
        esp_csi_gain_ctrl_set_rx_force_gain(agc_gain_baseline, fft_gain_baseline);
        ESP_LOGI(TAG, "fft_force %d, agc_force %d", fft_gain_baseline, agc_gain_baseline);
#endif
    }
    esp_csi_gain_ctrl_get_gain_compensation(&compensate_gain, agc_gain, fft_gain);
    ESP_LOGD(TAG, "compensate_gain %f, agc_gain %d, fft_gain %d", compensate_gain, agc_gain, fft_gain);
#endif

    /*#if CONFIG_IDF_TARGET_ESP32C5 || CONFIG_IDF_TARGET_ESP32C6 || CONFIG_IDF_TARGET_ESP32C61
        if (!s_count) {
            ESP_LOGI(TAG, "================ CSI RECV ================");
            ets_printf("type,seq,mac,rssi,rate,noise_floor,fft_gain,agc_gain,channel,local_timestamp,sig_len,rx_format,len,first_word,data\n");
        }
        ets_printf("CSI_DATA,%d," MACSTR ",%d,%d,%d,%d,%d,%d,%d,%d,%d",
                   s_count, MAC2STR(info->mac), rx_ctrl->rssi, rx_ctrl->rate,
                   rx_ctrl->noise_floor, fft_gain, agc_gain, rx_ctrl->channel,
                   rx_ctrl->timestamp, rx_ctrl->sig_len, rx_ctrl->cur_bb_format);
    #else
        if (!s_count) {
            ESP_LOGI(TAG, "================ CSI RECV ================");
            ets_printf("type,id,mac,rssi,rate,sig_mode,mcs,bandwidth,smoothing,not_sounding,aggregation,stbc,fec_coding,sgi,noise_floor,ampdu_cnt,channel,secondary_channel,local_timestamp,ant,sig_len,rx_format,len,first_word,data\n");
        }
        ets_printf("CSI_DATA,%d," MACSTR ",%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d",
                   s_count, MAC2STR(info->mac), rx_ctrl->rssi, rx_ctrl->rate, rx_ctrl->sig_mode,
                   rx_ctrl->mcs, rx_ctrl->cwb, rx_ctrl->smoothing, rx_ctrl->not_sounding,
                   rx_ctrl->aggregation, rx_ctrl->stbc, rx_ctrl->fec_coding, rx_ctrl->sgi,
                   rx_ctrl->noise_floor, rx_ctrl->ampdu_cnt, rx_ctrl->channel, rx_ctrl->secondary_channel,
                   rx_ctrl->timestamp, rx_ctrl->ant, rx_ctrl->sig_len, rx_ctrl->sig_mode);
    #endif

#if (CONFIG_IDF_TARGET_ESP32C5 || CONFIG_IDF_TARGET_ESP32C61) && CSI_FORCE_LLTF

        int16_t csi = ((int16_t)(((((uint16_t)info->buf[1]) << 8) | info->buf[0]) << 4) >> 4);
        ets_printf(",%d,%d,\"[%d", (info->len - 2) / 2, info->first_word_invalid, (int16_t)(compensate_gain * csi));
        for (int i = 2; i < (info->len - 2); i += 2) {
            csi = ((int16_t)(((((uint16_t)info->buf[i + 1]) << 8) | info->buf[i]) << 4) >> 4);
            ets_printf(",%d", (int16_t)(compensate_gain * csi));
        }

    #else
        ets_printf(",%d,%d,\"[%d", info->len, info->first_word_invalid, (int16_t)(compensate_gain * info->buf[0]));
        for (int i = 1; i < info->len; i++) {
            ets_printf(",%d", (int16_t)(compensate_gain * info->buf[i]));
        }
    #endif
        ets_printf("]\"\n");*/

    csi_received_count++;

    csi_queue_item_t item;

    /*
     * CSIの基本情報をQueue用の構造体にコピー
     */

    item.seq = s_count;
    item.len = info->len;
    item.rssi = rx_ctrl->rssi;
    item.timestamp = rx_ctrl->timestamp;
    item.channel = rx_ctrl->channel;
    item.rx_format = rx_ctrl->cur_bb_format;

    item.secondary_channel = rx_ctrl->second;

    /*
    ESP32-C5はstbcメンバが無いので，HTフレームの場合はHT-SIGからSTBC情報を取り出す
    */
    item.stbc = 0;

    if (rx_ctrl->cur_bb_format == 2)
    {

        item.stbc = (rx_ctrl->he_siga1 >> 28) & 0x03;
    }

    item.first_word_invalid = info->first_word_invalid;

    /*
     * CSIデータが配列サイズを超えないようにする
     */
    if (item.len <= 0 || item.len > CSI_MAX_LEN) {
        csi_dropped_count++;
        return;
    }

    memcpy(item.data, info->buf, item.len);

    if (xQueueSend(csi_queue, &item, 0) != pdTRUE)
    {
        csi_dropped_count++;
    }

    s_count++;
}

static void wifi_csi_init(void)
{
    /**
     * @brief In order to ensure the compatibility of routers, only LLTF sub-carriers are selected.
     */
#if CONFIG_IDF_TARGET_ESP32C5 || CONFIG_IDF_TARGET_ESP32C61
    wifi_csi_config_t csi_config = {
        .enable = true,
        .acquire_csi_legacy = false,
        .acquire_csi_force_lltf = CSI_FORCE_LLTF,
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
        .reserved = false};
#elif CONFIG_IDF_TARGET_ESP32C6
    wifi_csi_config_t csi_config = {
        .enable = true,
        .acquire_csi_legacy = true,
        .acquire_csi_ht20 = true,
        .acquire_csi_ht40 = true,
        .acquire_csi_su = false,
        .acquire_csi_mu = false,
        .acquire_csi_dcm = false,
        .acquire_csi_beamformed = false,
        .acquire_csi_he_stbc = 2,
        .val_scale_cfg = false,
        .dump_ack_en = false,
        .reserved = false};
#else
    wifi_csi_config_t csi_config = {
        .lltf_en = true,
        .htltf_en = false,
        .stbc_htltf2_en = false,
        .ltf_merge_en = true,
        .channel_filter_en = true,
        .manu_scale = true,
        .shift = true,
    };
#endif
    static wifi_ap_record_t s_ap_info = {0};
    ESP_ERROR_CHECK(esp_wifi_sta_get_ap_info(&s_ap_info));
    ESP_ERROR_CHECK(esp_wifi_set_csi_config(&csi_config));
    ESP_ERROR_CHECK(esp_wifi_set_csi_rx_cb(wifi_csi_rx_cb, s_ap_info.bssid));
    ESP_ERROR_CHECK(esp_wifi_set_csi(true));
}


// ICMP Echo Request (IPv4), wire format. Separate send/receive tasks.
// The Python CSI binary framing remains unchanged.
#define ICMP_ECHO_REQUEST 8
#define ICMP_ECHO_REPLY   0
#define ICMP_IDENTIFIER  0xC512
#define ICMP_PACKET_LEN  16

static int icmp_sock = -1;
static struct sockaddr_in icmp_target;
static TaskHandle_t icmp_sender_handle = NULL;
static esp_timer_handle_t icmp_timer = NULL;
static volatile uint32_t icmp_send_ok_count = 0;
static volatile uint32_t icmp_send_error_count = 0;
static volatile uint32_t icmp_timer_missed_count = 0;

static uint16_t icmp_checksum(const uint8_t *data, size_t len)
{
    uint32_t sum = 0;
    while (len >= 2) {
        sum += ((uint16_t)data[0] << 8) | data[1];
        data += 2;
        len -= 2;
    }
    if (len) sum += ((uint16_t)data[0] << 8);
    while (sum >> 16) sum = (sum & 0xffff) + (sum >> 16);
    return (uint16_t)~sum;
}

static void icmp_tick_callback(void *arg)
{
    (void)arg;

    if (icmp_sender_handle != NULL) {
        xTaskNotifyGive(icmp_sender_handle);
    }
}

static void icmp_sender_task(void *arg)
{
    (void)arg;

    uint16_t sequence = 0;
    uint8_t packet[ICMP_PACKET_LEN] = {0};
    packet[0] = ICMP_ECHO_REQUEST;
    packet[4] = (uint8_t)(ICMP_IDENTIFIER >> 8);
    packet[5] = (uint8_t)ICMP_IDENTIFIER;
    // Small fixed payload; the CSI is taken from the AP's Echo Reply.
    for (int i = 8; i < ICMP_PACKET_LEN; ++i) packet[i] = (uint8_t)i;

    for (;;) {
        // Discard backlog instead of generating an artificial burst after stalls.
        uint32_t ticks = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        if (ticks > 1) icmp_timer_missed_count += ticks - 1;
        packet[6] = (uint8_t)(sequence >> 8);
        packet[7] = (uint8_t)sequence;
        sequence++;
        packet[2] = 0;
        packet[3] = 0;
        uint16_t checksum = icmp_checksum(packet, sizeof(packet));
        packet[2] = (uint8_t)(checksum >> 8);
        packet[3] = (uint8_t)checksum;

        int n = sendto(icmp_sock, packet, sizeof(packet), 0,
                       (struct sockaddr *)&icmp_target, sizeof(icmp_target));
        if (n == (int)sizeof(packet)) icmp_send_ok_count++;
        else icmp_send_error_count++;
    }
}

static void icmp_receiver_task(void *arg)
{
    (void)arg;

    uint8_t packet[160];
    for (;;) {
        struct sockaddr_in from;
        socklen_t fromlen = sizeof(from);
        int n = recvfrom(icmp_sock, packet, sizeof(packet), 0,
                         (struct sockaddr *)&from, &fromlen);
        if (n <= 0) continue;
        if (from.sin_addr.s_addr != icmp_target.sin_addr.s_addr) continue;

        // lwIP raw IPv4 sockets generally return an IP header before ICMP.
        // Accept both representations defensively.
        int offset = 0;
        if ((packet[0] >> 4) == 4) {
            offset = (packet[0] & 0x0f) * 4;
            if (offset < 20 || offset > n) continue;
        }
        if (n - offset < 8) continue;
        const uint8_t *icmp = packet + offset;
        if (icmp[0] != ICMP_ECHO_REPLY || icmp[1] != 0) continue;
        uint16_t id = ((uint16_t)icmp[4] << 8) | icmp[5];
        if (id == ICMP_IDENTIFIER) ping_reply_count++;
    }
}

static esp_err_t wifi_ping_router_start(void)
{
    esp_netif_t *netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
    esp_netif_ip_info_t local_ip;
    if (!netif || esp_netif_get_ip_info(netif, &local_ip) != ESP_OK) {
        return ESP_FAIL;
    }

    icmp_target.sin_family = AF_INET;
    icmp_target.sin_port = 0;
    icmp_target.sin_addr.s_addr = ip4_addr_get_u32(&local_ip.gw);
    icmp_sock = socket(AF_INET, SOCK_RAW, IPPROTO_ICMP);
    if (icmp_sock < 0) {
        ESP_LOGE(TAG, "ICMP raw socket failed: errno=%d", errno);
        return ESP_FAIL;
    }

    // The timer callback only notifies; network I/O runs in a normal task.
    if (xTaskCreate(icmp_receiver_task, "icmp_receiver", 3072, NULL, 6, NULL) != pdPASS ||
        xTaskCreate(icmp_sender_task, "icmp_sender", 3072, NULL, 6,
                    &icmp_sender_handle) != pdPASS) {
        close(icmp_sock);
        icmp_sock = -1;
        return ESP_ERR_NO_MEM;
    }
    const esp_timer_create_args_t timer_args = {
        .callback = &icmp_tick_callback,
        .arg = NULL,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "icmp_tick",
    };
    ESP_ERROR_CHECK(esp_timer_create(&timer_args, &icmp_timer));
    // 1000 Hz -> 1000 us; independent of Echo Reply arrival.
    ESP_ERROR_CHECK(esp_timer_start_periodic(icmp_timer,
                                            1000000ULL / CONFIG_SEND_FREQUENCY));
    return ESP_OK;
}

static void my_wifi_got_ip_handler(
    void *arg,
    esp_event_base_t event_base,
    int32_t event_id,
    void *event_data)
{
    (void)arg;
    (void)event_base;
    (void)event_id;
    (void)event_data;

    ESP_LOGI(TAG, "IP address acquired");

    s_retry_num = 0;

    xEventGroupClearBits(
        s_wifi_event_group,
        WIFI_FAIL_BIT
    );

    xEventGroupSetBits(
        s_wifi_event_group,
        WIFI_CONNECTED_BIT
    );
}

static void my_wifi_disconnected_handler(
    void *arg,
    esp_event_base_t event_base,
    int32_t event_id,
    void *event_data)
{
    (void)arg;
    (void)event_base;
    (void)event_id;
    (void)event_data;

    ESP_LOGW(TAG, "Wi-Fi disconnected");

    // 接続済みの状態を解除
    xEventGroupClearBits(
        s_wifi_event_group,
        WIFI_CONNECTED_BIT
    );

    // 後始末中なら再接続しない
    if (s_wifi_shutting_down) {
        return;
    }

    // 最大3回まで再接続
    if (s_retry_num < WIFI_MAXIMUM_RETRY) {

        s_retry_num++;

        ESP_LOGW(TAG,
                 "Retry Wi-Fi connection: %d/%d",
                 s_retry_num,
                 WIFI_MAXIMUM_RETRY);

        esp_err_t err = esp_wifi_connect();

        if (err == ESP_OK) {
            return;
        }

        ESP_LOGE(TAG,
                 "Reconnect request failed: %s",
                 esp_err_to_name(err));
    }

    // 再接続できない場合は失敗を通知
    xEventGroupSetBits(
        s_wifi_event_group,
        WIFI_FAIL_BIT
    );
}

static esp_err_t my_wifi_connect(void)
{
    esp_err_t err = ESP_OK;
    bool wifi_initialized = false;
    bool wifi_started = false;

    esp_event_handler_instance_t got_ip_instance = NULL;
    esp_event_handler_instance_t disconnected_instance = NULL;

    s_retry_num = 0;
    s_wifi_shutting_down = false;

    // ① STA用ネットワークインターフェース作成
    esp_netif_t *sta_netif =
        esp_netif_create_default_wifi_sta();

    if (sta_netif == NULL) {
        return ESP_ERR_NO_MEM;
    }

    // ② Event Group作成
    s_wifi_event_group = xEventGroupCreate();

    if (s_wifi_event_group == NULL) {
        err = ESP_ERR_NO_MEM;
        goto cleanup;
    }

    // ③ Wi-Fiドライバ初期化
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();

    err = esp_wifi_init(&cfg);
    if (err != ESP_OK) {
        goto cleanup;
    }
    wifi_initialized = true;

    // ④ STAモード設定
    err = esp_wifi_set_mode(WIFI_MODE_STA);
    if (err != ESP_OK) {
        goto cleanup;
    }

    // ⑤ イベントハンドラ登録
    err = esp_event_handler_instance_register(
        IP_EVENT,
        IP_EVENT_STA_GOT_IP,
        &my_wifi_got_ip_handler,
        NULL,
        &got_ip_instance
    );
    if (err != ESP_OK) {
        goto cleanup;
    }

    err = esp_event_handler_instance_register(
        WIFI_EVENT,
        WIFI_EVENT_STA_DISCONNECTED,
        &my_wifi_disconnected_handler,
        NULL,
        &disconnected_instance
    );
    if (err != ESP_OK) {
        goto cleanup;
    }

    // ⑥ Wi-Fiドライバ起動
    err = esp_wifi_start();
    if (err != ESP_OK) {
        goto cleanup;
    }
    wifi_started = true;

    // ⑦ 使用するWi-Fi規格を設定
    wifi_protocols_t protocols = {
        .ghz_2g = WIFI_PROTOCOL_11B |
                  WIFI_PROTOCOL_11G |
                  WIFI_PROTOCOL_11N,

        .ghz_5g = WIFI_PROTOCOL_11A |
                  WIFI_PROTOCOL_11N
    };

    err = esp_wifi_set_protocols(WIFI_IF_STA, &protocols);
    if (err != ESP_OK) {
        goto cleanup;
    }

    // ⑧ SSID・パスワード設定
    wifi_config_t wifi_config = {
        .sta = {
            .ssid = CONFIG_EXAMPLE_WIFI_SSID,
            .password = CONFIG_EXAMPLE_WIFI_PASSWORD,
        },
    };

    err = esp_wifi_set_config(WIFI_IF_STA, &wifi_config);
    if (err != ESP_OK) {
        goto cleanup;
    }

    // ⑨ APへの接続開始
    err = esp_wifi_connect();
    if (err != ESP_OK) {
        goto cleanup;
    }

    // ⑩ 接続成功・失敗の通知を待つ
    EventBits_t bits = xEventGroupWaitBits(
        s_wifi_event_group,
        WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
        pdFALSE,
        pdFALSE,
        pdMS_TO_TICKS(10000)
    );

    if (bits & WIFI_FAIL_BIT) {
        ESP_LOGE(TAG, "Wi-Fi connection failed");
        err = ESP_FAIL;
        goto cleanup;
    }

    if (!(bits & WIFI_CONNECTED_BIT)) {
        ESP_LOGE(TAG, "Wi-Fi connection timeout");
        err = ESP_ERR_TIMEOUT;
        goto cleanup;
    }

    ESP_LOGI(TAG, "Wi-Fi connected and IP acquired");

    // 成功時はイベント登録とEvent Groupを残す
    return ESP_OK;

cleanup:
    ESP_LOGE(TAG,
             "Wi-Fi setup failed: %s",
             esp_err_to_name(err));

    // 後始末中の再接続を禁止
    s_wifi_shutting_down = true;

    // Wi-Fiを停止
    if (wifi_started) {
        esp_wifi_stop();
    }

    // イベントハンドラ登録を解除
    if (disconnected_instance != NULL) {
        esp_event_handler_instance_unregister(
            WIFI_EVENT,
            WIFI_EVENT_STA_DISCONNECTED,
            disconnected_instance
        );
    }

    if (got_ip_instance != NULL) {
        esp_event_handler_instance_unregister(
            IP_EVENT,
            IP_EVENT_STA_GOT_IP,
            got_ip_instance
        );
    }

    // Wi-Fiドライバの後始末
    if (wifi_initialized) {
        esp_wifi_deinit();
    }

    // Event Group削除
    if (s_wifi_event_group != NULL) {
        vEventGroupDelete(s_wifi_event_group);
        s_wifi_event_group = NULL;
    }

    // ネットワークインターフェース削除
    esp_netif_destroy_default_wifi(sta_netif);

    return err;
}

void app_main(void)
{
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

        // UART0ドライバが未インストールの場合だけ初期化
    if (!uart_is_driver_installed(UART_NUM_0))
    {
        uart_config_t uart_config = {
            .baud_rate = 2000000,
            .data_bits = UART_DATA_8_BITS,
            .parity = UART_PARITY_DISABLE,
            .stop_bits = UART_STOP_BITS_1,
            .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
            .source_clk = UART_SCLK_DEFAULT,
        };

        ESP_ERROR_CHECK(uart_param_config(UART_NUM_0, &uart_config));

        ESP_ERROR_CHECK(uart_driver_install(
            UART_NUM_0,
            4096,
            4096,
            0,
            NULL,
            0
        ));
    }
    // Apply baud rate even when UART0 was already installed by the console.
    ESP_ERROR_CHECK(uart_set_baudrate(UART_NUM_0, 2000000));

    /**
     * @brief This helper function configures Wi-Fi, as selected in menuconfig.
     *        Read "Establishing Wi-Fi Connection" section in esp-idf/examples/protocols/README.md
     *        for more information about this function.
     */
    ESP_ERROR_CHECK(my_wifi_connect());


    /*
     * CSIデータを一時的に入れるQueueを作成
     */
    csi_queue = xQueueCreate(
        CSI_QUEUE_SIZE,
        sizeof(csi_queue_item_t));

    if (csi_queue == NULL)
    {
        ESP_LOGE(TAG, "Failed to create CSI queue");
        return;
    }

    /*
     * QueueからCSIを取り出すTaskを作成
     */
    BaseType_t task_result = xTaskCreate(
        csi_process_task,   // 実行する関数
        "csi_process_task", // Task名
        4096,               // Stackサイズ
        NULL,               // 引数
        5,                  // Priority
        NULL                // Task handle
    );

    if (task_result != pdPASS)
    {
        ESP_LOGE(TAG, "Failed to create CSI task");
        return;
    }

    // Do not start stats logger: its text would corrupt the binary UART stream.
    wifi_csi_init();
    ESP_ERROR_CHECK(wifi_ping_router_start());
}
