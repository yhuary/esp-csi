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

#include "nvs_flash.h"

#include "esp_mac.h"
#include "rom/ets_sys.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_netif.h"
#include "esp_now.h"

#include "lwip/inet.h"
#include "lwip/netdb.h"
#include "lwip/sockets.h"
#include "ping/ping_sock.h"

#include "protocol_examples_common.h"
#include "esp_csi_gain_ctrl.h"

#define CONFIG_SEND_FREQUENCY 100
#if CONFIG_IDF_TARGET_ESP32C5 || CONFIG_IDF_TARGET_ESP32C61
#define CSI_FORCE_LLTF 0
#endif
#define CONFIG_FORCE_GAIN 0

#if CONFIG_IDF_TARGET_ESP32S3 || CONFIG_IDF_TARGET_ESP32C3 || CONFIG_IDF_TARGET_ESP32C5 || CONFIG_IDF_TARGET_ESP32C6 || CONFIG_IDF_TARGET_ESP32C61
#define CONFIG_GAIN_CONTROL 1
#endif

#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(6, 0, 0)
#define ESP_IF_WIFI_STA ESP_MAC_WIFI_STA
#endif

static const char *TAG = "csi_recv_router";
/*キューに入れる箱の形*/
#define CSI_MAX_LEN 512
#define CSI_QUEUE_SIZE 20

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

static QueueHandle_t csi_queue = NULL;

static uint32_t csi_received_count = 0;
static uint32_t csi_processed_count = 0;
static uint32_t csi_dropped_count = 0;

/* QueueにCSIが来るまで待って、来たら item に取り出す*/
static void csi_process_task(void *arg)
{
    csi_queue_item_t item;

    while (1)
    {

        if (xQueueReceive(csi_queue, &item, portMAX_DELAY) == pdTRUE)
        {

            csi_processed_count++;

            /*
             * 1サンプル分の基本情報
             *
             * 出力形式：
             * CSI_DATA,seq,timestamp,rssi,len,"[CSIデータ]"
             */
            ets_printf(
                "CSI_DATA,%d,%lu,%d,%d,%d,%d,%d,%d,%d,\"[",
                item.seq,
                (unsigned long)item.timestamp,
                item.rssi,
                item.len,
                item.channel,
                item.rx_format,
                item.secondary_channel,
                item.stbc,
                item.first_word_invalid);

            /*
             * CSI raw dataを順番に出力
             */
            for (int i = 0; i < item.len; i++)
            {

                if (i == 0)
                {
                    ets_printf("%d", item.data[i]);
                }
                else
                {
                    ets_printf(",%d", item.data[i]);
                }
            }

            /*
             * 1サンプル終了
             */
            ets_printf("]\"\n");
        }
    }
}

/*
 * 60秒後にCSIの受信・処理・ドロップ数を表示するTask
 */
static void csi_stats_task(void *arg)
{
    /*
     * 60秒待つ
     */
    vTaskDelay(pdMS_TO_TICKS(60000));

    /*
     * 60秒間の結果を1回だけ表示
     */
    ESP_LOGI(TAG, "===== 60 SEC RESULT =====");

    ESP_LOGI(TAG, "CSI received  : %lu",
             (unsigned long)csi_received_count);

    ESP_LOGI(TAG, "CSI processed : %lu",
             (unsigned long)csi_processed_count);

    ESP_LOGI(TAG, "CSI dropped   : %lu",
             (unsigned long)csi_dropped_count);

    ESP_LOGI(TAG, "=========================");

    /*
     * このTask自身を終了
     */
    vTaskDelete(NULL);
}

static void wifi_csi_rx_cb(void *ctx, wifi_csi_info_t *info)
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
    float compensate_gain = 1.0f;
    static uint8_t agc_gain = 0;
    static int8_t fft_gain = 0;
#if CONFIG_GAIN_CONTROL
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
    if (item.len > CSI_MAX_LEN)
    {
        item.len = CSI_MAX_LEN;
    }

    memcpy(item.data, info->buf, item.len);

    if (xQueueSend(csi_queue, &item, 0) != pdTRUE)
    {
        csi_dropped_count++;
    }

    s_count++;
}

static void wifi_csi_init()
{
    /**
     * @brief In order to ensure the compatibility of routers, only LLTF sub-carriers are selected.
     */
#if CONFIG_IDF_TARGET_ESP32C5 || CONFIG_IDF_TARGET_ESP32C61
    wifi_csi_config_t csi_config = {
        .enable = true,
        .acquire_csi_legacy = true,
        .acquire_csi_force_lltf = CSI_FORCE_LLTF,
        .acquire_csi_ht20 = true,
        .acquire_csi_ht40 = true,
        .acquire_csi_vht = true,
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

static esp_err_t wifi_ping_router_start()
{
    static esp_ping_handle_t ping_handle = NULL;

    esp_ping_config_t ping_config = ESP_PING_DEFAULT_CONFIG();
    ping_config.count = 0;
    ping_config.interval_ms = 1000 / CONFIG_SEND_FREQUENCY;
    ping_config.task_stack_size = 3072;
    ping_config.data_size = 1;

    esp_netif_ip_info_t local_ip;
    esp_netif_get_ip_info(esp_netif_get_handle_from_ifkey("WIFI_STA_DEF"), &local_ip);
    ESP_LOGI(TAG, "got ip:" IPSTR ", gw: " IPSTR, IP2STR(&local_ip.ip), IP2STR(&local_ip.gw));
    ping_config.target_addr.u_addr.ip4.addr = ip4_addr_get_u32(&local_ip.gw);
    ping_config.target_addr.type = ESP_IPADDR_TYPE_V4;

    esp_ping_callbacks_t cbs = {0};
    esp_ping_new_session(&ping_config, &cbs, &ping_handle);
    esp_ping_start(ping_handle);

    return ESP_OK;
}

void app_main()
{
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    /**
     * @brief This helper function configures Wi-Fi, as selected in menuconfig.
     *        Read "Establishing Wi-Fi Connection" section in esp-idf/examples/protocols/README.md
     *        for more information about this function.
     */
    ESP_ERROR_CHECK(example_connect());

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

    /*
     * 60秒後に結果を表示するTaskを作成
     */
    xTaskCreate(
        csi_stats_task,
        "csi_stats_task",
        2048,
        NULL,
        4,
        NULL);

    wifi_csi_init();
    wifi_ping_router_start();
}