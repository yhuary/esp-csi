/* 
 * SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD 
 * SPDX-License-Identifier: Apache-2.0 
 */

/*
 * ============================================================
 * このプログラム全体の流れ
 * ============================================================
 *
 * app_main()
 *   ↓
 * ① example_connect()
 *      ESP32をWi-Fiルータへ接続
 *
 *   ↓
 * ② wifi_csi_init()
 *      CSI取得条件を設定
 *      CSI受信時に呼ばれる関数 wifi_csi_rx_cb() を登録
 *      CSI取得を有効化
 *
 *   ↓
 * ③ wifi_ping_router_start()
 *      ESP32 → ルータへpingを継続的に送信
 *
 *
 * 【超重要】
 *
 * pingを送った関数から直接CSI_DATAを出力しているわけではない。
 *
 * Wi-Fiパケットを受信
 *        ↓
 * Wi-FiドライバがCSIを取得
 *        ↓
 * 登録しておいた wifi_csi_rx_cb() が呼ばれる
 *        ↓
 * その中で ets_printf("CSI_DATA,...") を実行
 *
 * という構造。
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

#include "nvs_flash.h"

#include "esp_mac.h"
#include "rom/ets_sys.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_netif.h"
#include "esp_timer.h"
#include "esp_now.h"

#include "lwip/inet.h"
#include "lwip/netdb.h"
#include "lwip/sockets.h"

/*
 * ESP-IDFのping機能
 *
 * 後半の wifi_ping_router_start() で使用。
 */
#include "ping/ping_sock.h"

#include "protocol_examples_common.h"
#include "esp_csi_gain_ctrl.h"


/*
 * 1秒あたり何回pingするか。
 *
 * 1000なので、後で
 *
 * interval_ms = 1000 / 1000 = 1 ms
 *
 * つまり1ms間隔でpingを開始する設定になる。
 */
#define CONFIG_SEND_FREQUENCY 1000 // 1000 ping/s = 1ms間隔 (ここで設定してんのは周波数)   


#if CONFIG_IDF_TARGET_ESP32C5 || CONFIG_IDF_TARGET_ESP32C61
#define CSI_FORCE_LLTF 0
#endif

#define CONFIG_FORCE_GAIN 0


#if CONFIG_IDF_TARGET_ESP32S3 || CONFIG_IDF_TARGET_ESP32C3 || \
    CONFIG_IDF_TARGET_ESP32C5 || CONFIG_IDF_TARGET_ESP32C6 || \
    CONFIG_IDF_TARGET_ESP32C61

#define CONFIG_GAIN_CONTROL 1
#endif


#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(6, 0, 0)
#define ESP_IF_WIFI_STA ESP_MAC_WIFI_STA
#endif


static const char *TAG = "csi_recv_router";

/*
 * ============================================================
 * pingの実際の成功回数を数える
 * ============================================================
 */
static volatile uint32_t ping_success_count = 0;

static esp_ping_handle_t ping_handle = NULL;


/*
 * ============================================================
 * ★★★★★ 最重要部分 ★★★★★
 *
 * CSIを受信したときに呼ばれるcallback関数
 * ============================================================
 *
 * この関数を自分で直接呼んでいる場所はない。
 *
 * 後の
 *
 * esp_wifi_set_csi_rx_cb(wifi_csi_rx_cb, ...)
 *
 * によってWi-Fiドライバへ登録される。
 *
 * Wi-Fiドライバ側でCSIが取得されると、
 *
 * wifi_csi_rx_cb(...)
 *
 * が呼ばれる。
 *
 * したがって、
 *
 * ★ CSI_DATAが出るタイミング
 *      =
 *   このcallbackが呼ばれたタイミング
 *
 * である。
 */
static void wifi_csi_rx_cb(void *ctx, wifi_csi_info_t *info)
{

    /*
     * CSI情報が存在しない場合は何もしない。
     */
    if (!info || !info->buf) {
        ESP_LOGW(TAG,
                 "<%s> wifi_csi_cb",
                 esp_err_to_name(ESP_ERR_INVALID_ARG));
        return;
    }


    /*
     * ========================================================
     * ★ かなり重要
     * ========================================================
     *
     * info->mac
     *      = CSIを取得した受信パケットの送信元MAC
     *
     * ctx
     *      = 後でcallback登録時に渡しているAPのBSSID
     *
     * つまり
     *
     *      送信元MAC == APのBSSID
     *
     * のパケットだけ残す。
     *
     * AP以外から届いたパケットについては
     *
     * return;
     *
     * しているのでCSI_DATAを出力しない。
     */
    if (memcmp(info->mac, ctx, 6)) {
        return;
    }


    /*
     * CSIを取得したWi-Fiパケットの受信情報。
     *
     * ここに
     *
     * RSSI
     * rate
     * sig_mode
     * mcs
     * cwb
     * stbc
     * channel
     * timestamp
     *
     * などが入っている。
     */
    const wifi_pkt_rx_ctrl_t *rx_ctrl = &info->rx_ctrl;


    /*
     * CSI_DATAが何個出たか数えるカウンタ。
     *
     * ★ ping回数ではない。
     *
     * このcallbackで条件を通過して
     * CSI_DATAを出力した回数。
     */
    static int s_count = 0;

    // ===== 60秒間だけCSIを記録する =====
    static int64_t start_time = 0;
    static bool finished = false;

    int64_t now = esp_timer_get_time();

    // APからの最初のCSIを受信した時刻を開始時刻にする
    if (start_time == 0) {
        start_time = now;
        ESP_LOGI(TAG, "===== START CSI RECORDING : 60 sec =====");
    }

    // 60秒経過したらCSI_DATAの出力を停止
if (now - start_time >= 60LL * 1000000LL) {

    if (!finished) {

        uint32_t ping_requests = 0;
        uint32_t ping_replies = 0;

        /* 実際に送信したICMP Echo Request数 */
        esp_ping_get_profile(
            ping_handle,
            ESP_PING_PROF_REQUEST,
            &ping_requests,
            sizeof(ping_requests)
        );

        /* 実際に受信したICMP Echo Reply数 */
        esp_ping_get_profile(
            ping_handle,
            ESP_PING_PROF_REPLY,
            &ping_replies,
            sizeof(ping_replies)
        );

        ESP_LOGI(TAG,
                 "===== FINISH CSI RECORDING : 60 sec =====");

        ESP_LOGI(TAG,
                 "ICMP Echo Requests = %lu",
                 (unsigned long)ping_requests);

        ESP_LOGI(TAG,
                 "ICMP Echo Replies  = %lu",
                 (unsigned long)ping_replies);

        ESP_LOGI(TAG,
                 "CSI samples   = %d",
                 s_count);

        ESP_LOGI(TAG,
                 "ICMP Echo Requests rate = %.2f Hz",
                 (float)ping_requests / 60.0f);

        ESP_LOGI(TAG,
                 "ICMP Echo Replies rate   = %.2f Hz",
                 (float)ping_replies / 60.0f);

        ESP_LOGI(TAG,
                 "CSI sample rate   = %.2f Hz",
                 (float)s_count / 60.0f);

        finished = true;
    }

    return;
}

    float compensate_gain = 1.0f;
    static uint8_t agc_gain = 0;
    static int8_t fft_gain = 0;


#if CONFIG_GAIN_CONTROL

    static uint8_t agc_gain_baseline = 0;
    static int8_t fft_gain_baseline = 0;

    /*
     * CSI受信時の受信ゲイン情報を取得。
     *
     * CSI_DATAが出る/出ないを決める中心処理ではない。
     */
    esp_csi_gain_ctrl_get_rx_gain(
        rx_ctrl,
        &agc_gain,
        &fft_gain
    );


    /*
     * 最初の100 CSIについてゲインを記録。
     */
    if (s_count < 100) {

        esp_csi_gain_ctrl_record_rx_gain(
            agc_gain,
            fft_gain
        );

    } else if (s_count == 100) {

        esp_csi_gain_ctrl_get_rx_gain_baseline(
            &agc_gain_baseline,
            &fft_gain_baseline
        );

#if CONFIG_FORCE_GAIN

        esp_csi_gain_ctrl_set_rx_force_gain(
            agc_gain_baseline,
            fft_gain_baseline
        );

        ESP_LOGI(TAG,
                 "fft_force %d, agc_force %d",
                 fft_gain_baseline,
                 agc_gain_baseline);
#endif
    }


    /*
     * ゲイン補正値を計算。
     */
    esp_csi_gain_ctrl_get_gain_compensation(
        &compensate_gain,
        agc_gain,
        fft_gain
    );

#endif



#if CONFIG_IDF_TARGET_ESP32C5 || \
    CONFIG_IDF_TARGET_ESP32C6 || \
    CONFIG_IDF_TARGET_ESP32C61

    /*
     * 最初のCSIだけヘッダー表示。
     */
    if (!s_count) {

        ESP_LOGI(TAG,
                 "================ CSI RECV ================");

        ets_printf(
            "type,seq,mac,rssi,rate,noise_floor,"
            "fft_gain,agc_gain,channel,"
            "local_timestamp,sig_len,rx_format,"
            "len,first_word,data\n"
        );
    }


    /*
     * ========================================================
     * ★★★★★ CSI_DATAが実際に出る場所 ★★★★★
     * ========================================================
     *
     * この行が実行された時点で
     *
     * CSI_DATA,0,...
     * CSI_DATA,1,...
     * CSI_DATA,2,...
     *
     * がシリアルへ出力される。
     *
     * つまりCSI_DATAの出力は
     *
     * ping送信関数
     *
     * ではなく
     *
     * wifi_csi_rx_cb()
     *
     * の中にある。
     */

     ESP_LOGI(TAG,
         "CSI CHECK: rx_format=%d, second=%d, len=%d, valid=%d",
         rx_ctrl->cur_bb_format,
         rx_ctrl->second,
         info->len,
         rx_ctrl->rx_channel_estimate_info_vld);
         
    ets_printf(
        "CSI_DATA,%d," MACSTR ",%d,%d,%d,%d,%d,%d,%d,%d,%d",

        s_count,

        MAC2STR(info->mac),

        rx_ctrl->rssi,
        rx_ctrl->rate,
        rx_ctrl->noise_floor,

        fft_gain,
        agc_gain,

        rx_ctrl->channel,
        rx_ctrl->timestamp,
        rx_ctrl->sig_len,
        rx_ctrl->cur_bb_format
    );


#else

    /*
     * ========================================================
     * ★ あなたの通常ESP32はこちら
     * ========================================================
     *
     * ESP32-C5/C6/C61ではない場合はこちらへ来る。
     */

    if (!s_count) {

        ESP_LOGI(TAG,
                 "================ CSI RECV ================");

        /*
         * CSI_DATAのCSVヘッダー。
         */
        ets_printf(
            "type,id,mac,rssi,rate,sig_mode,mcs,"
            "bandwidth,smoothing,not_sounding,"
            "aggregation,stbc,fec_coding,sgi,"
            "noise_floor,ampdu_cnt,channel,"
            "secondary_channel,local_timestamp,"
            "ant,sig_len,rx_format,"
            "len,first_word,data\n"
        );
    }


    /*
     * ========================================================
     * ★★★★★ ESP32でCSI_DATAが出る場所 ★★★★★
     * ========================================================
     */
    ets_printf(
        "CSI_DATA,%d," MACSTR
        ",%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,"
        "%d,%d,%d,%d,%d,%d,%d,%d",

        /*
         * CSI番号
         */
        s_count,

        /*
         * CSIを取得したパケットの送信元MAC
         */
        MAC2STR(info->mac),

        /*
         * 以下は受信したWi-Fiパケットの情報
         */
        rx_ctrl->rssi,
        rx_ctrl->rate,
        rx_ctrl->sig_mode,
        rx_ctrl->mcs,
        rx_ctrl->cwb,
        rx_ctrl->smoothing,
        rx_ctrl->not_sounding,
        rx_ctrl->aggregation,
        rx_ctrl->stbc,
        rx_ctrl->fec_coding,
        rx_ctrl->sgi,
        rx_ctrl->noise_floor,
        rx_ctrl->ampdu_cnt,
        rx_ctrl->channel,
        rx_ctrl->secondary_channel,
        rx_ctrl->timestamp,
        rx_ctrl->ant,
        rx_ctrl->sig_len,
        rx_ctrl->sig_mode
    );

#endif



/*
 * ============================================================
 * ここからCSI本体 info->buf[] を出力
 * ============================================================
 */

#if (CONFIG_IDF_TARGET_ESP32C5 || \
     CONFIG_IDF_TARGET_ESP32C61) && CSI_FORCE_LLTF

    /*
     * C5/C61の特殊処理。
     */
    int16_t csi =
        ((int16_t)(((((uint16_t)info->buf[1]) << 8)
        | info->buf[0]) << 4) >> 4);

    ets_printf(
        ",%d,%d,\"[%d",
        (info->len - 2) / 2,
        info->first_word_invalid,
        (int16_t)(compensate_gain * csi)
    );

    for (int i = 2;
         i < (info->len - 2);
         i += 2) {

        csi =
            ((int16_t)(((((uint16_t)info->buf[i + 1]) << 8)
            | info->buf[i]) << 4) >> 4);

        ets_printf(
            ",%d",
            (int16_t)(compensate_gain * csi)
        );
    }

#else

    /*
     * ========================================================
     * ★ 通常ESP32ではここ
     * ========================================================
     *
     * info->len
     *      CSIデータ長
     *
     * info->first_word_invalid
     *      CSI先頭データが有効かどうか
     *
     * info->buf[]
     *      実際のCSIデータ
     */

    ets_printf(
        ",%d,%d,\"[%d",
        info->len,
        info->first_word_invalid,
        (int16_t)(compensate_gain * info->buf[0])
    );


    /*
     * CSIデータを最後まで順番に出力。
     */
    for (int i = 1; i < info->len; i++) {

        ets_printf(
            ",%d",
            (int16_t)(compensate_gain * info->buf[i])
        );
    }

#endif


    /*
     * CSI_DATA 1行終了。
     */
    ets_printf("]\"\n");


    /*
     * ========================================================
     * ★ CSI_DATAを1行出したのでカウンタ+1
     * ========================================================
     *
     * これも重要。
     *
     * s_countは
     *
     * 「送ったpingの数」
     *
     * ではなく
     *
     * 「このcallbackで実際にCSI_DATAを出力した数」
     *
     * である。
     */
    s_count++;
}



/*
 * ============================================================
 * CSI取得の初期設定
 * ============================================================
 */
static void wifi_csi_init()
{

    /*
     * コメントには
     *
     * "ルータとの互換性を確保するため
     *  LLTF sub-carriersだけを選択"
     *
     * と書かれている。
     */


#if CONFIG_IDF_TARGET_ESP32C5 || \
    CONFIG_IDF_TARGET_ESP32C61

    /*
     * C5/C61用CSI設定
     */
    wifi_csi_config_t csi_config = {

        .enable = true,

        .acquire_csi_legacy = true,
        .acquire_csi_force_lltf = CSI_FORCE_LLTF,

        .acquire_csi_ht20 = true,
        .acquire_csi_ht40 = true,

        .acquire_csi_vht = false,
        .acquire_csi_su = false,
        .acquire_csi_mu = false,
        .acquire_csi_dcm = false,
        .acquire_csi_beamformed = false,

        .acquire_csi_he_stbc_mode = 2,

        .val_scale_cfg = 0,
        .dump_ack_en = false,
        .reserved = false
    };


#elif CONFIG_IDF_TARGET_ESP32C6

    /*
     * C6用設定
     */
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
        .reserved = false
    };


#else

    /*
     * ========================================================
     * ★ 通常ESP32ではこの設定が使われる
     * ========================================================
     */
    wifi_csi_config_t csi_config = {

        /*
         * LLTFを取得
         */
        .lltf_en = true,

        /*
         * HT-LTFは取得しない
         */
        .htltf_en = false,

        /*
         * STBC HT-LTFも取得しない
         */
        .stbc_htltf2_en = false,

        /*
         * LTF merge有効
         */
        .ltf_merge_en = true,

        /*
         * channel filter有効
         */
        .channel_filter_en = true,

        /*
         * 手動scale
         */
        .manu_scale = true,

        .shift = true,
    };

#endif


    /*
     * ========================================================
     * ★ AP情報を取得
     * ========================================================
     *
     * 現在ESP32が接続しているAPの情報を取得する。
     *
     * s_ap_info.bssid
     *
     * にAPのMACアドレス(BSSID)が入る。
     */
    static wifi_ap_record_t s_ap_info = {0};

    ESP_ERROR_CHECK(
        esp_wifi_sta_get_ap_info(&s_ap_info)
    );


    /*
     * CSI設定をWi-Fiドライバへ渡す。
     */
    ESP_ERROR_CHECK(
        esp_wifi_set_csi_config(&csi_config)
    );


    /*
     * ========================================================
     * ★★★★★ ものすごく重要 ★★★★★
     * ========================================================
     *
     * CSIを取得したときに呼ぶcallbackとして
     *
     * wifi_csi_rx_cb
     *
     * を登録する。
     *
     * さらにctxとして
     *
     * s_ap_info.bssid
     *
     * = 接続中APのMAC
     *
     * を渡している。
     *
     *
     * だからcallback内の
     *
     * if (memcmp(info->mac, ctx, 6)) {
     *     return;
     * }
     *
     * によって
     *
     * APから来たパケットだけCSI_DATAとして出力
     *
     * という処理になる。
     */
    ESP_ERROR_CHECK(
        esp_wifi_set_csi_rx_cb(
            wifi_csi_rx_cb,
            s_ap_info.bssid
        )
    );


    /*
     * ========================================================
     * ★ CSI取得を有効化
     * ========================================================
     */
    ESP_ERROR_CHECK(
        esp_wifi_set_csi(true)
    );
}





/*
 * Echo Replyを正常に受信したときに呼ばれる
 */
static void wifi_ping_success_cb(
    esp_ping_handle_t hdl,
    void *args)
{
    ping_success_count++;
}
/*
 * ============================================================
 * pingを開始する処理
 * ============================================================
 */
static esp_err_t wifi_ping_router_start()
{




    /*
     * pingのデフォルト設定を取得。
     */
    esp_ping_config_t ping_config =
        ESP_PING_DEFAULT_CONFIG();


    /*
     * ========================================================
     * ping回数
     * ========================================================
     *
     * count = 0
     *
     * ESP-IDF pingでは継続的にpingする設定として使われる。
     */
    ping_config.count = 0;


    /*
     * ========================================================
     * ping間隔
     * ========================================================
     *
     * CONFIG_SEND_FREQUENCY = 1000
     *
     * 1000 / 1000 = 1ms
     *
     * → 1ms間隔
     * → 設定上1000 ping/s
     */
    ping_config.interval_ms =
        1000 / CONFIG_SEND_FREQUENCY;


    ping_config.task_stack_size = 3072;


    /*
     * ICMP Echo Requestのpayloadサイズ
     */
    ping_config.data_size = 1;


    /*
     * ESP32自身のIP情報を取得。
     */
    esp_netif_ip_info_t local_ip;

    esp_netif_get_ip_info(
        esp_netif_get_handle_from_ifkey("WIFI_STA_DEF"),
        &local_ip
    );


    /*
     * IPとGatewayを表示。
     */
    ESP_LOGI(
        TAG,
        "got ip:" IPSTR ", gw: " IPSTR,
        IP2STR(&local_ip.ip),
        IP2STR(&local_ip.gw)
    );


    /*
     * ========================================================
     * ★ ping送信先
     * ========================================================
     *
     * local_ip.gw
     *
     * つまりGateway = Wi-Fiルータをpingする。
     */
    ping_config.target_addr.u_addr.ip4.addr =
        ip4_addr_get_u32(&local_ip.gw);

    ping_config.target_addr.type =
        ESP_IPADDR_TYPE_V4;


    /*
     * ping callbackは設定していない。
     *
     * ★ここにもCSI_DATAを出力する処理はない。
     */
    esp_ping_callbacks_t cbs = { 
        .on_ping_success = wifi_ping_success_cb,
        .cb_args = NULL,    
     };


    /*
     * ping session作成。
     */
    esp_ping_new_session(
        &ping_config,
        &cbs,
        &ping_handle
    );


    /*
     * ========================================================
     * ★ ping開始
     * ========================================================
     *
     * ここからルータへEcho Requestを送る。
     *
     * ただし！
     *
     * この関数はCSI_DATAを直接出力していない。
     */
    esp_ping_start(ping_handle);


    return ESP_OK;
}



/*
 * ============================================================
 * ESP32起動後のメイン処理
 * ============================================================
 */
void app_main()
{

    /*
     * NVS初期化
     */
    ESP_ERROR_CHECK(
        nvs_flash_init()
    );


    /*
     * TCP/IP関連初期化
     */
    ESP_ERROR_CHECK(
        esp_netif_init()
    );


    /*
     * Event Loop作成
     */
    ESP_ERROR_CHECK(
        esp_event_loop_create_default()
    );


    /*
     * ========================================================
     * ① Wi-Fiルータへ接続
     * ========================================================
     */
    ESP_ERROR_CHECK(
        example_connect()
    );


    /*
     * ========================================================
     * ② CSI取得準備
     * ========================================================
     *
     * callback登録
     *       +
     * CSI有効化
     */
    wifi_csi_init();


    
    /*
     * ========================================================
     * ③ ルータへのping開始
     * ========================================================
     */
    wifi_ping_router_start();
}