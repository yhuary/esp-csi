# ESP32-C5 1000 Hz Ping / CSI collector

ESP32-C5から接続中のWi-Fiルーター（DHCPで得たデフォルトゲートウェイ）へ、ICMP Echo Requestを **1 ms周期で10秒間、合計10,000回** 送ります。Echo Replyの受信とCSI処理は送信タスクから分離されているため、応答待ちによって次の送信が遅れません。

CSIは64個の静的バッファプールを経由し、UART0から4 Mbpsのバイナリ形式でPCへ保存します。

## 必要環境

- ESP32-C5
- ESP-IDF v5.5.6
- Python 3.10--3.13
- pyserial

## Wi-Fi設定

~~~zsh
source /Users/miba/.espressif/tools/activate_idf_v5.5.6.sh
cd /Users/miba/esp/esp-csi-1/examples/get-started/csi_recv_router
idf.py menuconfig
~~~

Example Configuration でSSIDとパスワードを設定して保存します。Ping先IPを手動設定する必要はありません。

## 初回セットアップ

~~~zsh
python3 -m venv .venv
.venv/bin/python -m pip install pyserial
~~~

## ビルドと書き込み

~~~zsh
source /Users/miba/.espressif/tools/activate_idf_v5.5.6.sh
idf.py -p /dev/cu.usbmodem5C371828181 build flash
~~~

ポート名が異なる場合は次で確認します。

~~~zsh
ls /dev/cu.*
~~~

## CSI取得

idf.py monitor は使いません。出力はUTF-8テキストではなくCSIバイナリです。

~~~zsh
.venv/bin/python tools/capture_csi.py \
  /dev/cu.usbmodem5C371828181 \
  -o captures/csi_001.csi
~~~

キャプチャツールがESP32-C5を自動リセットします。Wi-Fi接続後に10秒間測定し、次の2ファイルを生成します。自動リセットを使えない配線の場合は `--no-reset` を付け、起動後に基板のEN/Resetボタンを1回押してください。

- captures/csi_001.csi: CSIフレーム
- captures/csi_001.csi.json: 測定集計

## CSIバイナリ解析

`.csi`には各フレームのESP32受信タイムスタンプとCSI生配列が一緒に保存されています。次のコマンドで検証とCSV変換を行います。

~~~zsh
.venv/bin/python tools/analyze_csi_binary.py captures/csi_001.csi
~~~

次の2ファイルを生成します。

- `captures/csi_001.frames.csv`: 1行を1フレームとして、タイムスタンプ、経過時間、RSSI、CSI生配列を保存
- `captures/csi_001.summary.json`: フレーム数、実測周波数、間隔分布、欠落・破損検査結果

`frames.csv`の主な列は次のとおりです。

| 列 | 意味 |
| --- | --- |
| timestamp_us | ESP32が記録した32 bit受信タイムスタンプ（µs） |
| timestamp_unwrapped_us | 約71.6分周期の周回を補正したタイムスタンプ |
| elapsed_us / elapsed_sec | 先頭CSIからの経過時間 |
| interval_us | 直前CSIとの間隔 |
| csi_raw | `[虚部0,実部0,虚部1,実部1,...]`形式のint8生配列 |

実部、虚部、振幅、位相を複素インデックスごとに展開する場合は、次を実行します。このCSVは約10,000フレームで100万行を超えるため、必要な場合だけ生成してください。

~~~zsh
.venv/bin/python tools/analyze_csi_binary.py \
  captures/csi_001.csi \
  --long-csv
~~~

追加で`captures/csi_001.subcarriers.csv`を生成します。振幅は`sqrt(real^2 + imag^2)`、位相は`atan2(imag, real)`のラジアン値です。物理サブキャリア番号への対応はLTF構成に依存するため、列名は`complex_index`としています。

## 集計値

| 項目 | 意味 | 理想値 |
| --- | --- | ---: |
| ping_expected | 設計上の送信回数 | 10000 |
| ping_attempted | 実際に送信処理を呼んだ回数 | 10000 |
| ping_sent | ソケットが受理した回数 | 10000 |
| ping_send_failed | 送信API失敗数 | 0 |
| ping_send_enomem | Wi-Fi送信資源不足による失敗数 | 0 |
| ping_send_eagain | 一時的に送信不能だった回数 | 0 |
| ping_send_other | その他の送信失敗数 | 0 |
| ping_reply_unique | 重複を除いたEcho Reply数 | ルーター性能依存 |
| ping_duration_us | 10,000回の測定窓 | 約10000000 |
| ping_interval_avg_us | 実測送信間隔 | 約1000 |
| csi_received | CSIコールバック受信数 | 無線環境依存 |
| csi_processed | UARTへ処理したCSI数 | csi_receivedと同じ |
| csi_dropped | ESP32内の欠落数 | 0 |
| pc_sequence_gaps | PC保存時に検出した連番欠落 | 0 |

ping_actual_hz は実測Ping送信周波数、csi_actual_hz はCSIタイムスタンプから計算した平均取得周波数です。

## 実装上の注意

- Ping送信、Ping応答受信、CSIコールバック、UART出力を別タスクにしています。
- 標準の esp_ping は応答待ちが送信周期へ影響するため使用していません。
- Wi-Fi省電力は無効です。
- TX AMPDUは送信資源の枯渇を防ぐため有効、RX AMPDUはCSIを個別取得するため無効です。
- CSIキューには大きなCSI配列ではなく、静的プールのスロット番号だけを格納します。
- UART起動ログとCSIストリームは、安定して復号できる4 Mbpsに統一しています。
- ESP-IDF切替後にNVS形式が合わない場合は、該当エラー時だけNVSを消去して自動復旧します。
- 1000回/秒のPingに対してルーターが1000回/秒応答することは、ESP32側だけでは保証できません。ping_sent と ping_reply_unique を分けて評価してください。

## 実機確認結果

ESP32-C5と `/dev/cu.usbmodem5C371828181` を使った10秒測定では、次の結果になりました。

- Ping投入: 10,000回、送信成功10,000回、送信失敗0回
- Ping実測周期: 1000.001 Hz
- CSI取得: 10,009件、平均1002.004 Hz
- ESP32内CSI欠落: 0件
- PC保存時の連番欠落: 0件

無線環境によってEcho Reply数や瞬間的な最大間隔は変動するため、各測定のJSON集計も必ず保存してください。

## バイナリ形式

すべてリトルエンディアンです。

CSIフレーム:

~~~text
AA 55 | version:u8 | type:u8 | sequence:u32 | timestamp_us:u32 |
length:u16 | rssi:i8 | channel:u8 | rx_format:u8 | flags:u8 |
csi_data:i8[length]
~~~

終了フレーム:

~~~text
AA 56 | version:u8 | type:u8 | uint32 x 19
~~~

PC側ツールが両形式を復号し、連番欠落も検査します。
