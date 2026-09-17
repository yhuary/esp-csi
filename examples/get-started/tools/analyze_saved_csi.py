from scipy.signal import stft
import csv
import json
import numpy as np
import matplotlib.pyplot as plt

# ============================================================
# 保存したESP32-C5のCSIログ
# ============================================================

log_file = r"C:\Users\hayashi yuri\Desktop\log.csi_recv_router.20260916164125.txt"

timestamps = []
all_csi_complex = []

# ============================================================
# ログ読み込み
# ============================================================

with open(log_file, "r", encoding="utf-8", errors="ignore") as f:

    for line in f:

        if not line.startswith("CSI_DATA,"):
            continue

        row = next(csv.reader([line]))

        # local_timestamp [us]
        timestamp = int(row[9])

        # CSI raw data
        csi_raw_data = json.loads(row[-1])

        # [Imag, Real] → complex(Real, Imag)
        csi_complex = []

        for i in range(len(csi_raw_data) // 2):

            imag = csi_raw_data[i * 2]
            real = csi_raw_data[i * 2 + 1]

            csi_complex.append(complex(real, imag))

        timestamps.append(timestamp)
        all_csi_complex.append(csi_complex)


# ============================================================
# NumPy配列へ変換
# ============================================================

timestamps = np.array(timestamps)
all_csi_complex = np.array(all_csi_complex)

print("CSI_DATAの数 =", len(timestamps))
print("CSI配列の形 =", all_csi_complex.shape)


# ============================================================
# timestamp → 実験開始からの秒
# ============================================================

time_sec = (timestamps - timestamps[0]) / 1_000_000.0


# ============================================================
# 実際のサンプリング間隔を調べる
# ============================================================

interval_sec = np.diff(time_sec)

print()
print("===== Sampling interval =====")

print("平均 =", np.mean(interval_sec) * 1000, "ms")
print("中央値 =", np.median(interval_sec) * 1000, "ms")
print("最小 =", np.min(interval_sec) * 1000, "ms")
print("最大 =", np.max(interval_sec) * 1000, "ms")

actual_fs = 1.0 / np.mean(interval_sec)

print("平均から求めた取得周波数 =", actual_fs, "Hz")


# ============================================================
# CSI index 10 の振幅
# ============================================================

csi_index = 10

csi_one = all_csi_complex[:, csi_index]

amplitude = np.abs(csi_one)


# ============================================================
# 20 Hzの等間隔時間軸を作る
#
# 20 Hz → 1サンプル = 0.05秒
# ============================================================

fs = 20.0

uniform_time = np.arange(
    time_sec[0],
    time_sec[-1],
    1.0 / fs
)


# ============================================================
# 元データを20 Hzの時間軸へ線形補間
# ============================================================

uniform_amplitude = np.interp(
    uniform_time,
    time_sec,
    amplitude
)


print()
print("===== Resampling =====")

print("元データ数 =", len(amplitude))
print("20 Hz等間隔化後 =", len(uniform_amplitude))


# ============================================================
# グラフ1
# 実際のサンプリング間隔
# ============================================================

plt.figure(figsize=(12, 5))

plt.plot(time_sec[1:], interval_sec * 1000)

plt.axhline(
    50,
    linestyle="--",
    label="Ideal 20 Hz = 50 ms"
)

plt.xlabel("Time [s]")
plt.ylabel("Sampling Interval [ms]")
plt.title("Actual CSI Sampling Interval")

plt.legend()
plt.grid()
plt.tight_layout()


# ============================================================
# グラフ2
# 20 Hz等間隔化後のCSI振幅
# ============================================================

plt.figure(figsize=(12, 5))

plt.plot(uniform_time, uniform_amplitude)

plt.xlabel("Time [s]")
plt.ylabel("CSI Amplitude")

plt.title(
    f"Resampled CSI Amplitude "
    f"(20 Hz, CSI index = {csi_index})"
)

plt.grid()
plt.tight_layout()


# ============================================================
# 表示
# ============================================================
# ============================================================
# STFT前処理
# ============================================================

# 平均値を除去
amplitude_detrended = uniform_amplitude - np.mean(uniform_amplitude)

# ============================================================
# STFT
# ============================================================

frequencies, stft_times, Zxx = stft(
    amplitude_detrended,
    fs=20.0,
    window="hann",
    nperseg=64,
    noverlap=32
)

# dBへ変換
stft_db = 20 * np.log10(np.abs(Zxx) + 1e-10)


# ============================================================
# スペクトログラム
# ============================================================

plt.figure(figsize=(12, 6))

plt.pcolormesh(
    stft_times,
    frequencies,
    stft_db,
    shading="gouraud"
)

plt.xlabel("Time [s]")
plt.ylabel("Frequency [Hz]")
plt.title("STFT Spectrogram (Mean Removed)")

plt.colorbar(label="Magnitude [dB]")

# まず人の動きが見やすい低周波側を拡大
plt.ylim(0, 5)

plt.tight_layout()

plt.show()