import threading
import struct
import time
import numpy as np

# =========================
# ADC协议参数
# =========================

ad_sample_list = [1000, 50000, 100000, 5000000, 25000000]

ad_start_sample_cmd = [0xA0, 0xFF, 0x55, 0xAA]
ad_sample_rate_cmd = [0x20, 0xAA, 0x55, 0xAA]

ad_data_h = [0x33, 0xDD, 0x33, 0xDD]
ad_data_len = [0x40, 0x40, 0x00, 0x00]

# =========================
# 全局缓存
# =========================

adc_ch_data = [
    {'index': 0, 'data': [0]*512}
    for _ in range(16)
]

data_lock = threading.Lock()
data_ready = False


# =========================
# ADC配置
# =========================

def ad_start_sample_conf(client):
    client.send(bytes(ad_start_sample_cmd))
    time.sleep(1)


def ad_sample_rate_conf(sample_rate, client):
    if sample_rate not in ad_sample_list:
        print("Unsupported sample rate")
        return

    buf = bytes(ad_sample_rate_cmd) + struct.pack('<L', sample_rate)
    client.send(buf)
    time.sleep(1)


# =========================
# 数据接收线程
# =========================

def receive_handler(client):
    global data_ready

    while True:
        try:
            raw = client.recv(16460)
            if len(raw) != 16460:
                continue

            raw_list = list(raw)

            if raw_list[0:4] != ad_data_h:
                continue
            if raw_list[8:12] != ad_data_len:
                continue

            adc_data = raw[12:12+16384]

            values = [
                int.from_bytes(adc_data[i:i+2], 'little')
                for i in range(0, len(adc_data), 2)
            ]

            with data_lock:
                for i in range(512):
                    for ch in range(16):
                        adc_ch_data[ch]['data'][i] = values[16*i + ch]

                data_ready = True

        except Exception as e:
            print("ADC receive error:", e)
            time.sleep(0.1)


def ad_recv_thread(client):
    t = threading.Thread(target=receive_handler, args=(client,), daemon=True)
    t.start()


# =========================
# 信号处理
# =========================

def get_ADC_realtime_Vpp():
    with data_lock:
        if not data_ready:
            return np.zeros((4,4))

        vpp = []
        for ch in adc_ch_data:
            d = sorted(ch['data'])
            top = sum(d[-5:]) / 5
            bot = sum(d[:5]) / 5
            vpp.append(top - bot)

    vpp = np.array(vpp) * 0.122 * 5 * 1.03

    return vpp.reshape(4,4)


def get_ADC_sensitivity(baseline):
    realtime = get_ADC_realtime_Vpp()

    if np.any(baseline == 0):
        return realtime, realtime

    sens = (realtime - baseline) / baseline * 100
    sens[np.logical_and(sens >= -10, sens <= 1)] = 0

    return sens, realtime