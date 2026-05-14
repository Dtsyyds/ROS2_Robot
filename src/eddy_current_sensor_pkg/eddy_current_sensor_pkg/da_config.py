import struct
import time


da_ch_conf_h = [
    0xCC, 0xFF, 0x55, 0xAA,
    0x00, 0x01, 0x00, 0x00,
    0x10, 0x00, 0x00, 0x00
]


# 默认16通道配置
da_ch_conf_ref = [
    {'ch':  4, 'index': 1, 'freq': 1000000, 'phase':   0, 'amp': 20},
    {'ch':  3, 'index': 1, 'freq': 1000000, 'phase':   0, 'amp': 20},
    {'ch':  2, 'index': 1, 'freq': 1000000, 'phase': 180, 'amp': 20},
    {'ch':  1, 'index': 1, 'freq': 1000000, 'phase': 180, 'amp': 20},
    {'ch':  8, 'index': 1, 'freq': 1000000, 'phase':   0, 'amp': 20},
    {'ch':  7, 'index': 1, 'freq': 1000000, 'phase':   0, 'amp': 20},
    {'ch':  6, 'index': 1, 'freq': 1000000, 'phase': 180, 'amp': 20},
    {'ch':  5, 'index': 1, 'freq': 1000000, 'phase': 180, 'amp': 20},
    {'ch': 12, 'index': 1, 'freq': 1000000, 'phase':   0, 'amp': 20},
    {'ch': 11, 'index': 1, 'freq': 1000000, 'phase':   0, 'amp': 20},
    {'ch': 10, 'index': 1, 'freq': 1000000, 'phase': 180, 'amp': 20},
    {'ch':  9, 'index': 1, 'freq': 1000000, 'phase': 180, 'amp': 20},
    {'ch': 16, 'index': 1, 'freq': 1000000, 'phase':   0, 'amp': 20},
    {'ch': 15, 'index': 1, 'freq': 1000000, 'phase':   0, 'amp': 20},
    {'ch': 14, 'index': 1, 'freq': 1000000, 'phase': 180, 'amp': 20},
    {'ch': 13, 'index': 1, 'freq': 1000000, 'phase': 180, 'amp': 20}
]


def da_ch_conf_parse2hex(da_list):
    """
    将DA配置转换为二进制
    """
    for item in da_list:
        item['amp'] = max(0, min(100, item['amp']))

    buf = b''
    for item in da_list:
        buf += struct.pack('<LLLL',
                           item['index'],
                           item['freq'],
                           item['phase'],
                           item['amp'])
    return buf


def da_config_func(client, da_conf=None):
    """
    发送DA配置
    """
    if da_conf is None:
        da_conf = da_ch_conf_ref

    head = bytes(da_ch_conf_h)
    body = da_ch_conf_parse2hex(da_conf)
    packet = head + body

    try:
        client.send(packet)
        print("DA config sent")
    except Exception as e:
        print("DA config failed:", e)

    time.sleep(1)