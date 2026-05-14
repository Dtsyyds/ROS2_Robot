import traceback
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, String
import numpy as np

from .tcp_client import create_tcp_client, connect_tcp
from .adc_parser import (
    ad_recv_thread,
    ad_sample_rate_conf,
    ad_start_sample_conf,
    get_ADC_sensitivity
)
from .da_config import da_config_func


class SensorNode(Node):
    def __init__(self, default_da_config):
        super().__init__('adc_sensor_node')
        self.default_da_config = default_da_config
        self.pub_vpp = self.create_publisher(Float32MultiArray, 'adc/vpp', 10)
        self.init_sub = self.create_subscription(
            String,
            '/eddy_current_sensor/init_cmd',   # 涡流传感器初始化话题
            self.init_callback,
            10
        )

        self.client = None
        self.connected = False
        self.host = '192.168.1.11'
        self.port = 8080
        self.is_initialized = False
        self.timer = None

        self.get_logger().info('SensorNode started, waiting for "打开涡流传感器" command on /eddy_current_sensor/init_cmd')

    def init_callback(self, msg):
        if self.is_initialized:
            self.get_logger().warn('Already initialized, ignoring duplicate command')
            return

        # 检查消息内容是否为 "打开涡流传感器"
        if msg.data.strip() != "打开涡流传感器":
            self.get_logger().warn(f'Ignoring unknown command: {msg.data}')
            return

        self.get_logger().info('Received "打开涡流传感器" command, initializing hardware...')

        try:
            # 使用默认参数初始化
            sample_rate = 25000000  # 25MHz
            da_config = self.default_da_config

            self.get_logger().info(f'Using sample_rate: {sample_rate}')
            self.init_hardware(sample_rate, da_config)

            # 启动定时器发布数据（100ms）
            self.timer = self.create_timer(0.1, self.timer_callback)
            self.is_initialized = True
            self.get_logger().info('Sensor hardware initialized, data publishing started')

        except Exception as e:
            self.get_logger().error(f'Init failed: {e}\n{traceback.format_exc()}')

    def init_hardware(self, sample_rate, da_config):
        # 创建 TCP 连接
        self.client = create_tcp_client()
        try:
            connect_tcp(self.client, self.host, self.port)
            self.connected = True
            self.get_logger().info('TCP connected to sensor')
        except Exception as e:
            self.get_logger().error(f'TCP connect failed: {e}')
            self.connected = False
            raise

        # 启动接收线程
        ad_recv_thread(self.client)

        # 配置采样率
        ad_sample_rate_conf(sample_rate, self.client)

        # 开始采集
        ad_start_sample_conf(self.client)

        # 配置 DA 激励信号
        da_config_func(self.client, da_config)

    def timer_callback(self):
        if not self.connected:
            return

        try:
            baseline = np.ones((4, 4), dtype=float)
            _, realtime_vpp = get_ADC_sensitivity(baseline)
            ch_data = realtime_vpp.flatten().tolist()
            msg = Float32MultiArray()
            msg.data = ch_data
            self.pub_vpp.publish(msg)
        except Exception as e:
            self.get_logger().error(f'timer_callback error: {e}')

    def destroy_node(self):
        if self.client:
            self.client.close()
        super().destroy_node()


def main(args=None):
    # 默认 DA 配置（与原 da_config.py 中的 da_ch_conf_ref 一致）
    default_da_config = [
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

    rclpy.init(args=args)
    node = SensorNode(default_da_config)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()