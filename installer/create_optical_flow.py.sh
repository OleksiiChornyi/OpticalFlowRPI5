#!/bin/bash
set -e
echo "===================="
echo "4. Создание optical_flow.py"
echo "===================="

mkdir -p ~/OpticalFlowRPI5/scripts
cat > ~/OpticalFlowRPI5/scripts/optical_flow.py << 'EOF'
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, Imu, Range
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import cv2
import struct
import math
import time
import serial
from rclpy.qos import QoSProfile, ReliabilityPolicy

# ====== Настройки MAVLink ======
UART_MAVLINK_BAUDRATE = 115200
UART_SYSTEM_ID = 1
UART_COMPONENT_ID = 0x54
packet_sequence = 0
MAV_OPTICAL_FLOW_CONF_MIN = 0.225
MAV_OPTICAL_FLOW_CONF_MAX = 0.65
MAV_OPTICAL_FLOW_MESSAGE_ID = 100
MAV_OPTICAL_FLOW_ID = 0
MAV_OPTICAL_FLOW_EXTRA_CRC = 175

# ====== Фильтры и вспомогательные функции ======
class RunningAverage:
    def __init__(self, window_size=100):
        self.window_size = window_size
        self.window = []
        self.sum = 0

    def add(self, value):
        if len(self.window) == self.window_size:
            self.sum -= self.window.pop(0)
        self.window.append(value)
        self.sum += value
        return self.get_average()

    def get_average(self):
        if len(self.window) == 0:
            return 0
        return self.sum / len(self.window)

def sign(data):
    if data > 0: return 1
    elif data < 0: return -1
    else: return 0

def get_diff_img_gyro(img_data, gyro_data):
    if sign(img_data * gyro_data) > 0:
        return sign(img_data - gyro_data) * abs(abs(img_data) - abs(gyro_data))
    else:
        return sign(img_data - gyro_data) * abs(abs(img_data) + abs(gyro_data))

def checksum(data, extra):
    output = 0xFFFF
    for i in range(len(data)):
        tmp = data[i] ^ (output & 0xFF)
        tmp = (tmp ^ (tmp << 4)) & 0xFF
        output = ((output >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4)) & 0xFFFF
    tmp = extra ^ (output & 0xFF)
    tmp = (tmp ^ (tmp << 4)) & 0xFF
    output = ((output >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4)) & 0xFFFF
    return output

def clamp_short(value):
    return max(-32768, min(32767, int(value)))

def send_optical_flow_packet(uart, x, y, c):
    global packet_sequence
    payload = struct.pack("<qfffhhBB", 0, 0, 0, 0, clamp_short(x), clamp_short(y), MAV_OPTICAL_FLOW_ID, int(c*255))
    header = struct.pack("<BBBBB", 26, packet_sequence & 0xFF, UART_SYSTEM_ID, UART_COMPONENT_ID, MAV_OPTICAL_FLOW_MESSAGE_ID)
    msg = bytes([0xFE]) + header + payload
    crc = checksum(msg[1:], MAV_OPTICAL_FLOW_EXTRA_CRC)
    msg += struct.pack("<H", crc)
    packet_sequence += 1
    if uart is not None:
        uart.write(msg)

# ====== Основной класс ROS 2 ======
class OpticalFlowNode(Node):
    def __init__(self):
        super().__init__('optical_flow_node')
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )

        # ===== Подписки на MAVROS =====
        best_effort_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.imu_sub = self.create_subscription(
            Imu,
            '/mavros/imu/data',
            self.imu_callback,
            best_effort_qos
        )
        self.range_sub = self.create_subscription(
            Range,
            '/mavros/rangefinder/rangefinder',
            self.range_callback,
            10  # Reliable, нормально для Rangefinder
        )


        # ===== Переменные =====
        self.prev_frame = None
        self.extra_fb = None
        self.running_avg_FPS = RunningAverage(100)
        self.filtered_sub_pixel_x = 0
        self.filtered_sub_pixel_y = 0
        self.filtered_gyro_x = 0
        self.filtered_gyro_y = 0
        self.k = 0.5
        self.FPS = 100
        self.k_displacement_standart = 200
        self.k_gyro_standart = 0.4
        self.k_scale = 1.5
        self.alt = 5
        self.x_gyro = 0
        self.y_gyro = 0

        # Настроить UART MAVLink
        try:
            self.uart_mavlink = serial.Serial("/dev/ttyUSB0", UART_MAVLINK_BAUDRATE, timeout=1)
        except:
            self.uart_mavlink = None
            self.get_logger().warn("UART не найден, MAVLink пакеты не будут отправляться.")

    # ===== Колбэки MAVROS =====
    def imu_callback(self, msg: Imu):
        self.x_gyro = msg.angular_velocity.x
        self.y_gyro = msg.angular_velocity.y

    def range_callback(self, msg: Range):
        self.alt = msg.range

    def update_k_scale(self):
        self.k_scale = max(((self.alt - 5)/5)+1,1)

    # ===== Обработка изображения =====
    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except CvBridgeError as e:
            self.get_logger().error(f"CvBridge error: {e}")
            return

        if frame is None:
            return

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # ===== Фильтр гироскопа =====
        self.update_k_scale()
        k_gyro = self.FPS * self.k_gyro_standart
        k_displacement = (self.FPS**2)/(self.k_displacement_standart * self.k_scale)

        filter_weight = 0.8
        sub_gyro_x = self.x_gyro * k_gyro
        sub_gyro_y = self.y_gyro * k_gyro
        self.filtered_gyro_x = (filter_weight * sub_gyro_x) + ((1 - filter_weight) * self.filtered_gyro_x)
        self.filtered_gyro_y = (filter_weight * sub_gyro_y) + ((1 - filter_weight) * self.filtered_gyro_y)

        # ===== Оптический поток =====
        if self.prev_frame is not None:
            flow = cv2.calcOpticalFlowFarneback(self.prev_frame, gray, None,
                                                0.5, 3, 15, 3, 5, 1.2, 0)
            dx = np.mean(flow[...,0])
            dy = np.mean(flow[...,1])
            response = min(max(np.linalg.norm([dx, dy]),0.0),1.0)

            sub_pixel_x = dx * k_displacement
            sub_pixel_y = dy * k_displacement

            if response > MAV_OPTICAL_FLOW_CONF_MAX:
                filter_weight = 1.0
            elif response < MAV_OPTICAL_FLOW_CONF_MIN:
                filter_weight = 0.0
            else:
                filter_weight = math.exp(-self.k * (MAV_OPTICAL_FLOW_CONF_MAX - response))

            self.filtered_sub_pixel_x = (filter_weight * sub_pixel_x) + ((1 - filter_weight) * self.filtered_sub_pixel_x)
            self.filtered_sub_pixel_y = (filter_weight * sub_pixel_y) + ((1 - filter_weight) * self.filtered_sub_pixel_y)

            subtraction_x = get_diff_img_gyro(self.filtered_sub_pixel_x, self.filtered_gyro_x)
            subtraction_y = get_diff_img_gyro(self.filtered_sub_pixel_y, self.filtered_gyro_y)

            res_x = self.filtered_gyro_x + subtraction_x * self.k_scale
            res_y = self.filtered_gyro_y + subtraction_y * self.k_scale

            send_optical_flow_packet(self.uart_mavlink, res_x, res_y, response)

            # FPS
            now = time.time()
            self.FPS = self.running_avg_FPS.add(1.0 / (1e-6 + (now - getattr(self, 'prev_time', now))))
            self.prev_time = now

            print(f"{res_x:+.2f}x {res_y:+.2f}y {response:.3f} altitude {self.alt:.3f} FPS {self.FPS:.1f}")

        self.prev_frame = gray.copy()

# ====== Основная функция ======
def main(args=None):
    rclpy.init(args=args)
    node = OpticalFlowNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("Shutting down OpticalFlowNode")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
EOF

chmod +x ~/OpticalFlowRPI5/scripts/optical_flow.py
echo "optical_flow.py создан!"
