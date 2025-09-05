#!/bin/bash
set -e
echo "===================="
echo "3. Создание файла optical_flow.py"
echo "===================="

mkdir -p ~/optical_flow_ws/src/optical_flow_pkg/optical_flow
cd ~/optical_flow_ws/src/optical_flow_pkg/optical_flow

cat > optical_flow.py << 'EOF'
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, Range
from rclpy.qos import QoSProfile, ReliabilityPolicy

class SensorReader(Node):
    def __init__(self):
        super().__init__('sensor_reader')
        best_effort_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.imu_sub = self.create_subscription(Imu, '/mavros/imu/data', self.imu_callback, best_effort_qos)
        self.range_sub = self.create_subscription(Range, '/mavros/rangefinder/rangefinder', self.range_callback, 10)
        self.x_gyro = 0.0
        self.y_gyro = 0.0
        self.alt = 0.0

    def imu_callback(self, msg: Imu):
        self.x_gyro = msg.angular_velocity.x
        self.y_gyro = msg.angular_velocity.y
        print(f"x_gyro {self.x_gyro:.6f}, y_gyro {self.y_gyro:.6f}")

    def range_callback(self, msg: Range):
        self.alt = msg.range
        print(f"altitude {self.alt:.3f}")

def main(args=None):
    rclpy.init(args=args)
    node = SensorReader()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("Shutting down SensorReader node")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
EOF

# Создание setup.py и package.xml для ROS2
cat > ~/optical_flow_ws/src/optical_flow_pkg/setup.py << 'EOF'
from setuptools import setup

package_name = 'optical_flow_pkg'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    py_modules=['optical_flow'],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@example.com',
    description='Optical flow ROS2 package',
    license='MIT',
    entry_points={
        'console_scripts': [
            'optical_flow = optical_flow:main'
        ],
    },
)
EOF

cat > ~/optical_flow_ws/src/optical_flow_pkg/package.xml << 'EOF'
<package format="3">
  <name>optical_flow_pkg</name>
  <version>0.0.1</version>
  <description>Optical flow ROS2 package</description>
  <maintainer email="user@example.com">user</maintainer>
  <license>MIT</license>
  <buildtool_depend>setuptools</buildtool_depend>
  <exec_depend>rclpy</exec_depend>
  <exec_depend>sensor_msgs</exec_depend>
</package>
EOF

# Сборка пакета
cd ~/optical_flow_ws
colcon build --symlink-install
echo "source ~/optical_flow_ws/install/setup.bash" >> ~/.bashrc

echo "optical_flow.py и ROS2 пакет готовы!"
