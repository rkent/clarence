import os
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from gpio_leds_msgs.msg import Led
from sensor_msgs.msg import Temperature
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Float32
import psutil

def get_cpu_temperature():
    try:
        with open('/sys/class/thermal/thermal_zone0/temp', 'r') as f:
            temp_str = f.read().strip()
            temp_c = int(temp_str) / 1000.0
            return temp_c
    except FileNotFoundError:
        return None

class ClarenceNode(Node):
    def __init__(self):
        super().__init__('clarence_node')
        self.battery_publisher = self.create_publisher(Led, 'set_led', 10)
        self.temperature_publisher = self.create_publisher(Temperature, 'cpu_temperature', 10)
        self.cpu_percent_publisher = self.create_publisher(Float32, 'cpu_percent', 10)
        self.get_logger().info('Clarence node initialized.')
        # self.timer = self.create_timer(1.0, self.run)

        self.battery_subscription = self.create_subscription(
            BatteryState,
            'ups_state',
            self.battery_callback,
            10)
        #self.battery_subscription  # prevent unused variable warning
        self.inited_leds = set()

        # Periodic temperature logging
        self.temp_timer = self.create_timer(5.0, self.log_cpu_temperature)
        # Periodic CPU percentage logging
        self.cpu_percent_timer = self.create_timer(2.0, self.log_cpu_percent)

    # Set LEDs based on battery percentage
    def battery_callback(self, msg):
        if msg.percentage < 0.2:
            color = 'red'
        elif msg.percentage > 0.8:
            color = 'green'
        else:
            color = 'yellow'
        led23_msg = Led()
        led23_msg.number = 23
        led24_msg = Led()
        led24_msg.number = 24
        if color == 'red':
            led23_msg.is_on = True
        elif color == 'green':
            led24_msg.is_on = True
        elif color == 'yellow':
            led23_msg.is_on = True
            led24_msg.is_on = True
        self.battery_publisher.publish(led23_msg)
        self.battery_publisher.publish(led24_msg)

    def log_cpu_temperature(self):
        temp_c = get_cpu_temperature()
        if temp_c is not None:
            self.get_logger().info(f'CPU Temperature: {temp_c:.2f} °C')
        else:
            self.get_logger().warning('Could not read CPU temperature.')
        msg = Temperature()
        msg.temperature = temp_c if temp_c is not None else 0.0
        msg.header.stamp = self.get_clock().now().to_msg()
        self.temperature_publisher.publish(msg)

    def log_cpu_percent(self):
        try:
            cpu_percent = psutil.cpu_percent(interval=1)
            self.get_logger().info(f'CPU Usage: {cpu_percent:.1f}%')
            
            msg = Float32()
            msg.data = cpu_percent
            self.cpu_percent_publisher.publish(msg)
        except Exception as e:
            self.get_logger().error(f'Failed to get CPU percentage: {str(e)}')

    def run(self):
        msg = Led()
        msg.number = 25
        msg.is_on = True
        self.battery_publisher.publish(msg)
        self.log_cpu_temperature()
        self.get_logger().info(f'Published LED request: {msg.number} {msg.is_on}')


def main():
    try:
        rclpy.init()
        node = ClarenceNode()
        node.get_logger().info('Starting Clarence node.')
        time.sleep(1)
        node.run()  # This will run the node's main functionality
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        rclpy.shutdown()
        node.get_logger().info('Shutting down Clarence node.')


if __name__ == '__main__':
    main()
