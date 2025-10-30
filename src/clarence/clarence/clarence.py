import os
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from gpio_leds_msgs.msg import Led
from sensor_msgs.msg import BatteryState


class ClarenceNode(Node):
    def __init__(self):
        super().__init__('clarence_node')
        self.publisher = self.create_publisher(Led, 'set_led', 10)
        self.get_logger().info('Clarence node initialized.')
        # self.timer = self.create_timer(1.0, self.run)

        self.subscription = self.create_subscription(
            BatteryState,
            'ups_state',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.inited_leds = set()


    # Set LEDs based on battery percentage
    def listener_callback(self, msg):
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
        self.publisher.publish(led23_msg)
        self.publisher.publish(led24_msg)


    def run(self):
        msg = Led()
        msg.number = 25
        msg.is_on = True
        self.publisher.publish(msg)
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
