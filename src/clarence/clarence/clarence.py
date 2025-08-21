import os
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from gpio_leds_msgs.msg import Led


class ClarenceNode(Node):
    def __init__(self):
        super().__init__('clarence_node')
        self.publisher = self.create_publisher(Led, 'set_led', 10)
        self.get_logger().info('Clarence node initialized.')
        # self.timer = self.create_timer(1.0, self.run)

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
