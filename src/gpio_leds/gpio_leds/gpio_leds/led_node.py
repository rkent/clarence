import os
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from gpio_leds_msgs.msg import Led
import RPi.GPIO as GPIO

class Subscriber(Node):
    def __init__(self):
        super().__init__('led_node')
        self.subscription = self.create_subscription(
            Led,
            'set_led',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.inited_leds = set()

        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)


    def listener_callback(self, msg):
        self.get_logger().debug(f'I heard an LED request {msg.number} {msg.is_on}')
        if msg.number not in self.inited_leds:
            self.inited_leds.add(msg.number)
            GPIO.setup(msg.number, GPIO.OUT)
        GPIO.output(msg.number, GPIO.HIGH if msg.is_on else GPIO.LOW)

def main(args=None):
    try:
        with rclpy.init(args=args):
            subscriber = Subscriber()
            name = subscriber.get_fully_qualified_name()
            subscriber.get_logger().info(f'Starting node {name} from gpio_leds')
            rclpy.spin(subscriber)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass

    GPIO.cleanup()

if __name__ == '__main__':
    main()
