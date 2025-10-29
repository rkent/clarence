import os
import time
import math

from pca9685_servos_msgs.msg import Servo, ServoByTicks, ServoCalibration
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
import signal
from .PCA9685 import PCA9685

class ServosNode(Node):
    maxPulse = 2500
    minPulse = 500
    activeChannels = [0, 1]
    midPercent = [47, 53]

    def __init__(self):
        super().__init__('pca9685_servos_node')
        self.pwm = PCA9685(0x40, debug=False)
        self.pwm.setPWMFreq(50)
 
        self.percent_subscription = self.create_subscription(
            Servo,
            'set_servo',
            self.percent_callback,
            10)
        self.ticks_subscription = self.create_subscription(
            ServoByTicks,
            'set_servo_ticks',
            self.ticks_callback,
            10)
        #self.subscription  # prevent unused variable warning
        self.last = self.pulse_from_percent(50.0)  # neutral position
        self.travel_time = 1.0   # time to complete move
        self.step_size = 10.0  # pulse step size


    def percent_callback(self, msg):
        self.get_logger().info(f'I heard a Servo request {msg.number} {msg.position}')
        percent = max(0.0, min(100.0, msg.position))
        pulse = self.pulse_from_percent(percent)
        start = self.last
        end = pulse
        self.last = pulse
        steps = int(abs(end - start) / self.step_size)
        if steps == 0:
            steps = 1
        step_delay = self.travel_time / steps
        for step in range(1, steps + 1):
            interp_pulse = start + (end - start) * (step / steps)
            self.pwm.setServoPulse(msg.number, interp_pulse)
            time.sleep(step_delay)

    def ticks_callback(self, msg):
        self.get_logger().info(f'I heard a ServoByTicks request {msg.number} {msg.ticks}')
        pulse = max(self.minPulse, min(self.maxPulse, msg.ticks))
        self.pwm.setServoPulse(msg.number, pulse)
  
    def pulse_from_percent(self, percent):
        percent = max(0.0, min(100.0, percent))
        pulse = (self.minPulse + (self.maxPulse - self.minPulse) * (percent / 100.0))
        return pulse


def main(args=None):
    try:
        with rclpy.init(args=args):
            subscriber = ServosNode()
            name = subscriber.get_fully_qualified_name()
            subscriber.get_logger().info(f'Starting node {name} from pca9685_servos_node')
            rclpy.spin(subscriber)
    except (KeyboardInterrupt, ExternalShutdownException) as e:
        print(e)
        if isinstance(e, KeyboardInterrupt):
          pass
        elif isinstance(e, ExternalShutdownException):
          pass

    # GPIO.cleanup()

if __name__ == '__main__':
    main()
