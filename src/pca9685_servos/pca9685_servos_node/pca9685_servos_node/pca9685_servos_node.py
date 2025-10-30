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
    A = 500.0  # steps/second**2
    min_speed = 2.0  # minimum speed to avoid stalling
    dt = 1/50.0  # control loop time step

    def __init__(self, resting=[50.0]*16):
        super().__init__('pca9685_servos_node')
        self.pwm = PCA9685(0x40, debug=False)
        self.pwm.setPWMFreq(50)
        self.last = [self.pulse_from_percent(r) for r in resting]
 
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


    def percent_callback(self, msg):
        self.get_logger().info(f'I heard a Servo request {msg.number} {msg.position}')
        self.move_by_percent(msg.number, msg.position)

    def move_by_percent(self, number, position):
        percent = max(0.0, min(100.0, position))
        pulse = self.pulse_from_percent(percent)
        start = self.last[number]
        end = pulse
        D = abs(end - start)
        self.last[number] = pulse

        # Calculation of smoothed steps
        A = self.A
        min_speed = self.min_speed
        dt = self.dt

        x = 0.0  # current position
        v = 0.0  # current velocity
        max_steps = 500  # prevent infinite loop
        steps = 0
        while x < D:
            if x < (D / 2):
                # Accelerate
                v += A * dt
            else:
                # Decelerate
                v -= A * dt
                if v < min_speed:
                    v = min_speed
            x = min(v * dt + x, D)  # prevent overshoot
            interp_pulse = start + (end - start) * (x / D)
            self.pwm.setServoPulse(number, interp_pulse)
            time.sleep(dt)
            steps += 1
            if steps >= max_steps:
                break
        self.pwm.setServoPulse(number, end)  # ensure final position is set

    def ticks_callback(self, msg):
        self.get_logger().info(f'I heard a ServoByTicks request {msg.number} {msg.ticks}')
        pulse = max(self.minPulse, min(self.maxPulse, msg.ticks))
        self.pwm.setServoPulse(msg.number, pulse)
        self.last = pulse
  
    def pulse_from_percent(self, percent):
        percent = max(0.0, min(100.0, percent))
        pulse = (self.minPulse + (self.maxPulse - self.minPulse) * (percent / 100.0))
        return pulse


def main(args=None):
    resting = [41.0, 54.0]
    subscriber = None
    try:
        rclpy.init(args=args)
        subscriber = ServosNode(resting=resting)
        name = subscriber.get_fully_qualified_name()
        subscriber.get_logger().info(f'Starting node {name} from pca9685_servos_node')
        rclpy.spin(subscriber)
    except (KeyboardInterrupt, ExternalShutdownException) as e:
        subscriber.A = 1000.0  # increase acceleration for shutdown
        print('Shutting down, moving servos to resting positions...')
        for i in range(len(resting)):
            print(f'Moving servo {i} to resting position {resting[i]}%')
            subscriber.move_by_percent(i, resting[i])
    finally:
        if subscriber is not None:
            subscriber.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
