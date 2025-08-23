import os
import time
import math
import smbus

from pca9685_servos_msgs.msg import Servo
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
import smbus

class Subscriber(Node):
    maxPulse = 2500
    minPulse = 500

    def __init__(self):
        super().__init__('pca9685_servos_node')
        self.pwm = PCA9685(0x40, debug=False)
        self.pwm.setPWMFreq(50)
 
        self.subscription = self.create_subscription(
            Servo,
            'set_servo',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning


    def listener_callback(self, msg):
        self.get_logger().info(f'I heard a Servo request {msg.number} {msg.position}')
        percent = max(0.0, min(100.0, msg.position))
        pulse = (self.minPulse + (self.maxPulse - self.minPulse) * (percent / 100.0))
        self.pwm.setServoPulse(msg.number, pulse)


# ============================================================================
# Raspi PCA9685 16-Channel PWM Servo Driver
# Original source: https://www.waveshare.com/w/upload/6/6c/Servo_Driver_HAT.7z
# ============================================================================

class PCA9685:

  # Registers/etc.
  __SUBADR1            = 0x02
  __SUBADR2            = 0x03
  __SUBADR3            = 0x04
  __MODE1              = 0x00
  __PRESCALE           = 0xFE
  __LED0_ON_L          = 0x06
  __LED0_ON_H          = 0x07
  __LED0_OFF_L         = 0x08
  __LED0_OFF_H         = 0x09
  __ALLLED_ON_L        = 0xFA
  __ALLLED_ON_H        = 0xFB
  __ALLLED_OFF_L       = 0xFC
  __ALLLED_OFF_H       = 0xFD

  def __init__(self, address=0x40, debug=False):
    self.bus = smbus.SMBus(1)
    self.address = address
    self.debug = debug
    self.write(self.__MODE1, 0x00)
	
  def write(self, reg, value):
    "Writes an 8-bit value to the specified register/address"
    self.bus.write_byte_data(self.address, reg, value)
    if (self.debug):
      print("I2C: Write 0x%02X to register 0x%02X" % (value, reg))
	  
  def read(self, reg):
    "Read an unsigned byte from the I2C device"
    result = self.bus.read_byte_data(self.address, reg)
    if (self.debug):
      print("I2C: Device 0x%02X returned 0x%02X from reg 0x%02X" % (self.address, result & 0xFF, reg))
    return result
	
  def setPWMFreq(self, freq):
    "Sets the PWM frequency"
    prescaleval = 25000000.0    # 25MHz
    prescaleval /= 4096.0       # 12-bit
    prescaleval /= float(freq)
    prescaleval -= 1.0
    if (self.debug):
      print("Setting PWM frequency to %d Hz" % freq)
      print("Estimated pre-scale: %d" % prescaleval)
    prescale = math.floor(prescaleval + 0.5)
    if (self.debug):
      print("Final pre-scale: %d" % prescale)

    oldmode = self.read(self.__MODE1);
    newmode = (oldmode & 0x7F) | 0x10        # sleep
    self.write(self.__MODE1, newmode)        # go to sleep
    self.write(self.__PRESCALE, int(math.floor(prescale)))
    self.write(self.__MODE1, oldmode)
    time.sleep(0.005)
    self.write(self.__MODE1, oldmode | 0x80)

  def setPWM(self, channel, on, off):
    "Sets a single PWM channel"
    self.write(self.__LED0_ON_L+4*channel, on & 0xFF)
    self.write(self.__LED0_ON_H+4*channel, on >> 8)
    self.write(self.__LED0_OFF_L+4*channel, off & 0xFF)
    self.write(self.__LED0_OFF_H+4*channel, off >> 8)
    if (self.debug):
      print("channel: %d  LED_ON: %d LED_OFF: %d" % (channel,on,off))
	  
  def setServoPulse(self, channel, pulse):
    "Sets the Servo Pulse,The PWM frequency must be 50HZ"
    pulse = pulse*4096/20000        #PWM frequency is 50HZ,the period is 20000us
    self.setPWM(channel, 0, int(pulse))

  def setServoPercent(self, channel, percent):
    "Sets the Servo motion roughly percent, 1 to 100"
    pulse = max(1, percent)
    pulse = min(100, pulse)
    self.setPWM(channel, 0, int(pulse))


def main(args=None):
    try:
        with rclpy.init(args=args):
            subscriber = Subscriber()
            name = subscriber.get_fully_qualified_name()
            subscriber.get_logger().info(f'Starting node {name} from pca9685_servos_node')
            rclpy.spin(subscriber)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass

    # GPIO.cleanup()

if __name__ == '__main__':
    main()
