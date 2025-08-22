import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import BatteryState
from waveshare_3s_ups.INA219 import INA219


class Publisher(Node):
    def __init__(self):
        super().__init__('waveshare_3s_ups_node')
        self.get_logger().info('Waveshare 3s UPS Node Starting')
        self.publisher_ = self.create_publisher(BatteryState, 'ups_state', 10)
        timer_period = 10  # seconds
        self.ina219 = INA219(addr=0x41)
        self.timer_callback()  # Initial call to publish immediately
        self.timer = self.create_timer(timer_period, self.timer_callback)


    def timer_callback(self):
        ina219 = self.ina219
        bus_voltage = ina219.getBusVoltage_V()             # voltage on V- (load side)
        shunt_voltage = ina219.getShuntVoltage_mV() / 1000 # voltage between V+ and V- across the shunt
        current = ina219.getCurrent_mA()                   # current in mA
        power = ina219.getPower_W()                        # power in W
        p = (bus_voltage - 9)/3.6*100
        if(p > 100):p = 100
        if(p < 0):p = 0

        msg = BatteryState()
        msg.voltage = bus_voltage + shunt_voltage
        msg.current = current / 1000
        msg.percentage = p / 100.0
        msg.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_LION
        self.publisher_.publish(msg)
        self.get_logger().info(f'publishing BatteryState voltage={msg.voltage:.3f}V current={msg.current:.3f}A percentage={msg.percentage*100:.1f}%')


def main(args=None):
    try:
        with rclpy.init(args=args):
            publisher = Publisher()
            rclpy.spin(publisher)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass


def main_test():
    # Create an INA219 instance.
    import time
    ina219 = INA219(addr=0x41)
    while True:
        bus_voltage = ina219.getBusVoltage_V()             # voltage on V- (load side)
        shunt_voltage = ina219.getShuntVoltage_mV() / 1000 # voltage between V+ and V- across the shunt
        current = ina219.getCurrent_mA()                   # current in mA
        power = ina219.getPower_W()                        # power in W
        p = (bus_voltage - 9)/3.6*100
        if(p > 100):p = 100
        if(p < 0):p = 0

        # INA219 measure bus voltage on the load side. So PSU voltage = bus_voltage + shunt_voltage
        #print("PSU Voltage:   {:6.3f} V".format(bus_voltage + shunt_voltage))
        #print("Shunt Voltage: {:9.6f} V".format(shunt_voltage))
        print("Load Voltage:  {:6.3f} V".format(bus_voltage))
        print("Current:       {:9.6f} A".format(current/1000))
        print("Power:         {:6.3f} W".format(power))
        print("Percent:       {:3.1f}%".format(p))
        print("")

        time.sleep(2)


if __name__ == '__main__':
    main()
