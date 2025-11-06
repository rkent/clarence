import rclpy
from rclpy.node import Node
import threading
from alsa_gain_msgs.msg import AlsaGain
from .mixer_thread import MixerThread

class AlsaGainNode(Node):

    def __init__(self):
        super().__init__('alsa_gain_node')

        self.publisher = self.create_publisher(AlsaGain, 'alsa_gain', 10)

        # Declare and read ROS2 parameters for mixer control and device
        self.declare_parameter('control', 'Master')
        self.declare_parameter('device', 'default')
        self.declare_parameter('publish_rate', 5.0)  # in seconds
        self.control = self.get_parameter('control').value
        self.device = self.get_parameter('device').value
        publish_rate = self.get_parameter('publish_rate').value

        self.get_logger().info(f"Using mixer control='{self.control}', device='{self.device}'")

        # Initialize the ALSA Mixer object
        self.mixer_thread = MixerThread(self.control, self.device)
        self.thread = threading.Thread(target=self.mixer_thread.run)
        self.thread.start()

        # Set up a timer to publish gain information at the specified rate
        self.timer = self.create_timer(publish_rate, self.publish_gain)

    def publish_gain(self):
        msg = AlsaGain()
        msg.control = self.control
        msg.device = self.device
        msg.percent = self.mixer_thread.volume
        msg.decibels = self.mixer_thread.db
        msg.muted = self.mixer_thread.muted
        self.publisher.publish(msg)
        self.get_logger().info(
            f"Published Gain: {msg.percent}%, "
            f"Gain (dB): {msg.decibels}dB, Muted: {bool(msg.muted)}"
        )

def main(args=None):
    try:
        with rclpy.init(args=args):
            node = AlsaGainNode()
            rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.mixer_thread.close()
        node.thread.join()
