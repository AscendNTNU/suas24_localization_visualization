import rclpy
from rclpy.node import Node

from std_msgs.msg import Int32MultiArray

class LocalizationVisualization(Node):

    def __init__(self):
        super().__init__('LocalizationVisualization')
        self.subscription = self.create_subscription(
            Int32MultiArray,
            '/viz/points',
            self.listener_callback,
            10)
        self.subscription  

    def listener_callback(self, msg):
        layout = msg.layout
        data = msg.data
        
        self.get_logger().info('Logged : "%s"' % data)


def main(args=None):
    rclpy.init(args=args)

    node = LocalizationVisualization()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
