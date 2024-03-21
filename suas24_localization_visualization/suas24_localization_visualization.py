import rclpy
from rclpy.node import Node

from std_msgs.msg import Int32MultiArray
import matplotlib.pyplot as plt
from matplotlib.pyplot import draw
import numpy as np



class LocalizationVisualization(Node):

    def __init__(self):
        super().__init__('LocalizationVisualization')
        self.subscription = self.create_subscription(
            Int32MultiArray,
            '/viz/points',
            self.listener_callback,
            10)
        self.subscription

        plt.ion()
        plt.show()
        array = np.array([[255, 0],
                         [0, 255]], dtype=np.uint8)
        plt.imshow(array)
        plt.draw()
        plt.pause(0.01)


    def listener_callback(self, msg):
        layout = msg.layout
        data = msg.data
        index = data[0]
        x = data[1]
        y = data[2]
        z = data[3]
        conf = data[4]
        array = np.array([[0, 255],
                         [255, 0]], dtype=np.uint8)
        plt.imshow(array)
        plt.draw()
        plt.pause(0.01)

        self.get_logger().info('Logged : "%s"' % data)
        self.get_logger().info(f"value: {data[1]}")

def main(args=None):
    rclpy.init(args=args)

    node = LocalizationVisualization()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
