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

        plt.ion() #enables interatctive mode
        plt.show()
        array = np.array([[255, 0],
                         [0, 255]], dtype=np.uint8)
 
        self.n_objects = 5
        self.drop_points = np.zeros((self.n_objects, 300, 300))

        self.fig, self.axx = plt.subplots(3, 2)
        self.axx[0, 0].imshow(self.drop_points[0])
        

        #plt.imshow(self.drop_points[0])
        plt.draw()
        plt.pause(0.01)


    def listener_callback(self, msg):
        data = msg.data
        index = data[0]
        x = data[1]
        y = data[2]
        #z = data[3]
        conf = data[4]

        axx_row = index // 2
        axx_col = 0 if index % 2 == 0 else 1

        #intervals for the points such that the point becomes larger (visible)
        x_min = max(0, x - 5) 
        x_max = min(300, x + 5)
        y_min = max(0, y - 5)
        y_max = min(300, y + 5)

        #don't ask...
        x_min = min(x_min, x_max - 1)
        x_max = max(x_max, x_min + 1)
        y_min = min(y_min, y_max - 1)
        y_max = max(y_max, y_min + 1)

        self.drop_points[index][x_min:x_max, y_min:y_max] = 255

        self.axx[axx_row, axx_col].imshow(self.drop_points[index])


        #plt.imshow(array)
        plt.draw()
        plt.pause(0.01)

        self.get_logger().info('Logged : "%s"' % data)
        self.get_logger().info(f"sum: {np.sum(self.drop_points[index])}")
        

def main(args=None):
    rclpy.init(args=args)

    node = LocalizationVisualization()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
