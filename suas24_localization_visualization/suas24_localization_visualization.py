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
        self.n_objects = 5

        self.fig, self.axx = plt.subplots(3, 2)
        for ax_row in self.axx:
            for ax in ax_row:
                x_min, x_max = -8, 8
                y_min, y_max = -40, 40
                ax.set_xlim(x_min, x_max)
                ax.set_ylim(y_min, y_max)
        

        plt.draw()
        plt.pause(0.01)

    def compute_tuple(self, n):
        return [(0,0),(0,1),(1,0),(1,1),(2,0)][n]

    def listener_callback(self, msg):
        assert (len(msg.data)%4 == 0)
        data = np.reshape(msg.data, (5,4))
        data = data.astype(np.float32)/100
        index = np.argmax(data[:, 3])
        # print(data)
        # print(index)
        axx_row, axx_col = self.compute_tuple(index)
        self.axx[axx_row, axx_col].scatter(data[0, index], data[1, index])
        plt.draw()
        plt.pause(0.01)

        # data = msg.data
        # index = data[0]
        # x = data[1]/100
        # y = data[2]/100
        # #z = data[3]
        # conf = data[4]/100
        #
        # #intervals for the points such that the point becomes larger (visible)
        # axx_row, axx_col = self.compute_tuple(index)
        # MAGIC = 0.3
        # if (conf >= MAGIC):
        #     self.axx[axx_row, axx_col].scatter(x, y)
        #
        # #plt.imshow(array)
        # plt.draw()
        # plt.pause(0.01)
        #
        # # self.get_logger().info('Logged : "%s"' % data)
        # # self.get_logger().info(f"sum: {np.sum(self.drop_points[index])}")
        

def main(args=None):
    rclpy.init(args=args)

    node = LocalizationVisualization()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
