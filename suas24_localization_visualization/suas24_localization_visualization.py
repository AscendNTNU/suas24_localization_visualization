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
        self.fig = plt.figure()
        self.ax1 = self.fig.add_subplot(321, projection='3d')
        self.ax2 = self.fig.add_subplot(322, projection='3d')
        self.ax3 = self.fig.add_subplot(323, projection='3d')
        self.ax4 = self.fig.add_subplot(324, projection='3d')
        self.ax5 = self.fig.add_subplot(325, projection='3d')
        self.axs = [self.ax1, self.ax2, self.ax3, self.ax4, self.ax5]
        for ax in [self.ax1, self.ax2, self.ax3, self.ax4, self.ax5]:
            ax.set_axis_off()
        plt.subplots_adjust(wspace=0, hspace=0)

        self.UPPER = 30
        self.LOWER = 30
        self._x = np.arange(-self.LOWER, self.UPPER)
        self._y = np.arange(-self.LOWER, self.UPPER)
        self._xx, self._yy = np.meshgrid(self._x,self. _y)
        self.x, self.y = self._xx.flatten(), self._yy.flatten()

        self.grid1 = np.zeros((self.LOWER*2,self.UPPER*2))
        self.grid2 = np.zeros((self.LOWER*2,self.UPPER*2))
        self.grid3 = np.zeros((self.LOWER*2,self.UPPER*2))
        self.grid4 = np.zeros((self.LOWER*2,self.UPPER*2))
        self.grid5 = np.zeros((self.LOWER*2,self.UPPER*2))
        self.grids = [self.grid1, self.grid2, self.grid3, self.grid4, self.grid5]

        for index,(ax, grid) in enumerate(zip(self.axs, self.grids)):
            ax.bar3d(self.x, self.y, np.zeros_like(self.x + self.y), 1, 1, grid.flatten(), shade=True)
        
        plt.draw()
        plt.pause(0.01)

    def compute_tuple(self, n):
        return [self][n]

    def listener_callback(self, msg):
        assert (len(msg.data)%4 == 0)
        data = np.reshape(msg.data, (5,4))
        data = data.astype(np.float32)/100
        # index = np.argmax(data[:, 3])
        for index,(ax, grid) in enumerate(zip(self.axs, self.grids)):
            self.grids[index][int(data[index, 0]), int(data[index, 1])] = data[index, 3]
            ax.bar3d(self.x, self.y, np.zeros_like(self.x + self.y), 1, 1, self.grids[index].flatten(), shade=True)
        plt.draw()
        plt.pause(0.01)
        print("done callback")
        # print(data)
        # print(index)
        # axx_row, axx_col = self.compute_tuple(index)
        # self.axx[axx_row, axx_col].scatter(data[0, index], data[1, index])
        # for index,(ax, grid) in enumerate(zip([self.ax1, self.ax2, self.ax3, self.ax4, self.ax5]
        #                               ,[self.grid1, self.grid2, self.grid3, self.grid4, self.grid5])):
        # self.grid[int(data[0,0])+self.N, int(data[0,0])+self.N] = data[0,3]
        # self.ax1.bar3d(self.x, self.y, np.zeros_like(self.x + self.y), 1, 1, self.grid.flatten(), shade=True)
        # plt.draw()
        # plt.pause(0.01)

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
