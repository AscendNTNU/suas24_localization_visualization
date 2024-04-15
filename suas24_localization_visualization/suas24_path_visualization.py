import rclpy
from rclpy.node import Node
import cv2
from std_msgs.msg import String
import matplotlib.pyplot as plt
from matplotlib.pyplot import draw
import numpy as np
import json

class PathVisualization(Node):

    def __init__(self):
        super().__init__('PathVisualization')
        self.subscription = self.create_subscription(
            String,
            '/viz/flythrough',
            self.listener_callback,
            10)

        plt.ion() #enables interatctive mode

        #self.drop_points = np.zeros((self.n_objects, 20, 20))
        
        #self.fig, self.axx = plt.subplots(3, 2, figsize=(15, 15))
        #self.axx[0, 0].imshow(self.drop_points[0])

        self.drone_xs = []
        self.drone_ys = []

        self.ground_truth_points = [(38, -29), (22, -18), (7, -7), (-10.4873, 6.57012), (-26, 18)] 

        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(111)

        self.ax.set_xlim([-40, 40])
        self.ax.set_ylim([-40, 40])

        self.camera_xs = []
        self.camera_ys = []
        self.camera_c  = ["#000000", "#444444", "#888888", "#CCCCCC"]
        self.cam_scatter = None

        self.detections = []

    def make_plots(self):

        self.ax.clear()
        for x, y in self.ground_truth_points:
            self.ax.scatter([x], [y], color="#000000")

        if len(self.camera_xs):
            self.ax.plot(self.camera_xs + [self.camera_xs[0]], self.camera_ys + [self.camera_ys[0]])

        self.ax.plot(self.drone_xs, self.drone_ys, color="#5555FF")

        for det_x, det_y, drone_x, drone_y in self.detections:
            self.ax.scatter([det_x, drone_x], [det_y, drone_y], c=["#00BB00", "#5555FF"])
            self.ax.plot([det_x, drone_x], [det_y, drone_y], color="#555555")

        self.ax.set_xlim([-40, 40])
        self.ax.set_ylim([-40, 40])

        self.fig.canvas.draw()
        self.fig.canvas.flush_events()

    def listener_callback(self, msg):
        data = json.loads(msg.data)

        drone_x, drone_y = data["drone_pos"]
        self.drone_xs.append(drone_x)
        self.drone_ys.append(drone_y)

        self.camera_xs = [t[0] for t in data["cam_corners"]]
        self.camera_ys = [t[1] for t in data["cam_corners"]]

        self.camera_xs[2], self.camera_xs[3] = self.camera_xs[3], self.camera_xs[2]
        self.camera_ys[2], self.camera_ys[3] = self.camera_ys[3], self.camera_ys[2]

        if "detections" in data:
            self.detections += data["detections"]

        self.make_plots()

        """
        images = msg.images
        self.get_logger().info("Heigth: %s" % images[0].height)
        for idx, object in enumerate(images):
            #self.get_logger().info("MSG: %s" % object)
            result = sensormsg_to_numpy(object)
            draw_image = result[0] #boolean representing if the image size is large enough to be displayed
            img = result[1] 
            #self.get_logger().info("Received image: %s" % str(idx) + ", " + str())

            if draw_image:
                print(img.shape)

                #heatmap = create_heatmap(img[1])

                axx_row = idx // 2
                axx_col = 0 if idx % 2 == 0 else 1

         #       self.axx[axx_row, axx_col].imshow(img)
                plt.imshow(img)
                plt.draw()
                plt.pause(0.01)

                if idx == 1:
                    cv2.imwrite("/home/ascend/repos/mono24/normal.png", img)
                    #cv2.imwrite("/home/ascend/repos/mono24/heatmap.png", heatmap)
        
            
                
        """


def main(args=None):
    rclpy.init(args=args)

    node = PathVisualization()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()