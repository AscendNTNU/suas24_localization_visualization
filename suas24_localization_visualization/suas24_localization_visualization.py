import rclpy
from rclpy.node import Node
import cv2
from std_msgs.msg import Int32MultiArray
import matplotlib.pyplot as plt
from matplotlib.pyplot import draw
import numpy as np
from suas24_interfaces.msg import VisualizationImgs


#heatmap filter variables
kernel_size = round(5 / 0.01)
kernel = cv2.getGaussianKernel(kernel_size + (1 - kernel_size % 2), 0, cv2.CV_32F)


def sensormsg_to_numpy(msg):
    """
        Expects msg.data to be of type uint8
    """
    if msg.height <= 1:
        return (False, False)
    

    img = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width)) #simply a grey scale image
    return (True, img)

def create_heatmap(img):
    if len(img.shape) > 2:
        img = gray_image = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY) 
        print(f"grayscale shape: {img.shape}")
    
    img = img.astype(np.float32)

    heatmap = cv2.sepFilter2D(img, -1, kernel, kernel)
    return heatmap

class LocalizationVisualization(Node):

    def __init__(self):
        super().__init__('LocalizationVisualization')
        self.subscription = self.create_subscription(
            VisualizationImgs,
            '/viz/heatmap',
            self.listener_callback,
            10)
        self.subscription
        self.n_objects = 5 


        plt.ion() #enables interatctive mode
        plt.show()

        self.drop_points = np.zeros((self.n_objects, 20, 20))
        
        #self.fig, self.axx = plt.subplots(3, 2, figsize=(15, 15))
        #self.axx[0, 0].imshow(self.drop_points[0])
        
        plt.draw()
        plt.pause(0.01)


    def listener_callback(self, msg):

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
        
            
                

        self.get_logger().info("Drawed!")


def main(args=None):
    rclpy.init(args=args)

    node = LocalizationVisualization()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()





























###############################################################################################################
# class LocalizationVisualization(Node):

#     def __init__(self):
#         super().__init__('LocalizationVisualization')
#         self.subscription = self.create_subscription(
#             Int32MultiArray,
#             '/viz/points',
#             self.listener_callback,
#             10)
#         self.subscription
        
        
#         self.img_height = 400
#         self.img_width = 400
#         self.n_objects = 5
#         self.magic_offset = 70 #burde være
#         self.min_x = float('inf')
#         self.max_x = float('-inf')
#         self.min_y = float('inf')
#         self.max_y = float('-inf')
#         #self.bag_start_enu = np.array([10, 14, 25]) * 10 #local enu coordinates at the time the bag starts, scaled same as the message coordinates

#         plt.ion() #enables interatctive mode
#         plt.show()
 
#         self.drop_points = np.zeros((self.n_objects, self.img_height, self.img_width))
#         self.drop_points[self.img_height//2-10 : self.img_height//2+10, self.img_width//2-10 : self.img_width//2+10] = 150 #display origin

#         self.fig, self.axx = plt.subplots(3, 2)
#         self.axx[0, 0].imshow(self.drop_points[0])
        

#         #plt.imshow(self.drop_points[0])
#         plt.draw()
#         plt.pause(0.01)


#     def listener_callback(self, msg):
#         #x = rows
#         #y = cols

#         data = msg.data
#         index = data[0]
#         x = data[1] + self.img_height // 2 #add offsets to adjust origin
#         y = data[2] - self.magic_offset
#         self.get_logger().info(f"Log here: {x, y}")
#         #z = data[3] #trivial
#         conf = data[4]

#         axx_row = index // 2
#         axx_col = 0 if index % 2 == 0 else 1

#         #intervals for the points such that the point becomes larger (visible)
#         padding = 7
#         x_min = max(0, x - padding) 
#         x_max = min(self.img_height, x + padding)
#         y_min = max(0, y - padding)
#         y_max = min(self.img_width, y + padding)

#         self.drop_points[index][x_min:x_max, y_min:y_max] = 255

#         self.axx[axx_row, axx_col].imshow(self.drop_points[index])
#         #self.get_logger().info(f"{x_min, x_max, y_min, y_max}")

#         plt.draw()
#         plt.pause(0.01)

#         self.get_logger().info('Logged : "%s"' % data)
#         self.get_logger().info(f"sum: {np.sum(self.drop_points[index])}")

#     # def adjust_image_size(self):
#     #     new_height = 
#     #     new_width = 
        

# def main(args=None):
#     rclpy.init(args=args)

#     node = LocalizationVisualization()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()
