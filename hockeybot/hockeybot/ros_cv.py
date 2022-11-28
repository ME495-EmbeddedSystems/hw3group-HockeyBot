import rclpy                            
from rclpy.node import Node             
from sensor_msgs.msg import Image       
from cv_bridge import CvBridge
from sensor_msgs.msg import CameraInfo
import sys
import cv2

sys.path.append('/home/nuws/src/')

print(sys.path)
import pyrealsense2 as rs2

#if (not hasattr(rs2, 'intrinsics')):
    #import pyrealsense2.pyrealsense2 as rs2          
import cv2                              
import numpy as np
lower_red = np.array([0, 90, 128])      #HSV
upper_red = np.array([180, 255, 255])

class ImageSubscriber(Node):
        def __init__(self, name):
            super().__init__(name)                                  
            self.sub = self.create_subscription(Image, 'image_raw', self.listener_callback, 10)
            self.sub_info = self.create_subscription(CameraInfo, '/color/camera_info', self.InfoCallback, 10)     
            self.cv_bridge = CvBridge()
            self.intrinsics = None
            self.pix = None
            self.pix_grade = None
        def object_detect(self, image):
            hsv_img = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)        
            mask_red = cv2.inRange(hsv_img, lower_red, upper_red)   
            contours, hierarchy = cv2.findContours(mask_red, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)
            for cnt in contours:                                    # 去除一些轮廓面积太小的噪声
                if cnt.shape[0] < 150:
                    continue
                circles = cv2.HoughCircles(image, cv2.HOUGH_GRADIENT, 2, 70, param1=200, param2=40, minRadius=7, maxRadius=12)
        # ensure at least some circles were found
                if circles is not None:
            # convert the (x, y) coordinates and radius of the circles to integers
                    circles = np.round(circles[0, :]).astype("int")
            # loop over the (x, y) coordinates and radius of the circles
                    for (x, y, r) in circles:
                # draw the circle in the output image, then draw a rectangle
                # corresponding to the center of the circle
                        cv2.circle(image, (x, y), r, (0, 255, 0), 4)
                        cv2.rectangle(image, (x - 5, y - 5), (x + 5, y + 5), (0, 128, 255), -1)                    
        def InfoCallback(self,data):  
            """ 
            Function: Callback function for geting camera information
            Args:
            data (sensor_msgs/CameraInfo)
             Returns:
            None
            """
            self.intrinsics = rs2.intrinsics()
            self.intrinsics.width = data.width
            self.intrinsics.height = data.height
            self.intrinsics.ppx = data.k[2]
            self.intrinsics.ppx = data.k[5]
            self.intrinsics.fx = data.k[0]
            self.intrinsics.fy = data.k[4]
            self.intrinsics.model  = rs2.distortion.none
            self.intrinsics.list = [0.0, 0.0, 0.0, 0.0, 0.0]
        def listener_callback(self, data):
            self.get_logger().info('Receiving video frame')         
            image = self.cv_bridge.imgmsg_to_cv2(data, 'bgr8')      
            self.object_detect(image)  
        def show_image(self, img, img_name):
            cv2.namedWindow(img_name, cv2.WINDOW_NORMAL)
            cv2.imshow(img_name, img)
            cv2.waitKey(3)
def main(args=None):                                       
    rclpy.init(args=args)                                   
    node = ImageSubscriber()              
    rclpy.spin(node)                                       
    rclpy.shutdown() 