import rclpy                            
from rclpy.node import Node             
from sensor_msgs.msg import Image       
from cv_bridge import CvBridge
from sensor_msgs.msg import CameraInfo
import sys
sys.path.append('/home/hanyin/nuws/src/librealsense/')
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
            mask_black = cv2.inRange(hsv_img, lower_red, upper_red)   
            contours, hierarchy = cv2.findContours(
            mask_black, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)
            for cnt in contours:                                    
                if cnt.shape[0] < 150:
                    continue
                """contour for puck"""
                # if len(contours) != 0:
                #     areas = [cv2.contourArea(c) for c in contours]
                #     max_index = np.argmax(areas)
                #     cnt=contours[max_index]
                #     M = cv2.moments(cnt)
                #     if M['m00'] != 0:
                #     cx = int(M['m10']/M['m00'])
                #     cy = int(M['m01']/M['m00'])
                #     centroid = (cx, cy)
                #     im = cv2.drawContours(color_image, cnt, -1, (0,255,255), 7)
                #     centd = cv2.circle(im, centroid, 3, (0,0,0), 7)
                (x, y, w, h) = cv2.boundingRect(cnt)               
                cv2.drawContours(image, [cnt], -1, (180, 255, 255), 7)  
                cv2.circle(image, (int(x+w/2), int(y+h/2)), 3,(0, 0, 0), 7)                         
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
            cv2.imshow("object", image)                            
            cv2.waitKey(1) 

def main(args=None):                                       
    rclpy.init(args=args)                                   
    node = ImageSubscriber()              
    rclpy.spin(node)                                       
    rclpy.shutdown() 