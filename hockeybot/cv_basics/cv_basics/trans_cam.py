import cv2
import numpy as np
import apriltag
import rclpy                            
from rclpy.node import Node             
from sensor_msgs.msg import Image
from tf2_msgs.msg import TFMessage      
from sensor_msgs.msg import CameraInfo
from geometry_msgs.msg import TransformStamped
from cv_bridge import CvBridge, CvBridgeError
class AprilTag_c(Node):
    def __init__(self):
            super().__init__("april")
            self.april_cam_pub = self.create_publisher(CameraInfo, '/camera_info', 10)
            self.cam_sub = self.create_subscription(CameraInfo, '/camera/depth/camera_info ', self.Cam_callback, 10)
            self.april_raw_pub = self.create_publisher(Image, '/image_rect', 10)
            self.raw_sub = self.create_subscription(Image, '/camera/depth/image_rect_raw', self.Raw_callback, 10)
            self.tf_sub = self.create_subscription(TFMessage,'/tf',self.TF_callback,10)

            self.tf_info = TransformStamped()
            self.cam_info = CameraInfo()
            self.raw_info = Image()

    # def timer_callback(self):
    def Cam_callback(self,data):
        self.cam_info = data
        self.april_cam_pub.publish(self.cam_info)

    def Raw_callback(self,data):
        self.raw_info = data
        self.april_raw_pub.publish(self.raw_info)    
        #self.get_logger().info('get in ')

    def TF_callback(self,msg):      
        if len(msg.transforms) >= 4:
            self.k = msg.transforms
            self.get_logger().info('its id ' + self.tf_info[0].child_frame_id) #first id
            self.get_logger().info('its id ' + self.tf_info[1].child_frame_id) #second id
            self.get_logger().info('its id ' + self.tf_info[2].child_frame_id) #third id
            self.get_logger().info('its id ' + self.tf_info[3].child_frame_id) #4th id


        else:
            self.get_logger().info('detected less than 4 apriltags')

        #     self.tf_x0 = self.tf_info[0].transform.translation.x
        #     self.tf_y0 = self.tf_info[0].transform.translation.y
        #     self.tf_z0 = self.tf_info[0].transform.translation.z
        #     self.tf_xr0 = self.tf_info[0].transform.rotation.x
        #     self.tf_yr0 = self.tf_info[0].transform.rotation.y
        #     self.tf_zy0 = self.tf_info[0].transform.rotation.z
        #     self.tf_w0 = self.tf_info[0].transform.rotation.w

        # if self.tf_info.child_frame_id == 'tag36h11:27':
        #     self.tf_x1 = self.tf_info[1].transform.translation.x
        #     self.tf_y1 = self.tf_info[1].transform.translation.y
        #     self.tf_z1 = self.tf_info[1].transform.translation.z
        #     self.tf_xr1 = self.tf_info[1].transform.rotation.x
        #     self.tf_yr1 = self.tf_info[1].transform.rotation.y
        #     self.tf_zy1 = self.tf_info[1].transform.rotation.z
        #     self.tf_w1 = self.tf_info[1].transform.rotation.w
        # if self.tf_info.child_frame_id[1] == 'tag36h11:28':
        #     self.get_logger().info('its id 28 ')
        #     self.tf_x2 = self.tf_info.transform.translation.x
        #     self.tf_y2 = self.tf_info.transform.translation.y
        #     self.tf_z2 = self.tf_info.transform.translation.z
        #     self.tf_xr2 = self.tf_info.transform.rotation.x
        #     self.tf_yr2 = self.tf_info.transform.rotation.y
        #     self.tf_zy2 = self.tf_info.transform.rotation.z
        #     self.tf_w2 = self.tf_info.transform.rotation.w
        # if self.tf_info.child_frame_id == 'tag36h11:30':
        #     self.get_logger().info('its id 30 ')
        #     self.tf_x3 = self.tf_info.transform.translation.x
        #     self.tf_y3 = self.tf_info.transform.translation.y
        #     self.tf_z3 = self.tf_info.transform.translation.z
        #     self.tf_xr3 = self.tf_info.transform.rotation.x
        #     self.tf_yr3 = self.tf_info.transform.rotation.y
        #     self.tf_zy3 = self.tf_info.transform.rotation.z
        #     self.tf_w3 = self.tf_info.transform.rotation.w


def main(args=None):                                       
    rclpy.init(args=args)                                   
    node = AprilTag_c()              
    rclpy.spin(node)                                       
    rclpy.shutdown() 
