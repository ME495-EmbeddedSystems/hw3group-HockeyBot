import cv2
import numpy as np
import apriltag
import rclpy                            
from rclpy.node import Node             
from sensor_msgs.msg import Image
from tf2_msgs.msg import TFMessage      
from cv_bridge import CvBridge
from sensor_msgs.msg import CameraInfo
from geometry_msgs.msg import TransformStamped
class AprilTag(object):
    def __init__(self, name):
            super().__init__(name)
            self.pub = self.create_publisher(Image, 'image_raw', self.listener_callback, 10)
            self.sub = self.create_subscription(TFMessage, '/tf_static', self.sub_callback, 10)
            self.pub_info = self.create_publisher(CameraInfo, '/color/camera_info', self.InfoPub, 10)
            self.timer = self.create_timer(0.1, self.timer_callback)
            self.counter = 0                                 
            self.cv_bridge = CvBridge()
            self.detector = apriltag("tag36h11")
    def timer_callback(self):
        self.msg = CameraInfo()
        self.msg.height = 640
        self.msg.width = 480
        self.msg.distortion_model = "airhocky"
        self.msg.d = [0.038385, -0.042656, 0.000869, -0.002758, 0.000000]
        self.msg.k = [445.448888, 0.000000, 317.524152, 0.000000, 444.667306, 241.555012, 0.000000, 0.000000, 1.000000]
        self.pub_info(self.msg)
        self.get_logger().info("AprilTag Detection Node has been started.")

         
    def sub_callback(self,data):
        self.x1 = data.transform.translation.x
        self.y1 = data.transform.translation.y
        self.z1 = data.transform.translation.z
        image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

            # Define AprilTag detector
        self.get_logger().info("Detecting AprilTags...")
        options = apriltag.DetectorOptions(families='tag36_11')
        detector = apriltag.Detector(options)
        results = detector.detect(gray)
        # loop over the AprilTag detection results
        for r in results:
                # extract the bounding box (x, y)-coordinates for the AprilTag
                # and convert each of the (x, y)-coordinate pairs to integers
            (ptA, ptB, ptC, ptD) = r.corners
            ptB = (int(ptB[0]), int(ptB[1]))
            ptC = (int(ptC[0]), int(ptC[1]))
            ptD = (int(ptD[0]), int(ptD[1]))
            ptA = (int(ptA[0]), int(ptA[1]))
                # draw the bounding box of the AprilTag detection
            cv2.line(image, ptA, ptB, (0, 255, 0), 2)
            cv2.line(image, ptB, ptC, (0, 255, 0), 2)
            cv2.line(image, ptC, ptD, (0, 255, 0), 2)
            cv2.line(image, ptD, ptA, (0, 255, 0), 2)
                # draw the center (x, y)-coordinates of the AprilTag
            (cX, cY) = (int(r.center[0]), int(r.center[1]))
            cv2.circle(image, (cX, cY), 5, (0, 0, 255), -1)
                # draw the tag family on the image
            tagFamily = r.tag_family.decode("utf-8")
            cv2.putText(image, tagFamily, (ptA[0], ptA[1] - 15),
            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            print("[INFO] tag family: {}".format(tagFamily))
            # show the output image after AprilTag detection
            cv2.imshow("Image", image)
            cv2.waitKey(0)

def main(args=None):                                       
    rclpy.init(args=args)                                   
    node = AprilTag()              
    rclpy.spin(node)                                       
    rclpy.shutdown() 
