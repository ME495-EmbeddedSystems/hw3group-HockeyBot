import rclpy
from rclpy.node import Node
import numpy as np
import cv2
import pyrealsense2 as rs
from geometry_msgs.msg import Point

class CamNode(Node):

    def __init__(self):
        super().__init__('cam_node')
        self.currentpos = self.create_publisher(Point, '/puck_pose', 10)
        self.pos = Point()
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.pipeline_wrapper = rs.pipeline_wrapper(self.pipeline)
        self.pipeline_profile = self.config.resolve(self.pipeline_wrapper)
        self.device = self.pipeline_profile.get_device()
        # device_product_line = str(self.device.get_info(rs.camera_info.product_line))
        found_ir = False
        for s in self.device.sensors:
            if s.get_info(rs.camera_info.name) == 'Stereo Module':
                found_ir = True
                break
        if not found_ir:
            print("The demo requires camera with IR sensor")
            exit(0)
        
        # config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        self.config.enable_stream(rs.stream.infrared, 1, 480, 270,  rs.format.y8, 90)
        self.config.enable_stream(rs.stream.infrared, 2, 480, 270,  rs.format.y8, 90)


        # Start streaming
        self.profile = self.pipeline.start(self.config)

        self.i =0
        self.count = 1
        self.x = 0
        self.y = 0
        self.cx = 0.0
        self.cy = 0.0
        self.timer = self.create_timer(0.01, self.timer_callback)
    
    def GetCenter(self, frame):
        circles = cv2.HoughCircles(frame, cv2.HOUGH_GRADIENT, 2, 70, param1=300, param2=40, minRadius=10, maxRadius=28)
        # ensure at least some circles were found
        if len(circles[0]) == 1:
            # convert the (x, y) coordinates and radius of the circles to integers
            circles = np.round(circles[0, :]).astype("int")
            # loop over the (x, y) coordinates and radius of the circles
            x, y, r = circles[0]
            print(x, y, r)
            # draw the circle in the output image, then draw a rectangle
            # corresponding to the center of the circle
            cv2.circle(frame, (x, y), r, (0, 255, 0), 4)
            cv2.rectangle(frame, (x - 5, y - 5), (x + 5, y + 5), (0, 128, 255), -1)
            
            self.cx += x
            self.cy += y

                

    def timer_callback(self):
        frame = self.pipeline.wait_for_frames()
        ir_frame = frame.get_infrared_frame(1 )
        ir_image = np.asanyarray(ir_frame.get_data())
        if self.count < 50:
            print(self.cx, self.cy)
            self.GetCenter(ir_image)
            if self.cx != 0 and self.cy != 0:
                self.count +=1
        elif self.count == 50:
            self.GetCenter(ir_image)
            print(self.cx/50, self.cy/50)
            self.cx = 0 - self.cx/50
            self.cy = 1.0541 - self.cy/50
            print(self.cx, self.cy)
            self.count +=1
        else:
            circles = cv2.HoughCircles(ir_image, cv2.HOUGH_GRADIENT, 2, 70, param1=200, param2=40, minRadius=7, maxRadius=12)
            # ensure at least some circles were found
            if circles is not None:
                # convert the (x, y) coordinates and radius of the circles to integers
                circles = np.round(circles[0, :]).astype("int")
                # loop over the (x, y) coordinates and radius of the circles
                print(circles[0])
                x, y, r = circles[0]
                # draw the circle in the output image, then draw a rectangle
                # corresponding to the center of the circle
                cv2.circle(ir_image, (x, y), r, (0, 255, 0), 4)
                cv2.rectangle(ir_image, (x - 5, y - 5), (x + 5, y + 5), (0, 128, 255), -1)
                
                self.x += x
                self.y += y
            # cv2.rectangle(ir_image, (self.cx - 5, self.cy - 5), (self.cx + 5, self.cy + 5), (0, 128, 255), -1)
            # show the output image
            cv2.namedWindow('output', cv2.WINDOW_AUTOSIZE)
            cv2.imshow("output", ir_image)
            # print ('Position: ', x, y)
            
            if self.i == 15: 
                self.i = 0
                self.pos.x = self.cx - self.x/15
                self.pos.y = self.cy - self.y/15
                self.currentpos.publish(self.pos)
                print(self.pos , self.x/15, self.y/15)
                self.x = 0
                self.y = 0
            else:
                self.i += 1

        key = cv2.waitKey(1)
        if key & 0xFF == ord('q') or key == 27:
            cv2.destroyAllWindows()

def frames_entry(args=None):
    rclpy.init(args=args)
    node = CamNode()
    rclpy.spin(node)
    node.pipeline.stop()
    rclpy.shutdown()

