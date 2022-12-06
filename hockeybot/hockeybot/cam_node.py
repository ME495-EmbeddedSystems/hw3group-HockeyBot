import rclpy
from rclpy.node import Node
import numpy as np
import cv2
import pyrealsense2 as rs
from geometry_msgs.msg import Point
import matplotlib.pyplot as plt

class CamNode(Node):

    def __init__(self):
        super().__init__('cam_node')
        self.currentpos = self.create_publisher(Point, '/puck_pose', 10)
        self.pos = Point()
        self.folder = []
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
        self.config.enable_stream(rs.stream.depth, 480, 270,  rs.format.z16, 90)


        # Start streaming
        self.profile = self.pipeline.start(self.config)

        self.i =0
        self.count = 1
        self.x = 0
        self.y = 0
        self.cx = 0.0
        self.cy = 0.0
        self.hist = 1
        self.checkx = np.array([])
        self.checky = np.array([])

        self.csv = np.array([[0, 0]])
        self.csv2 = np.array([[0, 0]])
        self.timer = self.create_timer(0.01, self.timer_callback, )
    
    def GetCenter(self, frame, depth_frame, depth_intrin):
        circles = cv2.HoughCircles(frame, cv2.HOUGH_GRADIENT, 2, 70, param1=300, param2=40, minRadius=10, maxRadius=28)
        # ensure at least some circles were found
        if len(circles[0]) == 1:
            # convert the (x, y) coordinates and radius of the circles to integers
            circles = np.round(circles[0, :]).astype("int")
            # loop over the (x, y) coordinates and radius of the circles
            x, y, r = circles[0]
            depth = depth_frame.get_distance(x, y)
            depth_point = rs.rs2_deproject_pixel_to_point(depth_intrin, [x, y], depth)
            # print(x, y, r)
            # draw the circle in the output image, then draw a rectangle
            # corresponding to the center of the circle
            cv2.circle(frame, (x, y), r, (0, 255, 0), 4)
            cv2.rectangle(frame, (x - 5, y - 5), (x + 5, y + 5), (0, 128, 255), -1)
            
            self.cx += depth_point[0]
            self.cy += depth_point[1]

    def timer_callback(self):
        frame = self.pipeline.wait_for_frames()
        ir_frame = frame.get_infrared_frame(1)
        depth_frame = frame.get_depth_frame()
        ir_image = np.asanyarray(ir_frame.get_data())
        depth_image = np.asanyarray(depth_frame.get_data())
        depth_intrin = depth_frame.profile.as_video_stream_profile().intrinsics
        if self.count < 25:
            # print(self.cx, self.cy)
            self.GetCenter(ir_image, depth_frame, depth_intrin)
            if self.cx != 0 and self.cy != 0:
                self.count +=1
        elif self.count == 25:
            self.GetCenter(ir_image, depth_frame, depth_intrin)
            
            self.cx = self.cx/25 + 1.015
            self.cy = self.cy/25 #1.0541 -
            print('Center: ', self.cx, self.cy )
            # print(self.cx, self.cy)
            self.count +=1
        else:
            circles = cv2.HoughCircles(ir_image, cv2.HOUGH_GRADIENT, 2, 70, param1=200, param2=40, minRadius=7, maxRadius=12)
            # ensure at least some circles were found
            if circles is not None:
                # convert the (x, y) coordinates and radius of the circles to integers
                circles = np.round(circles[0, :]).astype("int")
                # loop over the (x, y) coordinates and radius of the circles
                # print(circles[0])
                x, y, r = circles[0]
                depth = depth_frame.get_distance(x, y)
                depth_point = rs.rs2_deproject_pixel_to_point(depth_intrin, [x, y], depth)
                # draw the circle in the output image, then draw a rectangle
                # corresponding to the center of the circle
                cv2.circle(ir_image, (x, y), r, (0, 255, 0), 4)
                cv2.rectangle(ir_image, (x - 5, y - 5), (x + 5, y + 5), (0, 128, 255), -1)
                self.checkx = np.append(self.checkx,  depth_point[0])
                self.checky = np.append(self.checky, depth_point[1])
                self.csv2 = np.vstack((self.csv2, [depth_point[0], depth_point[1]]))
            # cv2.rectangle(ir_image, (self.cx - 5, self.cy - 5), (self.cx + 5, self.cy + 5), (0, 128, 255), -1)
            # show the output image
            cv2.namedWindow('output', cv2.WINDOW_AUTOSIZE)
            cv2.imshow("output", ir_image)
            # print ('Position: ')
            if np.shape(self.checkx)[0] == 4:
                print(self.checkx)
                if (self.checkx[2]) > (self.checkx[3]):
                    print('Fucking up incoming!!!!!!')
                    self.checkx[3] = self.checkx[2]
                if (self.checkx[0]) <= (self.checkx[1]) and (self.checkx[1]) <= (self.checkx[2]) and \
                    (self.checkx[2]) <= (self.checkx[3]):
                    print('DIRECTION IS RIGHT')
                    m, b= np.polyfit(self.checkx, self.checky, 1)
                    d = abs(-1 * m*self.checkx[0] + self.checky[0] - b)/ (m**2 +1)**0.5
                    if d < 0.005:
                        print('VALUE WITHIN RANGE')
                        self.x += self.checkx[0]
                        self.y += self.checky[0]
                        self.csv = np.vstack((self.csv, [self.checkx[0], self.checky[0]]))
                    else:
                        print('VALUE NOT WITHIN RANGE')
                        self.x += self.checkx[1]
                        self.y += self.checky[1]
                        self.csv = np.vstack((self.csv, [self.checkx[1], self.checky[1]]))
                
                # else:
                #     self.checkx = np.delete(self.checkx, 0)
                #     self.checkx = np.delete(self.checkx, 0)
                #     self.checkx = np.delete(self.checkx, 0)
                #     self.checky = np.delete(self.checky, 0)
                #     self.checky = np.delete(self.checky, 0)
                #     self.checky = np.delete(self.checky, 0)
                # frames per batch
                    self.fpb = 1 # 3
                    if self.i == self.fpb-1: 
                        self.i = 0
                        self.pos.x = - self.y/self.fpb
                        self.pos.y = - self.x/self.fpb + self.cx
                        self.currentpos.publish(self.pos)
                        
                        print('PUBLISHING')
                        # print('center', self.cx, self.cy)
                        # print('These are the point values', self.x/self.fpb, self.y/self.fpb)
                        self.x = 0
                        self.y = 0
                    elif circles is not None:
                        self.i += 1
                self.checkx = np.delete(self.checkx, 0)
                self.checky = np.delete(self.checky, 0)
            np.savetxt("PuckPath.csv", self.csv, delimiter = ",")
            np.savetxt("PuckPath_all.csv", self.csv2, delimiter = ",")

        # np.savetxt("cam_node.csv", self.folder, delimiter=",")
        # print('****CSV SAVED******')
        key = cv2.waitKey(1)
        if key & 0xFF == ord('q') or key == 27:
            cv2.destroyAllWindows()

def frames_entry(args=None):
    rclpy.init(args=args)
    
    node = CamNode()
    rclpy.spin(node)
    node.pipeline.stop()
    rclpy.shutdown()

