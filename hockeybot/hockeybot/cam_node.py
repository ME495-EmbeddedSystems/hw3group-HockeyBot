"""
Launches realsense IR and depth camera and tracks the puck when going towards the robot.

It checks if the puck is moving towards the robot upto a center of the table and publishes those
values to the /puck_pose topic.

PUBLISHERS:
    + /puck_pose (geometry_msgs/msg/Point) - Publishes the puck position wrt robot frame
"""
import rclpy
from rclpy.node import Node
import numpy as np
import cv2
import pyrealsense2 as rs
from geometry_msgs.msg import Point


class CamNode(Node):
    """
    Runs realsense camera and publishes puck position in robot frame coordinates.

    This node streams IR and depth camera of realsense and using OpenCV tracks the position of the
    puck in the robot frame when it is going towards the robot, only on one half of the airhockey
    table.
    """
    def __init__(self):
        """
        Initialize services, clients, and subscribers.

        Also initializes class variables, counters and flags to calculate puck positions.
        """
        super().__init__('cam_node')
        self.currentpos = self.create_publisher(Point, '/puck_pose', 10)
        self.pos = Point()
        # counter for averaging collective puck points
        self.i = 0
        # counter to average the center point of the table
        self.count = 1
        # Puck point values
        self.x = 0
        self.y = 0
        # center of the table values
        self.cx = 0.0
        self.cy = 0.0
        self.center_flag = True
        # Flag to reset when puck is going the other direction or other side of the table
        self.reset = 0
        # Array to check direction and noise values
        self.checkx = np.array([])
        self.checky = np.array([])
        # Initiating realsense pipeline
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.pipeline_wrapper = rs.pipeline_wrapper(self.pipeline)
        self.pipeline_profile = self.config.resolve(self.pipeline_wrapper)
        self.device = self.pipeline_profile.get_device()
        # Checking if IR camer is on
        found_ir = False
        for s in self.device.sensors:
            if s.get_info(rs.camera_info.name) == 'Stereo Module':
                found_ir = True
                break
        if not found_ir:
            print("The demo requires camera with IR sensor")
            exit(0)
        # Enabling IR camera 1 and depth camera streaming at 90 fps and same specs(480x270x90)
        self.config.enable_stream(rs.stream.infrared, 1, 480, 270, rs.format.y8, 90)
        self.config.enable_stream(rs.stream.depth, 480, 270, rs.format.z16, 90)
        # Start to streaming
        self.profile = self.pipeline.start(self.config)
        # Creating a timer callback
        self.timer = self.create_timer(0.01, self.timer_callback, )

    def GetCenter(self, frame, depth_frame, depth_intrin):
        """
        Capture the center of the table.

        Detects the large circle at the center of the table which coincides with the center of the
        table and updates the center real world coordinates in the camera frame. Used in the
        calibration of realsense frame to robot frame.
        Args:
            frame: Gets the frame data from IR camera
            depth_frame: Gets the frame from the depth camera
            depth_intrin: The intrinsics data from the depth frame
        Returns
        ---------
            None
        """
        circles = cv2.HoughCircles(frame, cv2.HOUGH_GRADIENT, 2, 70, param1=300, param2=40,
                                   minRadius=10, maxRadius=28)
        # To make sure only circle was found
        if circles is not None:
            self.center_flag = True
            # convert the (x, y) coordinates and radius of the circles to integers
            circles = np.round(circles[0, :]).astype("int")
            radii = circles[:, 2]
            max_index = np.argmax(radii)
            # loop over the (x, y) coordinates and radius of the circles
            x, y, r = circles[max_index]
            # Converts pixel coordinates to real world coordinates
            depth = depth_frame.get_distance(x, y)
            depth_point = rs.rs2_deproject_pixel_to_point(depth_intrin, [x, y], depth)
            # Draws the circle in the output image, then draw a rectangle
            # corresponding to the center of the circle
            cv2.circle(frame, (x, y), r, (0, 255, 0), 4)
            cv2.rectangle(frame, (x - 5, y - 5), (x + 5, y + 5), (0, 128, 255), -1)
            # Updating the variables with current point found
            self.cx += depth_point[0]
            self.cy += depth_point[1]
        else:
            self.center_flag = False

    def timer_callback(self):
        """
        Detect the puck, filter noise and publish points moving in the right direction.

        Gets frames from both IR and depth camera and with the help of depth intrinsics, it
        converts pixel coordinates to real world coordinates wrt realsense camera. It transforms
        the puck position from the realsense camera frame to the robot manipulator frame before
        publishing. It also eliminates noisy puck position values and checks if the puck is moving
        in the right direction and on the right side of the table before publishing to the
        /puck_pose topic.
        """
        # Get frames from realsense
        frame = self.pipeline.wait_for_frames()
        ir_frame = frame.get_infrared_frame(1)
        depth_frame = frame.get_depth_frame()
        ir_image = np.asanyarray(ir_frame.get_data())
        # depth_image = np.asanyarray(depth_frame.get_data())
        depth_intrin = depth_frame.profile.as_video_stream_profile().intrinsics
        # Calculates the center of the table
        if self.count < 25:
            self.GetCenter(ir_image, depth_frame, depth_intrin)
            # Only update counter if values center value was added
            if self.cx != 0 and self.cy != 0 and self.center_flag == True:
                self.count += 1
        # Averaging the center of table coordinates
        elif self.count == 25:
            self.GetCenter(ir_image, depth_frame, depth_intrin)
            self.cx = self.cx/25 + 1.015
            self.cy = self.cy/25
            self.count += 1
        else:
            circles = cv2.HoughCircles(ir_image, cv2.HOUGH_GRADIENT, 2, 70, param1=200, param2=40,
                                       minRadius=7, maxRadius=12)
            # ensure at least some circles were found
            if circles is not None:
                # makes sure only one circle was detected
                if len(circles) == 1:
                    # converts the (x, y) coordinates and radius of the circle to integers
                    circles = np.round(circles[0, :]).astype("int")
                    # gets (x, y) coordinates and radius of the circle
                    x, y, r = circles[0]
                    # Converts from pixel coordinates to real world coordinates
                    depth = depth_frame.get_distance(x, y)
                    depth_point = rs.rs2_deproject_pixel_to_point(depth_intrin, [x, y], depth)
                    # draw the circle in the output image, then draw a rectangle
                    # corresponding to the center of the circle
                    cv2.circle(ir_image, (x, y), r, (0, 255, 0), 4)
                    cv2.rectangle(ir_image, (x - 5, y - 5), (x + 5, y + 5), (0, 128, 255), -1)
                    # Updates the checkx and checky arrays
                    self.checkx = np.append(self.checkx,  depth_point[0])
                    self.checky = np.append(self.checky, depth_point[1])

            # show the output image
            cv2.namedWindow('output', cv2.WINDOW_AUTOSIZE)
            cv2.imshow("output", ir_image)
            # Checks if a circle was detected
            if circles is not None:
                # Checks if the x in cam frame if on the right side of the table
                if self.checkx[0] <= 0.005:
                    # Checks if 4 values were in the check arrays
                    if np.shape(self.checkx)[0] == 4:
                        # checks if the current puck position is other side of the table
                        if self.checkx[3] >= 0.005:
                            # Entering the other side of the table so makes reset flag True
                            self.reset = 1
                        # Now its in correct side of the table
                        # Checks if the current puck position is less than the previous position
                        elif (self.checkx[2]) > (self.checkx[3]):
                            # Checks if the position values are increasing
                            if (self.checkx[0]) <= (self.checkx[1]) and \
                               (self.checkx[1]) < (self.checkx[2]):
                                # puck is moving in the right direction,encountered noise
                                # Replaces noise current value to previous position
                                self.checkx[3] = self.checkx[2]
                        if (self.checkx[0]) <= (self.checkx[1]) and\
                           (self.checkx[1]) <= (self.checkx[2]) and \
                           (self.checkx[2]) <= (self.checkx[3]):
                            # Calculating slope and intercept of best fit line
                            try:
                                m, b = np.polyfit(self.checkx, self.checky, 1)
                                # Calculating distance between last point of check array
                                # to the best fit line
                                d = abs(-1 * m * self.checkx[0] +
                                        self.checky[0] - b) / (m**2 + 1)**0.5
                                # Checks if the distance is less than a tolerance of 0.005
                                if d < 0.005:
                                    # Adding these points to averaging sum of x and y
                                    self.x += self.checkx[0]
                                    self.y += self.checky[0]
                                else:
                                    # this means this value is is noisy
                                    self.x += self.checkx[1]
                                    self.y += self.checky[1]
                                # frames per batch
                                self.fpb = 1
                                if self.i == self.fpb-1:
                                    # resetting counter
                                    self.i = 0
                                    # Transforming to robot manipulator frame
                                    self.pos.x = - self.y/self.fpb + self.cy
                                    self.pos.y = - self.x/self.fpb + self.cx
                                    # publishing the point
                                    self.currentpos.publish(self.pos)
                                    # resetting the averagin sum x and y variables
                                    self.x = 0
                                    self.y = 0
                                # elif circles is not None:
                                elif len(circles) == 1:
                                    # updates counter is only 1 circle was detected
                                    self.i += 1
                            except:
                                self.get_logger().info('POLYFIT FAILED')
                                pass
                        # If the reset flag is set reset the check arrays
                        if self.reset == 1:
                            self.reset = 0
                            self.checkx = np.array([])
                            self.checky = np.array([])
                        # remove the published vlaues from check array(FIFO)
                        else:
                            self.checkx = np.delete(self.checkx, 0)
                            self.checky = np.delete(self.checky, 0)
                # If the last check value is on the other side of the table still
                else:
                    self.checkx = np.delete(self.checkx, 0)
                    self.checky = np.delete(self.checky, 0)
        key = cv2.waitKey(1)
        if key & 0xFF == ord('q') or key == 27:
            cv2.destroyAllWindows()


def frames_entry(args=None):
    """Run CamNode node."""
    rclpy.init(args=args)
    node = CamNode()
    rclpy.spin(node)
    node.pipeline.stop()
    rclpy.shutdown()
