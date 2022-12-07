import rclpy
from rclpy.node import Node
import numpy as np
import matplotlib.pyplot as plt
from geometry_msgs.msg import PointStamped, Point, PoseArray, Pose
from std_msgs.msg import Bool
from enum import Enum, auto
from moveit_interface.srv import Initial, Goal, GripperSrv
import time

class State(Enum):
    """
    These are the 7 states of the system.

    It determines what the main timer function and other callback functions should be doing
    on each iteration in the correct order.
    """

    SETUP = auto(),
    INIT_CV = auto(),
    TRAJ = auto(),
    START_PLAN = auto(),
    AWAIT_PLAN = auto(),
    RETURN_HOME = auto()

class main(Node):
    """
    Main node for bridging CV with trajectory calculations to be passed into API to move the robot.

    This node receives data from the computer vision node to be passed into the trajectory
    calculations (TrajCalc) node. It also receives the calculations back from TrajCalc for
    additional processing to ultimately be passed into the SimpleMove API.
    """

    def __init__(self):
        """
        Initialize services, clients, publishers, and subscribers.

        Also initializes class variables for ----.
        """
        super().__init__("main")

        # True initial variables - these only get set once
        self.frequency = 100 # Hz
        self.puck_interval = 5  # 1 means getting consecutive waypoints, 2 means every other, etc.
        self.home_posn = Goal.Request()
        self.home_posn.x = 0.0
        # self.home_posn.z = -0.015
        self.home_posn.y = 0.407    # paddle is right up against the wall - for repeatability 
        self.home_posn.z = -0.008   # includes offset to account for robot heigh variability
        self.home_posn.roll = 3.1416
        self.home_posn.pitch = 0.0
        self.home_posn.yaw = 1.5707
        self.state = State.SETUP
        self.ee_posn = Pose()
        self.iter_count = 0

        # Re-initializable variables - these get reset every cycle
        # self.state = State.INIT_CV
        self.initial_puck = True
        self.puck_pose_count = 0
        self.p1_msg = Pose()
        self.p2_msg = Pose()
        self.p1_msg.position = Point(x=0.0, y=0.0, z=0.0)
        self.p2_msg.position = Point(x=0.0, y=0.0, z=0.0)
        self.pucks_tmp = []
        self.puck_posns = PoseArray()
        self.sm_plan_done = False
        self.sm_execute_done = False
        self.wp1_prev = PointStamped()
        self.wp2_prev = PointStamped()
        self.wp1_prev.point.y = 0.45
        self.wp2_prev.point.y = 0.7
        self.wp1_flag = 0
        self.wp2_flag = 0
        self.return_flag = 0
        self.initial_flag = 0
        self.start_home_flag = 0
        self.cv_to_traj_flag = 0
        self.one = 0
        self.tmr_count = 0

        # Subscribers
        self.sub_wp1 = self.create_subscription(PointStamped, '/waypoint1', self.wp1_callback, 10)
        self.sub_wp2 = self.create_subscription(PointStamped, '/waypoint2', self.wp2_callback, 10)
        self.sub_puck_pose = self.create_subscription(Point, '/puck_pose', 
                                                        self.puck_pose_filter, 10)
        self.sub_sm_plan = self.create_subscription(Bool, '/sm_plan', self.sm_plan_callback, 10)
        self.sub_sm_execute = self.create_subscription(Bool, '/sm_execute', 
                                                        self.sm_execute_callback, 10)
        self.sub_ee_posn = self.create_subscription(Pose, "/ee_posn", self.ee_posn_callback, 10)

        # Publishers
        self.pub_puck_posn = self.create_publisher(PoseArray, '/puck_position', 10)

        # Clients
        self.initial_client = self.create_client(Initial, "/initial_service")
        self.waypoint_client = self.create_client(Goal, "/waypoint_service")
        self.goal_client = self.create_client(Goal, "/goal_service")
        self.gripper_client = self.create_client(GripperSrv, "/gripper_service")

        # Always have the robot start with its grippers open
        time.sleep(2)
        self.gripper_open = GripperSrv.Request()
        self.gripper_open.open = True
        self.gripper_client.call_async(self.gripper_open)
        self.get_logger().info('gripper should open')
        time.sleep(5)

        self.timer = self.create_timer(1/self.frequency, self.timer_callback)

    def ee_posn_callback(self, data):
        """Receive current end-effector position."""
        self.ee_posn.position.x = data.position.x
        self.ee_posn.position.y = data.position.y
        self.ee_posn.position.z = data.position.z

    def starting_posn(self):
        """Move robot from original startup position to home configuration for playing."""

        self.get_logger().info('inside starting_posn call')

        wpx0 = 0.0
        wpy0 = 0.407
        wpz0 = 0.3
        time.sleep(10)
        self.start_rq = Goal.Request()
        self.start_rq.x = wpx0
        self.start_rq.y = wpy0
        self.start_rq.z = 0.5
        self.start_rq.roll = 3.1416
        self.start_rq.pitch = 0.0
        self.start_rq.yaw = 1.5707

        self.goal1 = Goal.Request()
        self.goal1.x = wpx0
        self.goal1.y = wpy0
        self.goal1.z = 0.3
        self.goal1.roll = 3.1416
        self.goal1.pitch = 0.0
        self.goal1.yaw = 1.5707

        self.waypoint_client.call_async(self.start_rq)
        self.goal_client.call_async(self.goal1)

        time.sleep(3)

        self.start_wp2 = Goal.Request()
        self.start_wp2.x = wpx0
        self.start_wp2.y = wpy0
        self.start_wp2.z = 0.1
        self.start_wp2.roll = 3.1416
        self.start_wp2.pitch = 0.0
        self.start_wp2.yaw = 1.5707


        self.start_goal = Goal.Request()
        self.start_goal.x = wpx0
        self.start_goal.y = wpy0
        # self.start_goal.z = -0.015
        self.start_goal.z = -0.02
        self.start_goal.roll = 3.1416
        self.start_goal.pitch = 0.0
        self.start_goal.yaw = 1.5707
        self.start_wp_future = self.waypoint_client.call_async(self.start_wp2)
        self.start_goal_future = self.goal_client.call_async(self.start_goal)

        time.sleep(5)
        self.gripper_open.open = False
        self.gripper_future = self.gripper_client.call_async(self.gripper_open)
        time.sleep(2)

        adapter_offsetwp = Goal.Request()
        adapter_offsetwp.x = wpx0
        adapter_offsetwp.y = wpy0
        adapter_offsetwp.z = 0.01
        adapter_offsetwp.roll = 3.1416
        adapter_offsetwp.pitch = 0.0
        adapter_offsetwp.yaw = 1.5707

        self.start_wp_future = self.waypoint_client.call_async(adapter_offsetwp)
        self.start_goal_future = self.goal_client.call_async(self.home_posn)

        self.get_logger().info('start asyncs called')

        time.sleep(3)

    def puck_pose_filter(self, data):
        """
        Callback for subscription to /puck_pose topic published from CV.

        Checks that puck is moving towards the robot.
        Selects two waypoints to be passed into TrajCalc.

        Args
        ----
        data: Data from /puck_pose topic

        Returns
        -------
        None
        """

        # # Previous one used
        # if data.y >= 1.0:
        #     if self.initial_puck is True:
        #         self.initial_puck_pose = data
        #         self.initial_puck = False
        #     elif data.y < (self.initial_puck_pose.y - 0.01):    # tolerance to check puck direction
        #         if self.puck_pose_count == 0 or self.puck_pose_count == self.puck_interval:
        #             self.pucks_tmp.append(data)
        #             self.get_logger().info(f'puck pose data {data}')
        #         self.puck_pose_count += 1

        # Distance-based
        if self.state == State.INIT_CV:
            self.get_logger().info(f'data.y {data.y} ITER {self.iter_count}')
            if data.y >= 1.0:
                # self.get_logger().info(f'data.y {data.y}')
                if self.initial_puck == True:
                    self.initial_puck_pose = data
                    self.get_logger().info(f'initial pose {self.initial_puck_pose}')
                    self.initial_puck = False
                elif data.y < (self.initial_puck_pose.y - 0.01):    # tolerance to check puck direction
                    if self.puck_pose_count == 0:
                        self.pucks_tmp.append(data)
                        self.get_logger().info(f'puck pose 1 {self.pucks_tmp[0]}')
                        self.puck_pose_count = 1
                    elif data.y < (self.pucks_tmp[0].y - 0.05):     # distance between puck posns # 0.03
                        if self.puck_pose_count == 1:
                            self.pucks_tmp.append(data)
                            self.get_logger().info(f'puck pose 2 {self.pucks_tmp[1]}')
                            self.puck_pose_count = 2

    def wp1_callback(self, data):
        """
        Checks if data is being published to /waypoint1.
        Saving PointStamped to be used in goal service.
        """

        if self.wp1_prev != data and self.cv_to_traj_flag == 1:
            self.wp1_traj = data
            self.wp1_flag = 1
            self.wp1_prev = data
            # self.get_logger().info(f'wp1_traj {self.wp1_traj}')

    def wp2_callback(self, data):
        """
        Checks if data is being published to /waypoint2.
        Saving PointStamped to be used in waypoint service.
        """

        if self.wp2_prev != data and self.cv_to_traj_flag == 1:
            self.wp2_traj = data
            self.wp2_flag = 1
            self.wp2_prev = data
            # self.get_logger().info(f'wp2_traj {self.wp2_traj}')

    def sm_plan_callback(self, msg):
        """Checks if SimpleMove has finished planning the trajectory to hit the puck."""

        if msg.data == False:
            self.sm_plan_done = True

    def sm_execute_callback(self, msg):
        """Checks if SimpleMove has finished executing a trajectory."""

        if msg.data == False:
            self.sm_execute_done = True

    def timer_callback(self):
        """
        Continuously running timer callback.

        Uses the state machine to detect when data for an oncoming puck as been received, select
        waypoints for TrajCalc, and receive trajectories to send to the API.

        Returns
        -------
        None
        """

        # Continuously publish puck positions
        self.puck_posns.poses = [self.p1_msg, self.p2_msg]
        self.pub_puck_posn.publish(self.puck_posns)

        if self.state == State.SETUP:
            if self.start_home_flag == 0:
                self.starting_posn()
                self.start_home_flag = 1
            # self.get_logger().info('waiting for start calls to finish')
            if self.start_wp_future.done() and self.start_goal_future.done():
                self.get_logger().info('----------- start calls finished-------------')
                self.state = State.INIT_CV

        if self.state == State.INIT_CV:
            # self.get_logger().info(f'iteration {self.iter_count}')
            # self.get_logger().info(f'iter {self.iter_count} puck_pose_count {self.puck_pose_count}')
            # if self.puck_pose_count > self.puck_interval:
            # if len(self.pucks_tmp) == 2:
            if self.tmr_count == 0:
                self.time0 = time.time()
                self.tmr_count = 1
            self.time1 = time.time()
            if (self.time1 - self.time0) > 0.5 and self.puck_pose_count != 2:
                # Reset initial variables
                self.state = State.INIT_CV
                self.initial_puck = True
                self.puck_pose_count = 0
                self.p1_msg = Pose()
                self.p2_msg = Pose()
                self.p1_msg.position = Point(x=0.0, y=0.0, z=0.0)
                self.p2_msg.position = Point(x=0.0, y=0.0, z=0.0)
                self.pucks_tmp = []
                self.puck_posns = PoseArray()
                self.sm_plan_done = False
                self.sm_execute_done = False
                self.wp1_prev = PointStamped()
                self.wp2_prev = PointStamped()
                self.wp1_prev.point.y = 0.45
                self.wp2_prev.point.y = 0.7
                self.wp1_flag = 0
                self.wp2_flag = 0
                self.return_flag = 0
                self.initial_flag = 0
                self.start_home_flag = 0
                self.cv_to_traj_flag = 0
                self.one = 0
                self.tmr_count = 0
            if self.puck_pose_count == 2:
                self.get_logger().info('pucks tmp has 2 posns')
                # This means both puck positions were selected
                self.cv_to_traj_flag = 1
                self.state = State.TRAJ
        
        if self.state == State.TRAJ:
            # Update the puck positions
            self.p1_msg.position = self.pucks_tmp[0]
            self.p2_msg.position = self.pucks_tmp[1]
            self.puck_posns.poses = [self.p1_msg, self.p2_msg]
            self.get_logger().info(f'puck posns sent to marno {self.puck_posns.poses}')
            self.pub_puck_posn.publish(self.puck_posns)
            self.state = State.START_PLAN

        if self.state == State.START_PLAN:
            # Store trajectory calculations from TrajCalc

            # If TrajCalc has finished, call Waypoint and Goal services to meet the puck
            if self.wp1_flag == 1 and self.wp2_flag == 1:
                if self.one == 0:
                    self.get_logger().info(f'inside START_PLAN {self.iter_count}')
                    self.wp1_request = Goal.Request()
                    self.wp1_request.x = self.wp1_traj.point.x
                    self.wp1_request.y = self.wp1_traj.point.y
                    self.wp1_request.z = self.home_posn.z
                    self.wp1_request.roll = 3.1416
                    self.wp1_request.pitch = 0.0
                    self.wp1_request.yaw = 1.5707

                    self.wp2_request = Goal.Request()
                    self.wp2_request.x = self.wp2_traj.point.x
                    self.wp2_request.y = self.wp2_traj.point.y
                    self.wp2_request.z = self.home_posn.z
                    self.wp2_request.roll = 3.1416
                    self.wp2_request.pitch = 0.0
                    self.wp2_request.yaw = 1.5707

                    self.get_logger().info(f'wp1 request {self.wp1_request}')
                    self.get_logger().info(f'wp2 request {self.wp2_request}')

                    self.waypoint_future = self.waypoint_client.call_async(self.wp1_request)
                    self.goal_future = self.goal_client.call_async(self.wp2_request)
                    # self.get_logger().info(f'Called waypoint and goal')
                    # time.sleep(20)
                    self.one = 1
                    # self.state = State.AWAIT_PLAN

                if abs(self.ee_posn.position.x - self.wp2_request.x) < 0.01 and \
                    abs(self.ee_posn.position.y - self.wp2_request.y) < 0.01:
                    self.state = State.AWAIT_PLAN

                
        if self.state == State.AWAIT_PLAN:
            if self.waypoint_future.done() and self.goal_future.done():
                if self.sm_plan_done == True:
                    self.state = State.RETURN_HOME

        if self.state == State.RETURN_HOME:
            # Do future path plan from goal position back to home
            if self.initial_flag == 0:
                # Set starting point to goal position
                self.init_request = Initial.Request()
                self.init_request.x = self.wp2_traj.point.x
                self.init_request.y = self.wp2_traj.point.y
                self.init_request.z = self.home_posn.z
                self.initial_future = self.initial_client.call_async(self.init_request)
                self.initial_flag = 1
            if self.initial_future.done() and self.return_flag == 0:
                self.get_logger().info(f'preparing return')
                # Set waypoint to halfway between initial and home
                # Set goal to home
                wpx1 = (self.init_request.x + self.home_posn.x)/2
                wpy1 = (self.init_request.y + self.home_posn.y)/2
                wpz1 = self.home_posn.z

                self.wp_return_request = Goal.Request()
                self.wp_return_request.x = wpx1
                self.wp_return_request.y = wpy1
                self.wp_return_request.z = wpz1
                self.wp_return_request.roll = self.home_posn.roll
                self.wp_return_request.pitch = self.home_posn.pitch
                self.wp_return_request.yaw = self.home_posn.yaw

                # self.get_logger().info(f'wp return {self.wp_return_request}')
                # self.get_logger().info(f'goal return {self.home_posn}')
                self.waypoint_future2 = self.waypoint_client.call_async(self.wp_return_request)
                self.goal_future2 = self.goal_client.call_async(self.home_posn)
                # time.sleep(20)
                self.return_flag = 1    # We only want to do these calls once

            if self.return_flag == 1:
                # self.get_logger().info(f'_________________________-Waiting to RESET')
                if self.waypoint_future2.done() and self.goal_future2.done():
                    if abs(self.ee_posn.position.x - self.home_posn.x) < 0.01 and \
                        abs(self.ee_posn.position.y - self.home_posn.y) < 0.01:
                        # Reset initial variables
                        self.state = State.INIT_CV
                        self.initial_puck = True
                        self.puck_pose_count = 0
                        self.p1_msg = Pose()
                        self.p2_msg = Pose()
                        self.p1_msg.position = Point(x=0.0, y=0.0, z=0.0)
                        self.p2_msg.position = Point(x=0.0, y=0.0, z=0.0)
                        self.pucks_tmp = []
                        self.puck_posns = PoseArray()
                        self.sm_plan_done = False
                        self.sm_execute_done = False
                        self.wp1_prev = PointStamped()
                        self.wp2_prev = PointStamped()
                        self.wp1_prev.point.y = 0.45
                        self.wp2_prev.point.y = 0.7
                        self.wp1_flag = 0
                        self.wp2_flag = 0
                        self.return_flag = 0
                        self.initial_flag = 0
                        self.start_home_flag = 0
                        self.cv_to_traj_flag = 0
                        self.one = 0
                        self.tmr_count = 0
                        self.iter_count += 1
                        
                        self.get_logger().info(f'_________________________ DONE RESET___________________')

def main_entry(args=None):
    """Run Main node."""
    rclpy.init(args=args)
    node = main()
    rclpy.spin(node)
    rclpy.shutdown()
