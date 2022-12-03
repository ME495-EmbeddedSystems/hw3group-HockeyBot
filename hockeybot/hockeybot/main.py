import rclpy
from rclpy.node import Node
import numpy as np
import matplotlib.pyplot as plt
from geometry_msgs.msg import PointStamped, Point, PoseArray, Pose
from std_msgs.msg import Bool
from enum import Enum, auto
from moveit_interface.srv import Initial, Goal

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
        self.puck_interval = 1  # 1 means getting consecutive waypoints, 2 means every other, etc.
        # self.home_posn = Pose(x=0.0, y=0.41, z=-0.015)
        self.home_posn = Goal.Request()
        self.home_posn.x = 0.0
        self.home_posn.y = 0.41
        self.home_posn.z = -0.015
        self.home_posn.roll = 3.1416
        self.home_posn.pitch = 0.0
        self.home_posn.yaw = 1.5707
        self.state = State.SETUP

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
        self.one = 0 # TODO take this out

        # Subscribers
        self.sub_wp1 = self.create_subscription(PointStamped, '/waypoint1', self.wp1_callback, 10)
        self.sub_wp2 = self.create_subscription(PointStamped, '/waypoint2', self.wp2_callback, 10)
        self.sub_puck_pose = self.create_subscription(Point, '/puck_pose', 
                                                        self.puck_pose_filter, 10)
        self.sub_sm_plan = self.create_subscription(Bool, '/sm_plan', self.sm_plan_callback, 10)
        self.sub_sm_execute = self.create_subscription(Bool, '/sm_execute', 
                                                        self.sm_execute_callback, 10)

        # Publishers
        self.pub_puck_posn = self.create_publisher(PoseArray, '/puck_position', 10)

        # Clients
        self.initial_client = self.create_client(Initial, "/initial_service")
        self.waypoint_client = self.create_client(Goal, "/waypoint_service")
        self.goal_client = self.create_client(Goal, "/goal_service")

        self.timer = self.create_timer(1/self.frequency, self.timer_callback)

    def starting_posn(self):
        """Move robot from original startup position to home configuration for playing."""

        self.get_logger().info('inside starting_posn call')

        wpx = 0.0
        wpy = 0.41
        wpz = 0.3

        self.start_rq = Goal.Request()
        self.start_rq.x = wpx
        self.start_rq.y = wpy
        self.start_rq.z = wpz
        self.start_rq.roll = 3.1416
        self.start_rq.pitch = 0.0
        self.start_rq.yaw = 1.5707
        self.start_wp_future = self.waypoint_client.call_async(self.start_rq)
        self.start_goal_future = self.goal_client.call_async(self.home_posn)

        self.get_logger().info('start asyncs called')

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

        if data.y >= 1.0:
            if self.initial_puck is True:
                self.initial_puck_pose = data
                self.initial_puck = False
            elif data.y < (self.initial_puck_pose.y - 0.01):    # tolerance to check puck direction
                if self.puck_pose_count == 0 or self.puck_pose_count == self.puck_interval:
                    self.pucks_tmp.append(data)
                self.puck_pose_count += 1

    def wp1_callback(self, data):
        """
        Checks if data is being published to /waypoint1.
        Saving PointStamped to be used in goal service.
        """

        if self.wp1_prev != data:
            self.wp1_traj = data
            self.wp1_flag = 1
            self.wp1_prev = data
            self.get_logger().info(f'wp1 traj {self.wp1_traj}')

    def wp2_callback(self, data):
        """
        Checks if data is being published to /waypoint2.
        Saving PointStamped to be used in waypoint service.
        """

        if self.wp2_prev != data:
            self.wp2_traj = data
            self.wp2_flag = 1
            self.wp2_prev = data
            self.get_logger().info(f'wp2 traj {self.wp2_traj}')

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

        if self.one == 0:
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
                # self.puck_pose_filter()
                if self.puck_pose_count > self.puck_interval:
                    # This means both puck positions were selected
                    self.state = State.TRAJ
            
            if self.state == State.TRAJ:
                # Update the puck positions
                self.p1_msg.position = self.pucks_tmp[0]
                self.p2_msg.position = self.pucks_tmp[1]
                self.puck_posns.poses = [self.p1_msg, self.p2_msg]
                self.pub_puck_posn.publish(self.puck_posns)
                self.state = State.START_PLAN

            if self.state == State.START_PLAN:
                # Store trajectory calculations from TrajCalc

                # If TrajCalc has finished, call Waypoint and Goal services to meet the puck
                if self.wp1_flag == 1 and self.wp2_flag == 1:

                    self.get_logger().info(f'wp1 traj outside callback {self.wp1_traj}')
                    self.get_logger().info(f'wp2 traj outside callback {self.wp2_traj}')

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
                    self.one = 1 # TODO: Take this out
                    self.state = State.AWAIT_PLAN
                    
            if self.state == State.AWAIT_PLAN:
                self.get_logger().info(f'Called waypoint and goal')
                if self.waypoint_future.done() and self.goal_future.done():
                    self.get_logger().info(f'ROBOT SHOULD MOVE -  waypoint and goal DONE')
                    if self.sm_plan_done == True:
                        self.get_logger().info(f'______________________________________')
                        self.state = State.RETURN_HOME

            if self.state == State.RETURN_HOME:
                # Do future path plan from goal position back to home

                # Set starting point to goal position
                self.init_request = Initial.Request()
                self.init_request.x = self.wp1_traj.point.x
                self.init_request.y = self.wp1_traj.point.y
                self.init_request.z = self.home_posn.z
                self.get_logger().info(f'Stuck in RETURN_HOME *****************************')
                if self.initial_flag == 0:
                    self.initial_future = self.initial_client.call_async(self.init_request)
                    self.initial_flag = 1
                    self.get_logger().info(f'Initial callback ==============================')
                if self.initial_future.done() and self.return_flag == 0:
                    self.get_logger().info(f'Return home initial plan DONE !!!!!!!!!!!!!!!s')
                    # Set waypoint to halfway between initial and home
                    # Set goal to home
                    wpx = (self.init_request.x + self.home_posn.x)/2
                    wpy = (self.init_request.y + self.home_posn.y)/2
                    wpz = self.home_posn.z

                    self.wp_return_request = Goal.Request()
                    self.wp_return_request.x = wpx
                    self.wp_return_request.y = wpy
                    self.wp_return_request.z = wpz
                    self.waypoint_future2 = self.waypoint_client.call_async(self.wp_return_request)
                    self.goal_future2 = self.goal_client.call_async(self.home_posn)
                    self.return_flag = 1    # We only want to do these calls once

                if self.return_flag == 1:
                    self.get_logger().info(f'_________________________-Waiting to RESET')
                    if self.waypoint_future2.done() and self.goal_future2.done():
                        self.get_logger().info(f'_________________________ DONE RESET___________________')
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
                        self.wp1_flag = 0
                        self.wp2_flag = 0
                        self.return_flag = 0
                        self.initial_flag = 0
                        self.start_home_flag = 0
                        self.one = 0

def main_entry(args=None):
    """Run Main node."""
    rclpy.init(args=args)
    node = main()
    rclpy.spin(node)
    rclpy.shutdown()