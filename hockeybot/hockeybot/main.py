import rclpy
from rclpy.node import Node
import numpy as np
import matplotlib.pyplot as plt
from geometry_msgs.msg import PointStamped, Point, PoseArray, Pose
from enum import Enum, auto
from moveit_interface.srv import Initial, Waypoint, Goal, Execute, Addobj, GripperSrv

class State(Enum):
    """
    These are the 7 states of the system.

    It determines what the main timer function and other callback functions should be doing
    on each iteration in the correct order.
    """

    INIT_CV = auto(),
    TRAJ = auto(),
    START_PLAN = auto(),
    AWAIT_PLAN = auto(),
    RETURN_HOME = auto()

class Main(Node):
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
        super().__init("main_node")

        # True initial variables - these only get set once
        self.frequency = 100 # Hz
        self.puck_interval = 1  # 1 means getting consecutive waypoints, 2 means every other, etc.
        self.home_posn = Pose(x=0.0, y=0.41, z=-0.015)
        
        # Re-initializable variables - these get reset every cycle
        self.state = State.INIT_CV
        self.initial_puck = True
        self.puck_pose_count = 0
        self.p1_msg = Pose(x=0.0, y=0.0, z=0.0)
        self.p2_msg = Pose(x=0.0, y=0.0, z=0.0)
        self.pucks_tmp = []
        self.puck_posns = PoseArray()
        self.sm_plan_done = False
        self.sm_execute_done = False
        self.wp1_prev = PointStamped()
        self.wp2_prev = PointStamped()
        self.wp1_flag = 0
        self.wp2_flag = 0

        # Subscribers
        self.sub_wp1 = self.create_subscription(PointStamped, '/waypoint1', self.wp1_callback, 10)
        self.sub_wp2 = self.create_subscription(PointStamped, '/waypoint2', self.wp2_callback, 10)
        self.sub_puck_pose = self.create_subscription(Point, '/puck_pose', 
                                                        self.puck_pose_filter, 10)
        self.sub_sm_plan = self.create_subscription(int, '/sm_plan', self.sm_plan_callback, 10)
        self.sub_sm_execute = self.create_subscription(int, '/sm_execute', 
                                                        self.sm_execute_callback, 10)

        # Publishers
        self.pub_puck_posn = self.create_publisher(PoseArray, '/puck_position', 10)

        # Clients
        self.initial_client = self.create_client(Initial, "/initial_service")
        self.waypoint_client = self.create_client(Goal, "/waypoint_service")
        self.goal_client = self.create_client(Goal, "/goal_service")

        self.timer = self.create_timer(1/self.frequency, self.timer_callback)


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

    def wp2_callback(self, data):
        """
        Checks if data is being published to /waypoint2.
        Saving PointStamped to be used in waypoint service.
        """

        if self.wp2_prev != data:
            self.wp2_traj = data
            self.wp2_flag = 1

    def sm_plan_callback(self, data):
        """Checks if SimpleMove has finished planning the trajectory to hit the puck."""

        if data == 1:
            self.sm_plan_done = True

    def sm_execute_callback(self, data):
        """Checks if SimpleMove has finished executing a trajectory."""
        if data == 1:
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

        if self.state == State.INIT_CV:
            self.puck_pose_filter()
            if self.puck_pose_count > self.puck_interval:
                # This means both puck positions were selected
                self.state = State.TRAJ
        
        if self.state == State.TRAJ:
            # Update the puck positions
            self.p1_msg = self.pucks_tmp[0]
            self.p2_msg = self.pucks_tmp[1]
            self.puck_posns.poses = [self.p1_msg, self.p2_msg]
            self.pub_puck_posn.publish(self.pub_puck_posn)
            self.state = State.START_PLAN

        if self.state == State.START_PLAN:
            # Store trajectory calculations from TrajCalc
            self.wp1_callback()
            self.wp2_callback()

            # If TrajCalc has finished, call Waypoint and Goal services to meet the puck
            if self.wp1_flag == 1 and self.wp2_flag == 1:
                self.waypoint_future = self.waypoint_client.call_async(f"x: {self.wp2_traj.x}, \
                                                                         y: {self.wp2_traj.y}, \
                                                                         z: {self.wp2_traj.z}")
                self.goal_future = self.goal_client.call_async(f"x: {self.wp1_traj.x}, \
                                                                 y: {self.wp1_traj.y}, \
                                                                 z: {self.wp1_traj.z}")

                self.state = State.AWAIT_PLAN
                
        if self.state == State.AWAIT_PLAN:
            if self.waypoint_future.done() and self.goal_future.done():
                self.sm_plan_callback()
                if self.sm_plan_done == True:
                    self.state = State.RETURN_HOME

        if self.state == State.RETURN_HOME:
            # Call services here
            self.initial_future = self.initial_client.call_async()
            if self.initial_future.done():
                self.waypoint_future = self.waypoint_client.call_async()
                self.goal_future = self.goal_client.call_async()

            if self.waypoint_future.done() and self.goal_future.done():
                # Reset initial variables
                self.state = State.INIT_CV
                self.initial_puck = True
                self.puck_pose_count = 0
                self.p1_msg = Pose(x=0.0, y=0.0, z=0.0)
                self.p2_msg = Pose(x=0.0, y=0.0, z=0.0)
                self.pucks_tmp = []
                self.puck_posns = PoseArray()
                self.sm_plan_done = False
                self.sm_execute_done = False