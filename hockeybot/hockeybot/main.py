import rclpy
from rclpy.node import Node
import numpy as np
import matplotlib.pyplot as plt
from geometry_msgs.msg import PointStamped, Point
from enum import Enum, auto

class State(Enum):
    """
    These are the 7 states of the system.

    It determines what the main timer function and other callback functions should be doing
    on each iteration in the correct order.
    """

    INITIAL = auto(),
    IK_CAL = auto(),
    PLAN = auto(),
    READY_EXECUTE = auto(),
    EXECUTE = auto(),
    BOX_DIMS = auto(),
    BOX_CREATE = auto()

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

        self.frequency = 100 # Hz

        # Subscribers
        self.sub_wp1 = self.create_subscription(PointStamped, '/waypoint1', self.wp1_callback, 10)
        self.sub_wp2 = self.create_subscription(PointStamped, '/waypoint2', self.wp2_callback, 10)
        self.sub_puck_pose = self.create_subscription(Point, '/puck_pose', 
                                                        self.puck_pose_filter, 10)

        # Publishers
        self.pub_p1 = self.create_publisher(Point, '/puck1_position', 10)
        self.pub_p2 = self.create_publisher(Point, '/puck2_position', 10)

        # Clients
        self.waypoint_client = self.create_client(PointStamped)
        self.goal_client = self.create_client()

        self.timer = self.create_timer(1/self.frequency, self.timer_callback)


    def timer_callback(self):
        """
        Continuously running timer callback.

        Uses the state machine to detect when data for an oncoming puck as been received, select
        waypoints for TrajCalc, and receive trajectories to send to the API.

        Returns
        -------
        None
        """