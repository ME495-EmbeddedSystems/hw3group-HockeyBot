import rclpy
from rclpy.node import Node
import numpy as np
import matplotlib.pyplot as plt
from geometry_msgs.msg import PointStamped, Point




class TrajCalc(Node):
    """
    Trajectory planning and optional execution.

    This node receives a starting position and an end goal position of the end effector, plans the
    path to the end goal configuration and then executes the path with the help of different
    services. It can also dynamically add a box object to the planning scene. It does not execute
    trajectories which lead to collisions.
    """

    def __init__(self):
        """
        Initialize services, clients, and subscribers.

        Also initializes class variables for creating a box and initial robot joint states.
        """
        super().__init__("trajectory_calculations")

        # Publishers
        self.pub_wp1 = self.create_publisher(PointStamped, '/waypoint1', 10)
        self.wp1 = PointStamped()
        self.pub_wp2 = self.create_publisher(PointStamped, '/waypoint2', 10)
        self.wp2 = PointStamped()

        # Subscribers
        self.sub_p1 = self.create_subscription(
            Point, '/puck1_position', self.update_puck_p1, 10)
        self.p1 = Point()  # Puck frame 1 position
        self.sub_p2 = self.create_subscription(
            Point, '/puck2_position', self.update_puck_p2, 10)
        self.p2 = Point()  # Puck frame 2 position

        # Timers
        self.create_timer(0.1, self.timer_callback)

        # Planar workspace of arm relative to robot base while playing
        self.xmin = -0.3
        self.xmax = 0.29
        self.ymin = 0.44
        self.ymax = 0.71

        # Airhockey table 4 corners
        self.Table_xmin = -0.35
        self.Table_xmax = 0.35
        self.Table_ymin = 0.30
        self.Table_ymax = 1.85

        # Robot arm waypoints 1 and 2 y-values
        self.wx1 = 0.0
        self.wy1 = 0.45
        self.wz1 = 0.0
        self.wx2 = 0.0
        self.wy2 = 0.7
        self.wz2 = 0.0

        # Previous values for waypoints x coordinates
        self.wx1_prev = 20
        self.wx2_prev = 20

        # Waypoints to be published
        self.wp1.point.x = self.wx1
        self.wp1.point.y = self.wy1
        self.wp1.point.z = self.wz1
        self.wp2.point.x = self.wx2
        self.wp2.point.y = self.wy2
        self.wp2.point.z = self.wz2

        # Two puck center positions
        # self.p1 = np.array([0.22,1.6])
        # self.p2 = np.array([0.19,1.5])
        # self.p1 = np.array([0.2,1.6])
        # self.p2 = np.array([0.2,1.5])

        self.not_straight_traj = 0
        self.Slope_Zero = 0
        # Random puck centre coordinates
        # p1 = np.array([0.0,0.0])
        # p2 = np.array([0.0,0.0])
        # p1[0] = random.uniform(-0.25,0.25)
        # p2[0] = random.uniform(-0.25,0.25)
        # p1[1] = random.uniform(1.2,1.6)
        # p2[1] = random.uniform(1.2,1.6)

    def update_puck_p1(self, data):
        """
        Get puck frame 1 coordinates 

        Subscribtion topic: /puck1_position
        Args:
            data ( geometry_msgs/msg/Point: Contains the x,y,z coordinates of the puck at frame 1

        Returns
        -------
            None

        """
        self.p1 = data

    def update_puck_p2(self, data):
        """
        Get puck frame 2 coordinates 

        Subscribtion topic: /puck2_position
        Args:
            data ( geometry_msgs/msg/Point: Contains the x,y,z coordinates of the puck at frame 2

        Returns
        -------
            None

        """
        self.p2 = data

    def traj_puck(self):
        """
        Args:
            + c1, c2 - x and y coordinates of the puck at two different locations
        Returns:
            + c - y axis intersection
            + m - Trajectory slope
        """
        if self.c1[1] == self.c2[1]:
            self.m = 0
        else:
            self.m = (self.c1[1]-self.c2[1])/(self.c1[0]-self.c2[0])

        self.c = self.c2[1]-self.m*self.c2[0]

    def play_waypoints(self):
        """
        Calculates the x coordinates of the two waypoints for the robot

        Returns:
            + wx1, wx2 - x coordinates fot robots two waypoints
        """
        self.wx2 = (self.wy2-self.c)/self.m
        self.wx1 = (self.wy1-self.c)/self.m

    def trajectory(self):
        # p1[0] = p2[0] situation where x1 = x2 then m = infinity (Slope)
        if self.p1[0] == self.p2[0]:
            self.Slope_Zero = 1
            self.wx1 = self.p1[0]
            self.wx2 = self.wx1
            self.m = 0
            self.c = np.inf
            self.draw_2D_sim()
        else:
            self.traj_puck()
            self.play_waypoints()

            if self.wx1 > self.xmin and self.wx1 < self.xmax:
                if self.wx2 > self.xmin and self.wx2 < self.xmax:
                    print(f"self.wx1 & self.wx2 inside workspace!!")
                    # not_straight_traj = 1
            else:
                print(f"self.wx1 or self.wx2 outside workspace!!")

            self.draw_2D_sim()

        print(f"\n m = {self.m}, c = {self.c}, wx1 = {self.wx1}, wx2 = {self.wx2}")

    def draw_2D_sim(self):
        """Draw a 2D representation of the airhockey table and the trajectory."""
        # Trajectory - Y-axis intercect
        plt.plot([0],[0.405], 'o', color = 'pink', label = 'Robot home location')
        # Two puck points from CV
        plt.plot([self.p1[0],self.p2[0]],[self.p1[1],self.p2[1]], 'ro', color = 'red', label = 'CV puck centres')
        # Intersect at boundary W2 & W1
        plt.plot([self.wx1,self.wx2],[self.wy1,self.wy2], 'ro', color = 'green', label = 'Robot move waypoints')
        # Trajectory - Y-axis intercect
        plt.plot([0],[self.c], 'x', color = 'red', label = 'Traj intercect with y-axis')
        # Puck trajectory line
        if self.Slope_Zero == 0:
            plt.axline((self.p1[0], self.p1[1]), slope=self.m, color="blue", linestyle=(0, (5, 5)), label = 'Puck trajectory line')
        else:
            plt.plot([self.p1[0],self.wx1],[self.p1[1],self.wy1], color = 'blue', linestyle=(0, (5, 5)), label = 'Puck trajectory line')
        # Table boundaries
        plt.plot([self.Table_xmin,self.Table_xmin],[self.Table_ymin,self.Table_ymax], color="black", label = 'Table boundary')
        plt.plot([self.Table_xmax,self.Table_xmax],[self.Table_ymin,self.Table_ymax], color="black")
        plt.plot([self.Table_xmin,self.Table_xmax],[self.Table_ymin,self.Table_ymin], color="black")
        plt.plot([self.Table_xmin,self.Table_xmax],[self.Table_ymax,self.Table_ymax], color="black")
        # Robot workspace
        plt.plot([self.xmin,self.xmin],[self.ymin,self.ymax], color="orange", label = 'Robot workspace')
        plt.plot([self.xmax,self.xmax],[self.ymin,self.ymax], color="orange")
        plt.plot([self.xmin,self.xmax],[self.ymin,self.ymin], color="orange")
        plt.plot([self.xmin,self.xmax],[self.ymax,self.ymax], color="orange")
        # Center lines of table
        plt.plot([0,0],[self.Table_ymin,self.Table_ymax], '--', color = 'grey', label = 'Table center lines') # Center y-axis
        plt.plot([self.Table_xmin,self.Table_xmax],[(self.Table_ymax+self.Table_ymin)/2,(self.Table_ymax+self.Table_ymin)/2], '--', color = 'grey') # Center x-axis
        # Robot axis and plot axis
        plt.plot([0,0],[0,0.2], color = 'black', label = 'Robot axis') # Robot y-axis
        plt.plot([0,0.2],[0,0], color = 'black') # Robot x-axis
        plt.axis([-1, 1, -0.1, 1.9]) # Window size
        # plt.legend(loc="upper right")
        plt.ylabel("Robot Y-axis ")
        plt.xlabel("Robot X-axis ")
        plt.show()

    def timer_callback(self):

        if self.wx1 == self.wx1_prev and self.wx2 == self.wx2_prev:
            # Publish waypoints
            self.pub_wp1(self.wp1)
            self.pub_wp2(self.wp2)
        else:
            # New header timestamp
            self.wp1.header.stamp = self.get_clock().now().to_msg()
            self.wp2.header.stamp = self.get_clock().now().to_msg()
            # Set x positions for waypoints
            self.wp1.point.x = self.wx1
            self.wp2.point.x = self.wx2
            # Publish waypoints
            self.pub_wp1.publish(self.wp1)
            self.pub_wp2.publish(self.wp2)

def traj_calc_entry(args=None):
    """Run TrajCalc node."""
    rclpy.init(args=args)
    node = TrajCalc()
    rclpy.spin(node)
    rclpy.shutdown()
