"""
Calculates the predicted trajectory of the puck and the play waypoints for the robot.

It handles collisions by reflecting the impact angle about the normal line. The waypoints for the
the robot to hit the puck is constrained in the robots workspace on the air hockey table. The most
optimal waypoints are selected by considering all four sides of the robots workspace. The robot will
then move to the first waypoint that is on the predicted trajectory line of the puck and then move
along the line to the second wayoint and hit the puck. A plot is dynamically generated and updated
each time a new trajectory is calculated. The robot blocks if the trajectory is out of the
workspace and unreachable.

PUBLISHERS:
    + /waypoint1 (geometry_msgs/msg/PointStamped) - Publishes the first position for the trajectory
    of the robot to hit the puck
    + /waypoint2 (geometry_msgs/msg/PointStamped) - Publishes the second position for the
    trajectory of the robot to hit the puck
SUBSCRIPTIONS:
    + /puck_position (geometry_msgs/msg/PoseArray) - Recieves a list of two puck positions to
    calculate a predicted trajectory for the puck
"""
import rclpy
from rclpy.node import Node
import numpy as np
import matplotlib.pyplot as plt
from geometry_msgs.msg import PointStamped, Point, PoseArray


def after_impact_traj_puck(impact, m):
    """
    Calculate y-intersept for after impact trajectory lines.

    Args:
    ----
        + impact (float) : Impact coordinates against the air airhockey table
        + m (float) : Reflected slope, angle, at impact

    Return:
    ------
        + c (float) : New trajectory y-intercept

    """
    c = impact[1]-m*impact[0]
    return c


class TrajCalc(Node):
    """Node for trajectory prediction, visualization and robot play waypoint calculations."""

    def __init__(self):
        """Initialize publishers, subscribers, timer and variables."""
        super().__init__("trajectory_calculations")

        # Publishers
        self.pub_wp1 = self.create_publisher(PointStamped, '/waypoint1', 10)
        self.wp1 = PointStamped()
        self.pub_wp2 = self.create_publisher(PointStamped, '/waypoint2', 10)
        self.wp2 = PointStamped()

        # Subscribers
        self.sub_puck_position = self.create_subscription(
            PoseArray, '/puck_position', self.update_puck_position, 10)
        self.puck_array = PoseArray()  # Contains the coordinates of the two puck positions
        self.p1 = Point()  # Puck first position
        self.p2 = Point()  # Puck second position

        # Timers
        self.create_timer(0.1, self.timer_callback)

        # Planar workspace of arm relative to robot base while playing
        self.xmin = -0.3
        self.xmax = 0.3
        self.ymin = 0.42
        self.ymax = 0.70

        # Puck Radius
        self.Puck_radius = 0.063/2

        # Airhockey table 4 corners [meters] - Minus the puck radius for
        # collision detection - Centre of Puck movement space
        self.Table_xmin = -(0.345 - self.Puck_radius)
        self.Table_xmax = 0.345 - self.Puck_radius
        self.Table_ymin = 0.3683 + self.Puck_radius
        self.Table_ymax = 1.7526 - self.Puck_radius
        # Real table inside corners
        self.Real_Table_xmin = -0.345
        self.Real_Table_xmax = 0.345
        self.Real_Table_ymin = 0.3683
        self.Real_Table_ymax = 1.7526

        # Amount of impacts
        self.impact_count = 0
        # y-axis intercept
        self.c_list = np.array([])
        # Slope
        self.m_list = np.array([])
        # List of all the impact points coordinates for one predicted trajectory
        self.ImpX = np.array([0])
        self.ImpY = np.array([0])

        # Flag for collision in the workspace
        self.Flag_collision_WS = 0
        self.Flag_BLOCK = 0
        self.y_ws_impact_Right = 0
        self.y_ws_impact_Left = 0
        self.collision = True
        self.not_straight_traj = 0
        self.Slope_Zero = 0

        # Robot arm waypoint 1 and 2 coordinates
        self.wx1 = np.array([])
        self.wy1 = 0.45
        self.wz1 = 0.0
        self.wx2 = np.array([])
        self.wy2 = 0.7
        self.wz2 = 0.0

        # x and y waypoints to publish
        self.wx1_CORRECT = 0
        self.wy1_CORRECT = self.ymin
        self.wx2_CORRECT = 0
        self.wy2_CORRECT = self.ymax

        # Previous values for waypoints x coordinates
        self.wx1_prev = 20
        self.wx2_prev = 20

        # Waypoints to be published
        self.wp1.point.x = 0.0
        self.wp1.point.y = self.wy1
        self.wp1.point.z = self.wz1
        self.wp2.point.x = 0.0
        self.wp2.point.y = self.wy2
        self.wp2.point.z = self.wz2

    def update_puck_position(self, data):
        """
        Get puck frame 1 and 2 coordinates.

        Subscribtion topic: /puck_position
        Args:
        ----
            + data : geometry_msgs/msg/PoseArray: Contains the x,y,z coordinates of the puck at
            frame 1 and 2
        Return:
        ------
            + None

        """
        self.puck_array = data
        self.p1 = self.puck_array.poses[0].position
        self.p2 = self.puck_array.poses[1].position

    def traj_puck(self):
        """
        Claculate pucks predicted trajectory.

        Calculate trajectory of by fitting a straight line through two points, self.p1 and
        salf.p2, by calculating the slope of the line, self.m, and the y-intercept of the line,
        self.c.
        """
        if self.p1.x == self.p2.x:
            self.m = 0
        else:
            self.m = (self.p1.y-self.p2.y)/(self.p1.x-self.p2.x)

        self.c = self.p2.y-self.m*self.p2.x

    def after_impact_traj_puck(self, impact, m):
        """
        Calculate y-intersect for after impact trajectory lines.

        Args:
        ----
            + impact (float) : Impact coordinates against the air airhockey table
            + m (float) : Reflected slope, angle, at impact
        Returns:
        -------
            + c (float) : New trajectory y-intercept

        """
        c = impact[1]-m*impact[0]
        return c

    def play_waypoints(self):
        """
        Calculate x-coordinates for robot play points.

        Calculate the x coordinates, self.wx1 and self.wx2, of the two waypoints for the robot to
        hit the puck along its predicted trajectory line.
        """
        self.wx2 = np.append(self.wx2, (self.wy2 -
                             self.c_list[self.impact_count])/self.m_list[self.impact_count])
        self.wx1 = np.append(self.wx1, (self.wy1 -
                             self.c_list[self.impact_count])/self.m_list[self.impact_count])

    def trajectory(self):
        # p1[0] = p2[0] situation where x1 = x2 then m = infinity (Slope)
        if self.p1.x == self.p2.x:
            self.Slope_Zero = 1
            self.wx1 = np.array([self.p1.x])
            self.wx2 = self.wx1
            self.m = 0
            self.c = np.inf
            self.draw_2D_sim()
        else:
            self.traj_puck()
            self.m_list = np.append(self.m_list, self.m)
            self.c_list = np.append(self.c_list, self.c)
            self.mf = self.m
            self.cf = self.c

            while self.collision is True:

                y_xmin = self.mf*self.Table_xmin + self.cf
                y_xmax = self.mf*self.Table_xmax + self.cf
                self.play_waypoints()

                if self.Flag_collision_WS == 1 or self.impact_count > 5:
                    break  # Immediately stops while loop

                if y_xmin < self.Table_ymax and y_xmin > self.Table_ymin and y_xmin < y_xmax:
                    # Check for collisions in the ws
                    if y_xmin < self.ymax and y_xmin > self.ymin and y_xmin < y_xmax:
                        # Calculate new y in workspace
                        self.y_ws_impact_Left = self.xmin*self.m_list[self.impact_count] \
                            + self.c_list[self.impact_count]
                        self.collision = False
                        self.Flag_collision_WS = 1
                    else:
                        # Upddate number of impacts with table
                        self.impact_count += 1
                        # Reflect impact angle
                        self.m_list = np.append(self.m_list, -self.m_list[self.impact_count-1])
                        # Impact coordinates
                        self.ImpX = np.append(self.ImpX, self.Table_xmin)
                        self.ImpY = np.append(self.ImpY, y_xmin)
                        # Call trajectory calculation for new trajectory y-axis intercept
                        self.c_list = np.append(self.c_list, self.after_impact_traj_puck(
                                               [self.ImpX[self.impact_count],
                                                self.ImpY[self.impact_count]],
                                               self.m_list[self.impact_count]))
                        # Set current c,m equal to final cf and mf
                        self.cf = self.c_list[self.impact_count]
                        self.mf = self.m_list[self.impact_count]
                        self.collision = True

                elif y_xmax < self.Table_ymax and y_xmax > self.Table_ymin and y_xmax < y_xmin:
                    if y_xmax < self.ymax and y_xmax > self.ymin and y_xmax < y_xmin:
                        # Calculate new y in workspace
                        self.y_ws_impact_Right = self.xmax*self.m_list[self.impact_count] \
                            + self.c_list[self.impact_count]
                        # Set a flag to indicate that there was a wall collision
                        self.collision = False
                        self.Flag_collision_WS = 1
                    else:
                        self.impact_count += 1
                        # Reflect impact angle
                        self.m_list = np.append(self.m_list, -self.m_list[self.impact_count-1])
                        # Impact coordinates
                        self.ImpX = np.append(self.ImpX, self.Table_xmax)
                        self.ImpY = np.append(self.ImpY, y_xmax)
                        # Call trajectory calculation for new trajectory y-axis intercept
                        self.c_list = np.append(self.c_list, self.after_impact_traj_puck(
                                               [self.ImpX[self.impact_count],
                                                self.ImpY[self.impact_count]],
                                               self.m_list[self.impact_count]))
                        # Set current c,m equal to final cf and mf
                        self.cf = self.c_list[self.impact_count]
                        self.mf = self.m_list[self.impact_count]
                        # Set a flag to indicate that there was a wall collision
                        self.collision = True

                else:  # No collision with the walls
                    self.collision = False

        # Select best waypoint x values to publish
        for i in range(len(self.wx1)):
            if self.wx1[i] < self.xmax and self.wx1[i] > self.xmin:
                for j in range(len(self.wx2)):
                    if self.wx2[j] < self.xmax and self.wx2[j] > self.xmin:
                        self.wx1_CORRECT = self.wx1[i]
                        self.wx2_CORRECT = self.wx2[i]

        self.choose_best_waypoints()
        self.draw_2D_sim()

    def choose_best_waypoints(self):
        """
        Select best waypoints for robot to hit puck.

        Select best waypoints for the robot in its workspace from two lists of all possible
        waypoints in the one predicted trajectory. If the waypoints are outside the robots
        workspace then the robot will block by setting the waypoints to the home position in front
        of the goal.
        """
        for i in range(len(self.wx2)):
            self.Flag_BLOCK = 0
            if self.wx2[i] < self.xmax and self.wx2[i] > self.xmin:
                self.Flag_BLOCK = 1
                if self.y_ws_impact_Right == 0 and self.y_ws_impact_Left == 0:
                    for j in range(len(self.wx1)):
                        if self.wx1[j] < self.xmax and self.wx1[j] > self.xmin:
                            # Intersect at front and back workspace boundaries
                            self.wx1_CORRECT = self.wx1[j]
                            self.wy1_CORRECT = self.wy1
                            self.wx2_CORRECT = self.wx2[i]
                            self.wy2_CORRECT = self.wy2
                else:
                    if self.y_ws_impact_Right != 0:
                        # Intersect at right workspace boundary
                        self.wx1_CORRECT = self.xmax
                        self.wy1_CORRECT = self.y_ws_impact_Right
                        self.wx2_CORRECT = self.wx2[i]
                        self.wy2_CORRECT = self.wy2
                    elif self.y_ws_impact_Left != 0:
                        # Intersect at left workspace boundary
                        self.wx1_CORRECT = self.xmin
                        self.wy1_CORRECT = self.y_ws_impact_Left
                        self.wx2_CORRECT = self.wx2[i]
                        self.wy2_CORRECT = self.wy2
        if self.Flag_BLOCK == 0:
            # Block/Stay in front of goal
            self.wx1_CORRECT = 0.0
            self.wy1_CORRECT = 0.407
            self.wx2_CORRECT = 0.0
            self.wy2_CORRECT = 0.407
            self.get_logger().info("___________ BLOCK ____________ ")

        if self.Flag_BLOCK == 1:
            self.get_logger().info("___________ HIT ____________ ")

    def draw_2D_sim(self):
        """
        Plot the predicted trajectory.

        Plot a dynamic 2D representation of the airhockey table, the predicted trajectory,
        the robots play waypoints and all the impacts.
        """
        # Interactive plot
        plt.ion()
        # Collision boundary
        plt.plot([self.Table_xmin, self.Table_xmin], [self.Table_ymin, self.Table_ymax], '--',
                 color="grey", label='Puck collision boundary')
        plt.plot([self.Table_xmax, self.Table_xmax], [self.Table_ymin, self.Table_ymax], '--',
                 color="grey")
        plt.plot([self.Table_xmin, self.Table_xmax], [self.Table_ymin, self.Table_ymin], '--',
                 color="grey")
        plt.plot([self.Table_xmin, self.Table_xmax], [self.Table_ymax, self.Table_ymax], '--',
                 color="grey")
        # Air hockey table inner boundary
        plt.plot([self.Real_Table_xmin, self.Real_Table_xmin],
                 [self.Real_Table_ymin, self.Real_Table_ymax], color="black",
                 label='Airhockey table')
        plt.plot([self.Real_Table_xmax, self.Real_Table_xmax],
                 [self.Real_Table_ymin, self.Real_Table_ymax], color="black")
        plt.plot([self.Real_Table_xmin, self.Real_Table_xmax],
                 [self.Real_Table_ymin, self.Real_Table_ymin], color="black")
        plt.plot([self.Real_Table_xmin, self.Real_Table_xmax],
                 [self.Real_Table_ymax, self.Real_Table_ymax], color="black")
        # Robot workspace
        plt.plot([self.xmin, self.xmin], [self.ymin, self.ymax], color="orange",
                 label='Robot workspace')
        plt.plot([self.xmax, self.xmax], [self.ymin, self.ymax], color="orange")
        plt.plot([self.xmin, self.xmax], [self.ymin, self.ymin], color="orange")
        plt.plot([self.xmin, self.xmax], [self.ymax, self.ymax], color="orange")
        # Center lines of table
        plt.plot([0, 0], [self.Table_ymin, self.Table_ymax], '--', color='grey')
        plt.plot([self.Table_xmin, self.Table_xmax], [(self.Table_ymax+self.Table_ymin)/2,
                 (self.Table_ymax+self.Table_ymin)/2], '--', color='grey')
        # Robot axis
        plt.plot([0, 0], [0, 0.2], color='black', label='Robot axis')
        plt.plot([0, 0.2], [0, 0], color='black')
        # Plot Robot Waypoints
        plt.plot([self.wx1_CORRECT, self.wx2_CORRECT], [self.wy1_CORRECT, self.wy2_CORRECT], 'ro',
                 color='green', label='Robot move waypoints')

        """Plot after impact the dashed trajectory"""
        if len(self.c_list) != 0:
            if len(self.ImpX) > 1:
                plt.plot([self.ImpX[1]], [self.ImpY[1]], 'ro', color='blue',
                         label='Impact point')
            for imp in range(2, len(self.c_list)):
                # Impact point
                plt.plot([self.ImpX[imp]], [self.ImpY[imp]], 'ro', color='blue')
            for imp in range(1, len(self.c_list)):
                # Y-intercept
                plt.plot([0], [self.c_list[imp]], 'x', color='red')

        """ Plot only in airhockey table the trajectory"""
        if len(self.ImpX) >= 2:
            # Puck cv point to first impact
            plt.plot([self.p1.x, self.p2.x], [self.p1.y, self.p2.y], ':', color='darkviolet',
                     label='New puck trajectory line')
            plt.plot([self.p2.x, self.ImpX[1]], [self.p2.y, self.ImpY[1]], ':', color='darkviolet')
            for imp in range(2, len(self.ImpX)):
                # After impact solid trajectory line only in airhockey task space
                plt.plot([self.ImpX[imp-1], self.ImpX[imp]], [self.ImpY[imp-1],
                         self.ImpY[imp]], ':', color='darkviolet')
            # Last impact to robot waypoints
            plt.plot([self.ImpX[-1], self.wx1_CORRECT], [self.ImpY[-1], self.wy1_CORRECT], ':',
                     color='darkviolet')
            plt.plot([self.ImpX[-1], self.wx2_CORRECT], [self.ImpY[-1], self.wy2_CORRECT], ':',
                     color='darkviolet')
        else:  # No side collisions
            plt.plot([self.p1.x, self.wx1_CORRECT], [self.p1.y, self.wy1_CORRECT], ':',
                     color='darkviolet', label='New puck trajectory line')
            plt.plot([self.p2.x, self.wx2_CORRECT], [self.p2.y, self.wy2_CORRECT], ':',
                     color='darkviolet')

        # Robot home position
        plt.plot([0], [0.405], '8', color='pink', label='Robot home location')
        # Two puck points from CV
        plt.plot([self.p1.x, self.p2.x], [self.p1.y, self.p2.y], 'ro', color='red',
                 label='CV puck centres')
        # Trajectory - Y-axis intercept
        plt.plot([0], [self.c], 'x', color='red', label='Trajectory intercept with y-axis')
        # Window size
        plt.axis([-0.5, 1.5, -0.1, 1.9])
        plt.legend(loc="upper right")
        plt.ylabel("Robot Y-axis ")
        plt.xlabel("Robot X-axis ")
        plt.pause(0.5)
        # Clear the plot to not show previous plot
        plt.clf()

    def timer_callback(self):

        if self.p1.y != 0 and self.p2.y != 0:
            if self.p1.x == self.wx1_prev and self.p2.x == self.wx2_prev:
                # Wait for new waypoints
                self.pub_wp1.publish(self.wp1)
                self.pub_wp2.publish(self.wp2)
            else:
                # Calcualate trajectory and waypoints
                self.trajectory()
                self.wp1.header.stamp = self.get_clock().now().to_msg()
                self.wp2.header.stamp = self.get_clock().now().to_msg()

                # Set x and y positions for waypoints
                self.wp1.point.x = self.wx1_CORRECT
                self.wp2.point.x = self.wx2_CORRECT
                self.wp1.point.y = self.wy1_CORRECT
                self.wp2.point.y = self.wy2_CORRECT

                # Publish waypoints
                self.pub_wp1.publish(self.wp1)
                self.pub_wp2.publish(self.wp2)

                # Set previous values
                self.wx1_prev = self.p1.x
                self.wx2_prev = self.p2.x

                # Reset all varibales and flags
                self.impact_count = 0
                self.c_list = np.array([])
                self.m_list = np.array([])
                self.ImpX = np.array([0])
                self.ImpY = np.array([0])
                self.Flag_collision_WS = 0
                self.Flag_BLOCK = 0
                self.y_ws_impact_Right = 0
                self.y_ws_impact_Left = 0
                self.collision = True
                self.not_straight_traj = 0
                self.Slope_Zero = 0
        else:
            self.pub_wp1.publish(self.wp1)
            self.pub_wp2.publish(self.wp2)


def traj_calc_entry(args=None):
    """Run TrajCalc node."""
    rclpy.init(args=args)
    node = TrajCalc()
    rclpy.spin(node)
    rclpy.shutdown()
