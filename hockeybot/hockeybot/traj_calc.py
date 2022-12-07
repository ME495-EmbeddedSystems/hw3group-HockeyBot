# TODO
import rclpy
from rclpy.node import Node
import numpy as np
import matplotlib.pyplot as plt
from geometry_msgs.msg import PointStamped, Point, PoseArray, Pose




class TrajCalc(Node):
    """
    TODO
    Trajectory planning and optional execution.

    This node receives a starting position and an end goal position of the end effector, plans the
    path to the end goal configuration and then executes the path with the help of different
    services. It can also dynamically add a box object to the planning scene. It does not execute
    trajectories which lead to collisions.
    """


# check for x,y of p1,p2 to be 0!!!!

    def __init__(self):
        """
        TODO
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
        self.sub_p = self.create_subscription(
            PoseArray, '/puck_position', self.update_puck_p, 10)
        self.puck_array = PoseArray() # Contains puck 1 and 2 coordinates
        self.p1 = Point()  # Puck frame 1 position
        self.p2 = Point()  # Puck frame 2 position

        # Timers
        self.create_timer(0.1, self.timer_callback)

        # Planar workspace of arm relative to robot base while playing
        self.xmin = -0.3
        self.xmax = 0.3
        self.ymin = 0.42 # Change from 0.44
        self.ymax = 0.70 # Change from 0.71

        # Puck Radius
        self.Puck_radius = 0.063/2

        # Airhockey table 4 corners [meters] - Minus this with puck radius for
        # collision detection - Centre of Puck movement space
        self.Table_xmin = -(0.345 - self.Puck_radius)
        self.Table_xmax = 0.32 - self.Puck_radius
        self.Table_ymin = 0.3683 + self.Puck_radius
        self.Table_ymax = 1.7526 - self.Puck_radius
        # Real table inside
        self.Real_Table_xmin = -0.345
        self.Real_Table_xmax = 0.345
        self.Real_Table_ymin = 0.3683
        self.Real_Table_ymax = 1.7526

        """ New Var """
        self.start = 0 # TODO Delete when using CV
        # Amount of impacts
        self.impact_count = 0
        # y-axis intercept
        self.c_list = np.array([])
        # Slope
        self.m_list = np.array([])
        # After first impact
        self.ImpX = np.array([0]) # Impact point X
        self.ImpY = np.array([0]) # Impact point Y
        # Added 0 to start just to make same length as m_list
        # Flag for collision in the workspace
        self.Flag_collision_WS = 0
        self.Flag_BLOCK = 0
        self.y_ws_impact_Right = 0
        self.y_ws_impact_Left = 0
        self.collision = True

        # Robot arm waypoints 1 and 2 y-values
        # self.wx1 = 0.0
        self.wx1 = np.array([])
        self.wy1 = 0.45
        self.wz1 = 0.0
        # self.wx2 = 0.0
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

        # # Two puck center positions
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


    def update_puck_p(self, data):
        """
        Get puck frame 1 and 2 coordinates 

        Subscribtion topic: /puck_position
        Args:
            data ( geometry_msgs/msg/PoseArray: Contains the x,y,z coordinates of the puck at frame
                1 and 2

        Returns
        -------
            None

        """
        self.puck_array = data
        self.p1 = self.puck_array.poses[0].position
        self.p2 = self.puck_array.poses[1].position
        # # Two puck center positions # TODO Delete when using CV
        # if self.start == 0:
        #     self.start = 1
        #     self.p1.x = 0
        #     self.p2.x = 0
        #     self.p1.y = 0
        #     self.p2.y = 0
        # else:
        #     self.p1.x = 0.22
        #     self.p2.x = 0.19
        #     self.p1.y = 1.6
        #     self.p2.y = 1.5

    def traj_puck(self):
        """
        Args:
            + c1, c2 - x and y coordinates of the puck at two different locations
        Returns:
            + c - y axis intersection
            + m - Trajectory slope
        """
        if self.p1.x == self.p2.x:
            self.m = 0
        else:
            self.m = (self.p1.y-self.p2.y)/(self.p1.x-self.p2.x)

        self.c = self.p2.y-self.m*self.p2.x

    def after_impact_traj_puck(self,c1,m):
        """ Traj y-intersect calc after impact."""
        c = c1[1]-m*c1[0]
        return c

    def play_waypoints(self):
        """
        Calculates the x coordinates of the two waypoints for the robot

        Returns:
            + wx1, wx2 - x coordinates fot robots two waypoints
        """
        self.wx2 = np.append(self.wx2, (self.wy2-self.c_list[self.impact_count])/self.m_list[self.impact_count])
        self.wx1 = np.append(self.wx1, (self.wy1-self.c_list[self.impact_count])/self.m_list[self.impact_count])

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

            while self.collision == True:

                y_xmin = self.mf*self.Table_xmin + self.cf
                y_xmax = self.mf*self.Table_xmax + self.cf

                self.play_waypoints()

                if self.Flag_collision_WS == 1 or self.impact_count > 5:
                    break # Immediately stops while loop

                # print(f"y_xmin = {y_xmin}, y_xmax = {y_xmax}")

                # print(f"\n clist = {c_list}, m_list = {m_list} \n")


                if y_xmin < self.Table_ymax and y_xmin > self.Table_ymin and y_xmin < y_xmax:

                    # Check for collisions in the ws
                    if y_xmin < self.ymax and y_xmin > self.ymin and y_xmin < y_xmax:
                        # ImpX[impact_count]
                        # ImpY[impact_count]
                        # Calculate new y in workspace
                        self.y_ws_impact_Left = self.xmin*self.m_list[self.impact_count] \
                            + self.c_list[self.impact_count]
                        # print(f"y_ws left = {y_ws_impact_Left}")

                        # y_xmin

                        # break
                        self.collision = False

                        # print("Left collision in ws")
                        self.Flag_collision_WS = 1
                    else:
                        self.impact_count += 1

                        self.m_list = np.append(self.m_list,-self.m_list[self.impact_count-1]) # Reflect impact angle
                        # print(f"\n clist = {c_list}, m_list = {m_list} \n")
                        # Impact coordinates
                        self.ImpX = np.append(self.ImpX, self.Table_xmin)
                        self.ImpY = np.append(self.ImpY, y_xmin)
                        # Call trajectory calculation for new trajectory y-axis intercept
                        self.c_list = np.append(self.c_list, self.after_impact_traj_puck([self.ImpX[self.impact_count],\
                            self.ImpY[self.impact_count]],self.m_list[self.impact_count]))
                        # Set current c,m equal to final cf and mf
                        self.cf = self.c_list[self.impact_count]
                        self.mf = self.m_list[self.impact_count]

                        # print(" Wall collision left")
                        self.collision = True # TODO: Add back in so that collisions will keep on be corrected
                elif y_xmax < self.Table_ymax and y_xmax > self.Table_ymin and y_xmax < y_xmin:
                    if y_xmax < self.ymax and y_xmax > self.ymin and y_xmax < y_xmin:

                        # ImpX[impact_count]
                        # ImpY[impact_count]
                        # Calculate new y in workspace
                        self.y_ws_impact_Right = self.xmax*self.m_list[self.impact_count] \
                            + self.c_list[self.impact_count]
                        # print(f"y_ws right = {y_ws_impact_Right}")

                        # y_xmin

                        # break
                        self.collision = False

                        print("Rigth collision in ws")
                        self.Flag_collision_WS = 1
                    else:
                        self.impact_count += 1

                        self.m_list = np.append(self.m_list,-self.m_list[self.impact_count-1]) # Reflect impact angle
                        # print(f"\n clist = {c_list}, m_list = {m_list} \n")
                        # Impact coordinates
                        self.ImpX = np.append(self.ImpX, self.Table_xmax)
                        self.ImpY = np.append(self.ImpY, y_xmax)
                        # Call trajectory calculation for new trajectory y-axis intercept
                        self.c_list = np.append(self.c_list, self.after_impact_traj_puck([self.ImpX[self.impact_count], self.ImpY[self.impact_count]],self.m_list[self.impact_count]))
                        # Set current c,m equal to final cf and mf
                        self.cf = self.c_list[self.impact_count]
                        self.mf = self.m_list[self.impact_count]


                        # print(" Wall collision right")
                        self.collision = True # TODO: Add back in so that collisions will keep on be corrected

                else: # No collision
                    self.collision = False

        self.get_logger().info(f"wx1 = {self.wx1}, wx2 = {self.wx2}")
        # print(f"clist = {c_list}, m_list = {m_list}")

        # Select best waypoint x values to publish
        for i in range(len(self.wx1)):
            # TODO:
            """[traj_calc-3]   File "/home/ritz/FallQ/Embedded/hw3_ws/build/hockeybot/hockeybot/traj_calc.py", line 303, in trajectory
[traj_calc-3]     for i in range(len(self.wx1)):
[traj_calc-3] TypeError: object of type 'float' has no len()
[ERROR] [traj_calc-3]: process has died [pid 91924, exit code 1, cmd '/home/ritz/FallQ/Embedded/hw3_ws/install/hockeybot/lib/hockeybot/traj_calc --ros-args -r __node:=traj_node'].
"""
            if self.wx1[i] < self.xmax and self.wx1[i] > self.xmin:
                for j in range(len(self.wx2)):
                    if self.wx2[j] < self.xmax and self.wx2[j] > self.xmin:
                        self.wx1_CORRECT = self.wx1[i]
                        self.wx2_CORRECT = self.wx2[i]
                        # print(f" wx1[{i}] = {wx1[i]} ")
                        # print(f" wx2[{j}] = {wx2[j]} ")



        if self.wx1[self.impact_count] > self.xmin and self.wx1[self.impact_count] < self.xmax:
            if self.wx2[self.impact_count] > self.xmin and self.wx2[self.impact_count] < self.xmax:
                print(f"self.wx1 & self.wx2 inside workspace!!")
                # not_straight_traj = 1
        else:
            print(f"self.wx1 or self.wx2 outside workspace!!")


        self.choose_best_waypoints()
        
        self.draw_2D_sim()

        print(f"\n m = {self.m}, c = {self.c}, wx1 = {self.wx1}, wx2 = {self.wx2}")


    def choose_best_waypoints(self):
        # """ Plot best wx intersection """
        for i in range(len(self.wx2)):
            self.Flag_BLOCK = 0
            if self.wx2[i] < self.xmax and self.wx2[i] > self.xmin:
                self.Flag_BLOCK = 1
                # print(f" wx2[{i}] = {wx2[i]} ")
                if self.y_ws_impact_Right == 0 and self.y_ws_impact_Left == 0:
                    for j in range(len(self.wx1)):
                        # print(f" wx1[{j}] = {wx1[j]} ")
                        if self.wx1[j] < self.xmax and self.wx1[j] > self.xmin:
                            # Intersect at boundary W2 & W1
                            # Value to be published
                            self.wx1_CORRECT = self.wx1[j]
                            self.wy1_CORRECT = self.wy1
                            self.wx2_CORRECT = self.wx2[i]
                            self.wy2_CORRECT = self.wy2
                            # plt.plot([wx1[j],wx2[i]],[wy1,wy2], 'ro', color = 'green', label = 'Robot move waypoints')
                            # print(f" w1 = {wx1[i],wy1}, w2 = {wx2[i],wy2} ")
                else:
                    if self.y_ws_impact_Right != 0:
                        # print("RIIIIGHT")
                        # Value to be published
                        self.wx1_CORRECT = self.xmax
                        self.wy1_CORRECT = self.y_ws_impact_Right
                        self.wx2_CORRECT = self.wx2[i]
                        self.wy2_CORRECT = self.wy2
                        # plt.plot([xmax,wx2[i]],[y_ws_impact_Right,wy2], 'ro', color = 'green', label = 'Robot move waypoints')
                        # print(f" w1 = {xmax,y_ws_impact_Right}, w2 = {wx2[i],wy2} ")
                    elif self.y_ws_impact_Left != 0:
                        # print("LEEEEEFTT")
                        # Value to be published
                        self.wx1_CORRECT = self.xmin
                        self.wy1_CORRECT = self.y_ws_impact_Left
                        self.wx2_CORRECT = self.wx2[i]
                        self.wy2_CORRECT = self.wy2
                        # plt.plot([xmin,wx2[i]],[y_ws_impact_Left,wy2], 'ro', color = 'green', label = 'Robot move waypoints')
                        # print(f" w1 = {xmin,y_ws_impact_Left}, w2 = {wx2[i],wy2} ")
        if self.Flag_BLOCK == 0: # Block/Stay in front of goal
            # Value to be published
            self.wx1_CORRECT = 0.0
            self.wy1_CORRECT = 0.407
            self.wx2_CORRECT = 0.0
            self.wy2_CORRECT = 0.407
            self.get_logger().info(f"___________ JUST BLOCK ____________ ")
            print(f"JUST BLOCK")




    def draw_2D_sim(self):
        """Draw a 2D interactive representation of the airhockey table and the trajectory."""
        # Interactive plot
        plt.ion()
        
        # Table boundaries
        plt.plot([self.Table_xmin,self.Table_xmin],[self.Table_ymin,self.Table_ymax], '--', color="grey", label = 'Table boundary for puck centre - Collision')
        plt.plot([self.Table_xmax,self.Table_xmax],[self.Table_ymin,self.Table_ymax], '--', color="grey")
        plt.plot([self.Table_xmin,self.Table_xmax],[self.Table_ymin,self.Table_ymin], '--', color="grey")
        plt.plot([self.Table_xmin,self.Table_xmax],[self.Table_ymax,self.Table_ymax], '--', color="grey")
        plt.plot([self.Real_Table_xmin,self.Real_Table_xmin],[self.Real_Table_ymin,self.Real_Table_ymax], color="black", label = 'Actual Table boundary')
        plt.plot([self.Real_Table_xmax,self.Real_Table_xmax],[self.Real_Table_ymin,self.Real_Table_ymax], color="black")
        plt.plot([self.Real_Table_xmin,self.Real_Table_xmax],[self.Real_Table_ymin,self.Real_Table_ymin], color="black")
        plt.plot([self.Real_Table_xmin,self.Real_Table_xmax],[self.Real_Table_ymax,self.Real_Table_ymax], color="black")
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




        """ NEW """
        # Plot Robot Waypoints
        plt.plot([self.wx1_CORRECT,self.wx2_CORRECT],[self.wy1_CORRECT,self.wy2_CORRECT], 'ro', color = 'green', label = 'Robot move waypoints')
       
        """Plot after impact the dashed trajectory"""
        if len(self.c_list) != 0:
            for imp in range(1,len(self.c_list)):
                # Impact point 1
                plt.plot([self.ImpX[imp]],[self.ImpY[imp]], 'ro', color = 'blue', label = 'Impact point')
                # New after impact SOLID trajectory line only in airhockey task space
                # plt.plot([ImpX[imp],ImpX[imp]],[ImpY[imp],ImpY[imp]], color = 'blue', label = 'New puck trajectory line')
                # New after impact trajectory line- DASHED
                # plt.axline((self.ImpX[imp],self.ImpY[imp]), slope=self.m_list[imp], color="red", linestyle=(0, (5, 5)), label = 'New puck trajectory line')
                # Trajectory - Y-axis intercect
                plt.plot([0],[self.c_list[imp]], 'x', color = 'red', label = 'Traj1 intercect with y-axis')
        
        """ Plot only in airhockey table the trajectory"""
        if len(self.ImpX) >= 2:
            # Puck cv point to first impact
            plt.plot([self.p1.x,self.p2.x],[self.p1.y,self.p2.y], ':', color = 'darkviolet', label = 'New puck trajectory line')
            plt.plot([self.p2.x,self.ImpX[1]],[self.p2.y,self.ImpY[1]], ':', color = 'darkviolet', label = 'New puck trajectory line')
            for imp in range(2,len(self.ImpX)):
                # New after impact SOLID trajectory line only in airhockey task space
                plt.plot([self.ImpX[imp-1],self.ImpX[imp]],[self.ImpY[imp-1],self.ImpY[imp]], ':', color = 'darkviolet', label = 'New puck trajectory line')
            # Last impact to robot waypoints
            plt.plot([self.ImpX[-1],self.wx1_CORRECT],[self.ImpY[-1],self.wy1_CORRECT], ':', color = 'darkviolet', label = 'New puck trajectory line')
            plt.plot([self.ImpX[-1],self.wx2_CORRECT],[self.ImpY[-1],self.wy2_CORRECT], ':', color = 'darkviolet', label = 'New puck trajectory line')
        else: # No impacts
            plt.plot([self.p1.x,self.wx1_CORRECT],[self.p1.y,self.wy1_CORRECT], ':', color = 'darkviolet', label = 'New puck trajectory line')
            plt.plot([self.p2.x,self.wx2_CORRECT],[self.p2.y,self.wy2_CORRECT], ':', color = 'darkviolet', label = 'New puck trajectory line')
        
        # Trajectory - Y-axis intercect
        plt.plot([0],[0.405], '8', color = 'pink', label = 'Robot home location')
        # Two puck points from CV
        plt.plot([self.p1.x,self.p2.x],[self.p1.y,self.p2.y], 'ro', color = 'red', label = 'CV puck centres')

        # # Intersect at boundary W2 & W1 # TODO commented this out
        # plt.plot([self.wx1,self.wx2],[self.wy1,self.wy2], 'ro', color = 'green', label = 'Robot move waypoints')

        # Trajectory - Y-axis intercect
        plt.plot([0],[self.c], 'x', color = 'red', label = 'Traj intercect with y-axis')

        # Puck trajectory line
        # if self.Slope_Zero == 0:
        #     plt.plot([self.p1.x,self.wx1_CORRECT],[self.p1.y,self.wy2_CORRECT], ':', color = 'darkviolet', label = 'New puck trajectory line')
            # plt.axline((self.p1.x, self.p1.y), slope=self.m, color="blue", linestyle=(0, (5, 5)), label = 'Puck trajectory line')
        # else: # TODO commented this out
        #     plt.plot([self.p1.x,self.wx1],[self.p1.y,self.wy1], color = 'blue', linestyle=(0, (5, 5)), label = 'Puck trajectory line')






        plt.axis([-1, 1, -0.1, 1.9]) # Window size
        # plt.legend(loc="upper right")
        plt.ylabel("Robot Y-axis ")
        plt.xlabel("Robot X-axis ")
        # plt.show()
        # Pause for 0.5 second
        plt.pause(0.5)
        # Clear the plot to not show previous plot
        plt.clf()

    def timer_callback(self):

        if self.p1.y != 0 and self.p2.y != 0:
            # if self.wx1[self.impact_count] == self.wx1_prev and self.wx2[self.impact_count] == self.wx2_prev:
            if self.p1.x == self.wx1_prev and self.p2.x == self.wx2_prev:
                # Publish waypoints
                self.pub_wp1.publish(self.wp1)
                self.pub_wp2.publish(self.wp2)
            else:
                self.trajectory()
                # New header timestamp
                self.wp1.header.stamp = self.get_clock().now().to_msg()
                self.wp2.header.stamp = self.get_clock().now().to_msg()
                # Set x positions for waypoints
                # self.wp1.point.x = self.wx1[self.impact_count]
                # self.wp2.point.x = self.wx2[self.impact_count]
                self.wp1.point.x = self.wx1_CORRECT
                self.wp2.point.x = self.wx2_CORRECT
                self.wp1.point.y = self.wy1_CORRECT # NEW TODO
                self.wp2.point.y = self.wy2_CORRECT # NEW TODO
                self.get_logger().info(f"Waypoint1x -  {self.wp1.point.x}")
                self.get_logger().info(f"Waypoint2x -  {self.wp2.point.x}")
                self.get_logger().info(f"Waypoint1y -  {self.wp1.point.y}")
                self.get_logger().info(f"Waypoint2y -  {self.wp2.point.y}")

                # Publish waypoints
                self.pub_wp1.publish(self.wp1)
                self.pub_wp2.publish(self.wp2)
                # Set previous values
                # self.wx1_prev = self.wx1[self.impact_count]
                # self.wx2_prev = self.wx2[self.impact_count]
                self.wx1_prev = self.p1.x
                self.wx2_prev = self.p2.x

                """ New Var """
                # Amount of impacts
                self.impact_count = 0
                # y-axis intercept
                self.c_list = np.array([])
                # Slope
                self.m_list = np.array([])
                # After first impact
                self.ImpX = np.array([0]) # Impact point X
                self.ImpY = np.array([0]) # Impact point Y
                # Added 0 to start just to make same length as m_list
                # Flag for collision in the workspace
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
