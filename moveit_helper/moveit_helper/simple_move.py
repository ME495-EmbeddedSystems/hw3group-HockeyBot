"""
Enables the planning and execution of paths using the move group node.

It can plan a path to a specified pose or just a position or or just an orientation from any start
configuration and it can also dynamically add a box to the planning scene.

SERVERS:
    + /initial_service(moveit_interface/srv/Initial)- Gets the starting configuration (position and
    orientation) of the end effector, response is None
    + /goal_service(moveit_interface/srv/Goal)- Gets the goal pose (position and orientation) of
    the end effector, response is None
    + /execute_service(moveit_interface/srv/Execute)- It has a bool request flag that indicates
    whether a plan should be executed, response is None.
    + /add_obj(moveit_interface/srv/Addobj)- It takes in the box id, position and dimensions of a
    box object to be added to the planning scene, response is None.
CLIENTS:
    + compute_ik(moveit_msgs/srv/GetPositionIK)- Send the goal position of the end effector and
    gets the corresponding joint angles of the manipulator
    + get_planning_scene(moveit_msgs/srv/GetPlanningScene)-Send the components and gets the
    planning scene as response
ACTION CLIENTS:
    + move_action(moveit_msgs/action/MoveGroup) - Send the plan request message and returns the
    trajectory path.
    + execute_trajectory(moveit_msgs/action/ExecuteTrajectory) - Send trajectory path message and
    the action then executes the trajectory.
PUBLISHERS:
    + /planning_scene (moveit_msgs/msg/PlanningScene) - Publishes the box object to the planning
    scene
SUBSCRIPTIONS:
    + /joint_states (sensor_msgs/msg/JointStates) - Gets the current joint vector of the robot
    manipulator
LISTENERS:
    + panda_link0->panda_hand (tf2_ros/transform_listener/TransformListener) - Gets the transform
    of end effector configuration w.r.t panda_link0 frame
FUNCTIONS:
    + quaternion.euler_quarternion - Converts euler roll, pitch and yaw to a quaternion
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup
import moveit_msgs.action
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Vector3, Pose
from moveit_msgs.msg import PositionIKRequest, JointConstraint, CartesianTrajectory,\
    CartesianTrajectoryPoint, CartesianPoint, GenericTrajectory, RobotState
from control_msgs.action import GripperCommand
from moveit_interface.srv import Initial, Goal, Execute, Addobj, GripperSrv
from moveit_msgs.srv import GetPositionIK, GetCartesianPath
from .quaternion import euler_quaternion
from enum import Enum, auto
from moveit_msgs.action import ExecuteTrajectory
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from shape_msgs.msg import SolidPrimitive
from moveit_msgs.msg import CollisionObject, PlanningScene, PlanningSceneComponents, Constraints
from moveit_msgs.srv import GetPlanningScene
from std_msgs.msg import Bool, Int32
import copy

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


class SimpleMove(Node):
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
        super().__init__("simple_move")

        # Initial guess for joint angles
        # These are the home configuration joint angles
        self.initial_js = [0.0, -0.785, 0.0, -2.356, 0.0, 1.57, 0.785]

        # Robot workspace on air hockey table
        # [min, max]
        self.x_table = [-0.3, 0.29]
        self.y_table = [0.41, 0.7]
        self.z_table = -0.015
        # Collision plane
        self.table_collision_dims = [2.0, 2.0, 0.05]
        self.table_collision_posn = [0.0, 1.3, -0.2]

        # Services
        self.initial = self.create_service(Initial, "/initial_service", self.initial_service)
        self.waypoint = self.create_service(Goal, "/waypoint_service", self.waypoint_service)
        self.goal = self.create_service(Goal, "/goal_service", self.goal_service)
        self.execute_srv = self.create_service(Execute, "/execute_service", self.execute_service)

        # Clients
        self.ik_client = self.create_client(GetPositionIK, "compute_ik")
        self.cartesian_path_client = self.create_client(GetCartesianPath, "compute_cartesian_path")

        # Publishers
        self.pub_sm_plan = self.create_publisher(Bool, "/sm_plan", 10)
        self.pub_sm_execute = self.create_publisher(Bool, "/sm_execute", 10)
        self.pub_ee_posn = self.create_publisher(Pose, "/ee_posn", 10)
        self.pub_execute_error_code = self.create_publisher(Int32, "/execute_error_code", 10)

        # Subscribers
        self.sub = self.create_subscription(
            JointState, "/joint_states", self.update_joint_states, 10)
        self.joint_states = JointState()  # Current joint_states

        # Timer callback
        self.timer = self.create_timer(0.1, self.timer_callback)

        # Action Clients
        self._action_client_plan_request = ActionClient(
            self,
            MoveGroup,
            "move_action"
            )
        self._action_client_execute_traj = ActionClient(
            self,
            ExecuteTrajectory,
            "execute_trajectory"
            )

        # State machine variables
        self.state = State.INITIAL
        self.Flag_start_ik = 0
        self.Flag_IK_CAL = 0
        self.Flag_PLAN = 0
        self.Flag_Execute = 0
        self.Flag_box_dim = 0
        self.Flag_box_create = 0
        self.Flag_ee = 0
        self.sm_plan_msg = Bool()
        self.sm_plan_msg.data = False
        self.sm_execute_msg = Bool()
        self.sm_execute_msg.data = False
        self.ee_posn_msg = Pose()
        # Execute trajectory error code flags
        self.Flag_execute = 0
        self.Flag_asyc_call = 0
        self.Execute_error_code = Int32()

        # Compute_IK variables
        self.joint_constr_list = []
        self.rq = PositionIKRequest()

        # Plan request variables
        self.min_corner = Vector3(x=-1.0, y=-1.0, z=-1.0)
        self.max_corner = Vector3(x=1.0, y=1.0, z=1.0)

        # Compute cartesian path variables
        self.car_path = RobotState()
        self.Flag_slow = 0

        # Transform lister
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Box
        self.box_pub = self.create_publisher(PlanningScene, "/planning_scene", 10)
        self.box_client = self.create_client(GetPlanningScene, 'get_planning_scene')
        self.goal = self.create_service(Addobj, "/add_obj", self.obj_service)
        self.robot_state_order = PlanningSceneComponents()
        self.robot_state_order.components = 0
        self.box_future_client = self.box_client.call_async(GetPlanningScene.Request(
            components=self.robot_state_order))

        # Gripper
        self.gripper_action_client = ActionClient(
            self,
            GripperCommand,
            "panda_gripper/gripper_action"
            )

        self.gripper_service = self.create_service(GripperSrv, "/gripper_service", self.gripper_service_callback)

    def gripper_service_callback(self, request, response):
        if request.open == True:
            self.gripper(open=True)
        elif request.open == False:
            self.gripper(open=False)

        return response

    def gripper(self, open):
        """
        open: boolean
        open = True if gripper should be open
        open = False if gripper should be closed
        """
        gripper_msg = GripperCommand.Goal()
        if open is False:
            # gripper_msg.command.position = 0.01   # closed all the way
            # gripper measurement is mm/2
            gripper_msg.command.position = 0.0226
            gripper_msg.command.max_effort = 10.0
        elif open is True:
            gripper_msg.command.position = 0.04 # max open

        if self.gripper_action_client.server_is_ready():
            self.future_gripper_action_request = self.gripper_action_client.send_goal_async(gripper_msg)
            self.future_gripper_action_request.add_done_callback(self.gripper_response_callback)

    def gripper_response_callback(self, future):
        """
        Check if the gripper response is received.

        Returns
        -------
            None

        """
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Unable to change gripper state')
            return

        self.get_logger().info('Changing gripper state')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.gripper_result_callback)
    
    def gripper_result_callback(self, future):
        """
        Store the gripper future result in a variable.

        Keyword arguments:
            gripper_result: Store gripper future result

        Returns
        -------
            None

        """
        self.gripper_result = future.result().result

    def obj_service(self, request, response):
        """
        Service for the box.

        This service obtains and stores the positions and dimension of the box along with an id to
        uniquely identify the object and dynamically place it in the planning scene.

        Args:
            request (moveit_interface/srv/Addobj): Contains the id of the box object (any int),
            the x, y, z position and the dim_x, dim_y, dim_z dimensions
            response: None

        Returns
        -------
            None

        """
        self.Flag_box_dim = 1
        self.id = request.id
        self.pos_x = request.x
        self.pos_y = request.y
        self.pos_z = request.z
        self.dim_x = request.dim_x
        self.dim_y = request.dim_y
        self.dim_z = request.dim_z

        return response

    def update_joint_states(self, data):
        """
        Get current joint states of the robot.

        Subscribtion topic: /joint_states
        This subscription callback obtains and stores the joint angles of the robot manipulator
        Args:
            data (sensor_msgs/msg/JointState): Contains the the joint angles of the robot
            manipulator

        Returns
        -------
            None

        """
        self.joint_states = data

    def initial_service(self, request, response):
        """
        Obtain and store the starting position and orientation of the end effector.

        Args:
            request: (moveit_interface/srv/Initial): Contains the x, y, z position and the roll,
                     pitch and yaw orientation values
            response: None

        Returns
        -------
            None

        """
        self.init_x = request.x
        self.init_y = request.y
        self.init_z = request.z
        self.init_roll = request.roll
        self.init_pitch = request.pitch
        self.init_yaw = request.yaw
        self.init_ori_x, self.init_ori_y, self.init_ori_z, self.init_ori_w = euler_quaternion(
                                                    self.init_roll, self.init_pitch, self.init_yaw)

        # self.start_IK_Callback() # TODO Uncomment this
        return response

    def goal_service(self, request, response):
        """
        Obtain and store the desired end pose of the end-effector.

        Args:
            request (moveit_interface/srv/Goal): Contains the x, y, z position and the roll,
            pitch and yaw orientation values
            response: None

        Returns
        -------
            None

        """
        self.state = State.IK_CAL
        self.goal_x = request.x
        self.goal_y = request.y
        self.goal_z = request.z
        self.goal_roll = request.roll
        self.goal_pitch = request.pitch
        self.goal_yaw = request.yaw
        self.goal_ori_x, self.goal_ori_y, self.goal_ori_z, self.goal_ori_w = euler_quaternion(
                                                    self.goal_roll, self.goal_pitch, self.goal_yaw)
        self.get_logger().info('Goal service called')
        return response

    def waypoint_service(self, request, response):
        """
        Obtain and store a waypoint pose of the end-effector.

        Args:
            request (moveit_interface/srv/Goal): Contains the x, y, z position and the roll,
            pitch and yaw orientation values
            response: None

        Returns
        -------
            None

        """
        # self.state = State.Waypoint
        self.waypoint_x = request.x
        self.waypoint_y = request.y
        self.waypoint_z = request.z
        self.waypoint_roll = request.roll
        self.waypoint_pitch = request.pitch
        self.waypoint_yaw = request.yaw
        self.waypoint_ori_x, self.waypoint_ori_y, self.waypoint_ori_z, self.waypoint_ori_w = euler_quaternion(
                                                    self.waypoint_roll, self.waypoint_pitch, self.waypoint_yaw)

        self.get_logger().info('Waypoint service called')
        return response

    def execute_service(self, request, response):
        """
        Update the state according to the bool flag in the request.

        Args:
            request (moveit_interface/srv/Execute): Contains the exec_bool flag which is either
            True or False
            response: None

        Returns
        -------
            None

        """
        if request.exec_bool is True:
            self.state = State.EXECUTE
        else:
            self.state = State.INITIAL
            # Reset all Flags
            self.Flag_start_ik = 0
            self.Flag_IK_CAL = 0
            self.Flag_PLAN = 0
            self.Flag_Execute = 0
            # Compute_IK variables
            self.joint_constr_list = []
            # Publisher messages
            self.sm_plan_msg.data = False
            self.sm_execute_msg.data = False
        return response

    def start_IK_Callback(self):
        """
        Populate desired starting configuration.

        This function send updated request with desired starting configuration and
        calls the compute_ik service.

        Returns
        -------
            None

        """
        self.Flag_start_ik = 1
        # Compute_IK variables
        self.rq.group_name = 'panda_manipulator'#'panda_arm'
        self.rq.robot_state.joint_state.header.stamp = self.get_clock().now().to_msg()
        self.rq.robot_state.joint_state.header.frame_id = 'panda_link0'
        self.rq.robot_state.joint_state.name = ['panda_joint1', 'panda_joint2', 'panda_joint3',
                                                'panda_joint4', 'panda_joint5', 'panda_joint6',
                                                'panda_joint7']
        self.rq.robot_state.joint_state.position = self.initial_js
        self.rq.robot_state.multi_dof_joint_state.header.stamp = self.get_clock().now().to_msg()
        self.rq.robot_state.multi_dof_joint_state.header.frame_id = 'panda_link0'
        self.rq.robot_state.multi_dof_joint_state.joint_names = ['panda_joint1', 'panda_joint2',
                                                                 'panda_joint3', 'panda_joint4',
                                                                 'panda_joint5', 'panda_joint6',
                                                                 'panda_joint7']
        self.rq.robot_state.is_diff = False
        self.rq.avoid_collisions = True
        self.rq.ik_link_name = 'panda_link8'
        self.rq.pose_stamped.header.stamp = self.get_clock().now().to_msg()
        self.rq.pose_stamped.header.frame_id = 'panda_link0'
        self.rq.pose_stamped.pose.position.x = self.init_x
        self.rq.pose_stamped.pose.position.y = self.init_y
        self.rq.pose_stamped.pose.position.z = self.init_z
        self.rq.pose_stamped.pose.orientation.x = self.init_ori_x
        self.rq.pose_stamped.pose.orientation.y = self.init_ori_y
        self.rq.pose_stamped.pose.orientation.z = self.init_ori_z
        self.rq.pose_stamped.pose.orientation.w = self.init_ori_w
        self.rq.ik_link_names = ['panda_hand', 'panda_hand_tcp', 'panda_leftfinger', 'panda_link0',
                                 'panda_link1', 'panda_link2', 'panda_link3', 'panda_link4',
                                 'panda_link5', 'panda_link6', 'panda_link7', 'panda_link8',
                                 'panda_rightfinger']
        self.rq.pose_stamped_vector = []
        self.rq.timeout.sec = 60
        # Call Compute_IK
        self.future_start_IK = self.ik_client.call_async(GetPositionIK.Request(ik_request=self.rq))

    def Compute_IK_Callback(self):
        """
        Compute the inverse kinematics of the goal end-effector position.

        This function send updated request with desired end configuration of the
        end effector and calls the compute_ik service.

        Returns
        -------
            None

        """
        # Reset all Flags
        self.Flag_IK_CAL = 0
        self.Flag_PLAN = 0
        self.Flag_Execute = 0

        # Compute_IK variables
        self.joint_constr_list = []

        self.rq.group_name = 'panda_manipulator' #'panda_arm'
        self.rq.robot_state.joint_state.header.stamp = self.get_clock().now().to_msg()
        self.rq.robot_state.joint_state.header.frame_id = 'panda_link0'
        self.rq.robot_state.joint_state.name = ['panda_joint1', 'panda_joint2', 'panda_joint3',
                                                'panda_joint4', 'panda_joint5', 'panda_joint6',
                                                'panda_joint7']
        self.rq.robot_state.joint_state.position = self.initial_js
        self.rq.robot_state.multi_dof_joint_state.header.stamp = self.get_clock().now().to_msg()
        self.rq.robot_state.multi_dof_joint_state.header.frame_id = 'panda_link0'
        self.rq.robot_state.multi_dof_joint_state.joint_names = ['panda_joint1', 'panda_joint2',
                                                                 'panda_joint3', 'panda_joint4',
                                                                 'panda_joint5', 'panda_joint6',
                                                                 'panda_joint7']
        self.rq.robot_state.is_diff = False
        self.rq.avoid_collisions = True
        self.rq.ik_link_name = 'panda_link8'
        self.rq.pose_stamped.header.stamp = self.get_clock().now().to_msg()
        self.rq.pose_stamped.header.frame_id = 'panda_link0'

        if self.goal_x == 0 and self.goal_y == 0 and self.goal_z == 0:
            self.rq.pose_stamped.pose.position.x = self.eeX
            self.rq.pose_stamped.pose.position.y = self.eeY
            self.rq.pose_stamped.pose.position.z = self.eeZ
        else:
            self.rq.pose_stamped.pose.position.x = self.goal_x
            self.rq.pose_stamped.pose.position.y = self.goal_y
            self.rq.pose_stamped.pose.position.z = self.goal_z

        self.rq.pose_stamped.pose.orientation.x = self.goal_ori_x
        self.rq.pose_stamped.pose.orientation.y = self.goal_ori_y
        self.rq.pose_stamped.pose.orientation.z = self.goal_ori_z
        self.rq.pose_stamped.pose.orientation.w = self.goal_ori_w
        self.rq.ik_link_names = ['panda_hand', 'panda_hand_tcp', 'panda_leftfinger', 'panda_link0',
                                 'panda_link1', 'panda_link2', 'panda_link3', 'panda_link4',
                                 'panda_link5', 'panda_link6', 'panda_link7', 'panda_link8',
                                 'panda_rightfinger']
        self.rq.pose_stamped_vector = []
        self.rq.timeout.sec = 60
        # Call Compute_IK
        self.future_compute_IK = self.ik_client.call_async(GetPositionIK.Request(
            ik_request=self.rq))

    def start_jointStates(self):
        """
        Parse the Compute_IK result and populating JointConstraint list.

        Returns
        -------
            None

        """
        self.start_ik_result = self.future_start_IK.result()
        self.start_joint_states = JointState()
        self.start_joint_states.header = self.start_ik_result.solution.joint_state.header
        self.start_joint_states.name = self.start_ik_result.solution.joint_state.name
        self.start_joint_states.position = self.start_ik_result.solution.joint_state.position

    def create_jointStates(self):
        """
        Create the joint states and some parameters used in the algorithm.

        Returns
        -------
            None

        """
        self.ik_result = self.future_compute_IK.result()
        for i in range(len(self.ik_result.solution.joint_state.name)):
            constraint = JointConstraint()
            constraint.joint_name = self.ik_result.solution.joint_state.name[i]
            constraint.position = self.ik_result.solution.joint_state.position[i]
            constraint.tolerance_above = 0.005 # TODO Previous 0.01 #0.0001
            constraint.tolerance_below = 0.005 # TODO Previous 0.01 #0.0001
            constraint.weight = 1.0
            self.joint_constr_list.append(constraint)

    # def play_waypoints(self,c1,c2):


    def plan_cartesian(self):
        """
        Create the request messege for the GetCartesianPath service.

        Returns
        -------
            None

        """
        # Move to home position slow
        if self.Flag_slow <3:
            self.Flag_slow += 1
            self.cartesian_velocity_scaling_factor = 1.2
        else:
            # Play speed
            self.cartesian_velocity_scaling_factor = 1.8 # Can go up to 1.9 # Increase velocities and acceleration
        # self.cartesian_time_scaling_factor = 0.8 # Decrease time

        self.car_path.joint_state.header.frame_id = 'panda_link0'
        self.car_path.joint_state.header.stamp = self.get_clock().now().to_msg()
        # Set start position of Franka
        # if self.Flag_start_ik == 1:
        #     self.get_logger().info('!!!!!!!!!! ************  USED IK JOINT  ************** !!!!!!!!!!!!!')
        #     self.car_path.joint_state = self.start_joint_states
        # else:
        #     self.get_logger().info('************  !!!!!!!!!!  USED Joint TOPIC  !!!!!!!!!!  *************')
        #     self.car_path.joint_state = self.joint_states

        self.car_path.joint_state = self.joint_states

        self.group_name = 'panda_manipulator'

        # Midway wapoint
        wp1 = Pose()
        wp1.position.x = self.goal_x
        wp1.position.y = self.goal_y
        wp1.position.z = self.goal_z
        wp1.orientation.x = self.goal_ori_x
        wp1.orientation.y = self.goal_ori_y
        wp1.orientation.z = self.goal_ori_z
        wp1.orientation.w =  self.goal_ori_w

        # End goal waypoint
        wp2 = Pose()
        wp2.position.x = self.waypoint_x
        wp2.position.y = self.waypoint_y
        wp2.position.z = self.waypoint_z
        wp2.orientation.x = self.waypoint_ori_x
        wp2.orientation.y = self.waypoint_ori_y
        wp2.orientation.z = self.waypoint_ori_z
        wp2.orientation.w =  self.waypoint_ori_w

        self.waypoints = [wp2,wp1]

        self.max_step = 5.0 # 10.0
        self.jump_threshold = 10.0
        self.prismatic_jump_threshold = 10.0
        self.revolute_jump_threshold = 10.0
        self.avoid_collisions = True


        # self.ik_result = self.future_compute_IK.result()
        # for i in range(len(self.ik_result.solution.joint_state.name)):
        #     constraint = JointConstraint()
        #     constraint.joint_name = self.ik_result.solution.joint_state.name[i]
        #     constraint.position = self.ik_result.solution.joint_state.position[i]
        #     constraint.tolerance_above = 0.005 # TODO Previous 0.01 #0.0001
        #     constraint.tolerance_below = 0.005 # TODO Previous 0.01 #0.0001
        #     constraint.weight = 1.0
        #     self.joint_constr_list.append(constraint)

        # self.path_constraints = Constraints()
        # self.path_constraints.joint_constraints = [moveit_msgs.msg.JointConstraint(joint_name='panda_joint1', position=-0.2783118071720283, tolerance_above=0.0001, tolerance_below=0.0001, weight=1.0), moveit_msgs.msg.JointConstraint(joint_name='panda_joint2', position=0.5561769123109166, tolerance_above=0.0001, tolerance_below=0.0001, weight=1.0), moveit_msgs.msg.JointConstraint(joint_name='panda_joint3', position=-0.3284530826835772, tolerance_above=0.0001, tolerance_below=0.0001, weight=1.0), moveit_msgs.msg.JointConstraint(joint_name='panda_joint4', position=-0.8883695936747683, tolerance_above=0.0001, tolerance_below=0.0001, weight=1.0), moveit_msgs.msg.JointConstraint(joint_name='panda_joint5', position=0.17305761845989487, tolerance_above=0.0001, tolerance_below=0.0001, weight=1.0), moveit_msgs.msg.JointConstraint(joint_name='panda_joint6', position=1.4224359966379985, tolerance_above=0.0001, tolerance_below=0.0001, weight=1.0), moveit_msgs.msg.JointConstraint(joint_name='panda_joint7', position=0.2511861260062665, tolerance_above=0.0001, tolerance_below=0.0001, weight=1.0)]
        # self.path_constraints.joint_constraints = [moveit_msgs.msg.JointConstraint(joint_name='panda_joint1', tolerance_above=0.001, tolerance_below=0.001, weight=1.0), moveit_msgs.msg.JointConstraint(joint_name='panda_joint2', tolerance_above=0.001, tolerance_below=0.001, weight=1.0), moveit_msgs.msg.JointConstraint(joint_name='panda_joint3', tolerance_above=0.001, tolerance_below=0.001, weight=1.0), moveit_msgs.msg.JointConstraint(joint_name='panda_joint4', tolerance_above=0.001, tolerance_below=0.001, weight=1.0), moveit_msgs.msg.JointConstraint(joint_name='panda_joint5', tolerance_above=0.001, tolerance_below=0.001, weight=1.0), moveit_msgs.msg.JointConstraint(joint_name='panda_joint6', tolerance_above=0.001, tolerance_below=0.001, weight=1.0), moveit_msgs.msg.JointConstraint(joint_name='panda_joint7', tolerance_above=0.001, tolerance_below=0.001, weight=1.0)]

        # Call plan cartesian path
        self.future_plan_cartesian = \
            self.cartesian_path_client.call_async(GetCartesianPath.Request(
                start_state = self.car_path,
                group_name = self.group_name,
                waypoints = self.waypoints,
                max_step = self.max_step,
                jump_threshold = self.jump_threshold,
                prismatic_jump_threshold = self.prismatic_jump_threshold,
                revolute_jump_threshold = self.revolute_jump_threshold,
                avoid_collisions = self.avoid_collisions))
                # path_constraints = self.path_constraints)) # TODO new joint constraints

        self.get_logger().info('Done Calling Cartesian')

    def plan_request(self):
        """
        Create the request messege for MoveGroup. Set workspace parameters.

        Returns
        -------
            None

        """
        plan_request_msg = moveit_msgs.action.MoveGroup.Goal()

        """Populate plan request messege."""
        # Set start position of Franka
        if self.Flag_start_ik == 1:
            plan_request_msg.request.start_state.joint_state = self.start_joint_states
        else:
            plan_request_msg.request.start_state.joint_state = self.joint_states
        # Set time stamp
        plan_request_msg.request.workspace_parameters.header.stamp = \
            self.get_clock().now().to_msg()
        # Header
        plan_request_msg.request.workspace_parameters.header.frame_id = 'panda_link0'
        # Min & Max workspace_parameters
        plan_request_msg.request.workspace_parameters.min_corner = self.min_corner
        plan_request_msg.request.workspace_parameters.max_corner = self.max_corner
        # Header multi_dof_joint_state
        plan_request_msg.request.start_state.multi_dof_joint_state.header.frame_id = 'panda_link0'
        # Additional parameters
        plan_request_msg.request.pipeline_id = 'move_group'
        plan_request_msg.request.group_name = 'panda_manipulator' # panda_arm
        plan_request_msg.request.num_planning_attempts = 10
        plan_request_msg.request.allowed_planning_time = 5.0
        plan_request_msg.request.max_velocity_scaling_factor = 0.1
        plan_request_msg.request.max_acceleration_scaling_factor = 0.1
        # Planning Options
        plan_request_msg.planning_options.planning_scene_diff.robot_state.is_diff = True
        plan_request_msg.planning_options.planning_scene_diff.is_diff = True
        plan_request_msg.planning_options.plan_only = True
        # Goal joint_states of Franka - From compute_ik
        plan_request_msg.request.goal_constraints = [moveit_msgs.msg.Constraints(
            joint_constraints=self.joint_constr_list)]

        #### CartesianTrajectory ####



        # # self.cartesian_point = CartesianPoint()

        # self.cartesianTrajectory_point = CartesianTrajectoryPoint()
        # self.cartesianTrajectory_point.point.pose.position.x = 0.6
        # self.cartesianTrajectory_point.point.pose.position.y = 0.0
        # self.cartesianTrajectory_point.point.pose.position.z = 0.5


        # self.cartesianTrajectory = CartesianTrajectory()
        # self.cartesianTrajectory.points = [self.cartesianTrajectory_point]

        # self.gen_traj = GenericTrajectory()
        # self.gen_traj.cartesian_trajectory = [self.cartesianTrajectory]

        # self.get_logger().info(f"self.gen_traj - {[self.gen_traj]}")
        
        # plan_request_msg.request.reference_trajectories = [self.gen_traj]

        # Future object of plan request
        self.future_plan_request = self._action_client_plan_request.send_goal_async(
            plan_request_msg)
        self.future_plan_request.add_done_callback(self.plan_response_callback)

    def plan_response_callback(self, future):
        """
        Test if the result is received.

        Goal not accept: rejected.
        Goal successful accept:accepted

        Returns
        -------
            None

        """
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        """
        Store the future result in a variable.

        Keyword arguments:
            plan_result: Store future result

        Returns
        -------
            None

        """
        self.plan_result = future.result().result

    def execute_traj(self):
        """
        Store the trajectory messages and use action client to send the trajectory messages.

        Returns
        -------
            None

        """
        execute_traj_msg = moveit_msgs.action.ExecuteTrajectory.Goal()
        # execute_traj_msg.trajectory = self.plan_result.planned_trajectory # MoveGroup plan result

        # Post plan processing - Velocity increase
        for i in range(len(self.future_plan_cartesian.result().solution.joint_trajectory.points)):
            # Add seconds and nanoseconds together
            StoNS = self.future_plan_cartesian.result().solution.joint_trajectory.points[i].time_from_start.sec * 1000000000
            Total_nanoSec = self.future_plan_cartesian.result().solution.joint_trajectory.points[i].time_from_start.nanosec + StoNS

            New_Total_nanoSec = Total_nanoSec / self.cartesian_velocity_scaling_factor
            New_sec = New_Total_nanoSec // 1000000000
            New_nanoSec = New_Total_nanoSec % 1000000000

            self.future_plan_cartesian.result().solution.joint_trajectory.points[i].time_from_start.sec = int(New_sec)
            self.future_plan_cartesian.result().solution.joint_trajectory.points[i].time_from_start.nanosec = int(New_nanoSec)

            # Velocity
            for j in range(len(self.future_plan_cartesian.result().solution.joint_trajectory.points[i].velocities)):
                self.future_plan_cartesian.result().solution.joint_trajectory.points[i].velocities[j] *= self.cartesian_velocity_scaling_factor
                self.future_plan_cartesian.result().solution.joint_trajectory.points[i].accelerations[j] *= self.cartesian_velocity_scaling_factor

        execute_traj_msg.trajectory = self.future_plan_cartesian.result().solution # CartesianPath result
        # self.get_logger().info('EXECUTE ________ service called')

        self.future_execute_traj = self._action_client_execute_traj.send_goal_async(execute_traj_msg)
        
        self.Flag_execute = 1

    def timer_callback(self):
        """
        Continuously running timer callback.

        Use the state machine to set the state for inverse kinematics and
        excutute trajectory.
        Create and add box into Planning Scene.

        Keyword arguments:
            Flag_IK_CAL: trigger conditon for inverse kinematics calculation
            Flag_start_ik: trigger conditon for start inverse Kinematics
            Flag_PLAN: trigger condition for plan a path
            Flag_Execute: trigger condition for execute path
            Flag_box_dim: trigger conditon for box dimension

        Parameters
        ----------
            param eeX: x-axis position
            param eeY: y-axis position
            param eeZ: z-axis position

        Returns
        -------
            None

        """
        if self.Flag_execute == 1:
            if self.future_execute_traj.done():
                if self.Flag_asyc_call == 0:
                    self.Flag_asyc_call = 1
                    self.future_execute_result = self.future_execute_traj.result().get_result_async()
                if self.future_execute_result.done():
                    self.future_result_result = copy.deepcopy(self.future_execute_result.result().result)
                    self.get_logger().info(f' ERROR CODE ++++++++++++++++++++++++++++++++ = {self.future_result_result.error_code}')
                    self.Execute_error_code = self.future_result_result.error_code
                    self.pub_execute_error_code.publish(self.Execute_error_code)
                    self.Flag_asyc_call = 0

        try:
            t = self.tf_buffer.lookup_transform(
                'panda_link0',
                'panda_hand_tcp',
                rclpy.time.Time())
            # Set end effector x,y,z position
            self.eeX = t.transform.translation.x
            self.eeY = t.transform.translation.y
            self.eeZ = t.transform.translation.z
            self.Flag_ee = 1
        except BaseException:
            self.get_logger().info('Panda frames does not exist')

        if self.Flag_ee == 1:
            # Continuously publish end effector position
            self.ee_posn_msg.position.x = self.eeX
            self.ee_posn_msg.position.y = self.eeY
            self.ee_posn_msg.position.z = self.eeZ
            self.pub_ee_posn.publish(self.ee_posn_msg)

        # Continuously publish whether or not node has planned or executed
        self.pub_sm_plan.publish(self.sm_plan_msg)
        self.pub_sm_execute.publish(self.sm_execute_msg)

        if self.state == State.IK_CAL:
            if self.Flag_IK_CAL == 0:
                self.Compute_IK_Callback()
                self.Flag_IK_CAL = 1

            if self.future_compute_IK.done():
                if self.Flag_start_ik == 1:
                    if self.future_start_IK.done():
                        self.state = State.PLAN
                        self.start_jointStates()
                else:
                    self.state = State.PLAN

        if self.state == State.PLAN:
            if self.Flag_PLAN == 0:
                self.Flag_PLAN = 1
                self.create_jointStates()
                # self.plan_request()
                self.sm_plan_msg.data = True
                self.plan_cartesian()
            elif self.future_plan_cartesian.done(): # Automatically execute when planning is done
                self.sm_plan_msg.data = False
                self.state = State.EXECUTE

        if self.state == State.EXECUTE:
            self.sm_execute_msg.data = True
            self.execute_traj()
            self.state = State.INITIAL
            self.sm_execute_msg.data = False
            # Reset all Flags
            self.Flag_start_ik = 0
            self.Flag_IK_CAL = 0
            self.Flag_PLAN = 0
            self.Flag_Execute = 0
            # Compute_IK variables
            self.joint_constr_list = []
            # Publisher messages
            self.sm_plan_msg.data = False
            self.sm_execute_msg.data = False

        if self.Flag_box_dim == 1:
            if self.box_future_client.done():
                self.input_box = self.box_future_client.result()
                self.Flag_box_create = 1
                self.Flag_box_dim = 0
        if self.Flag_box_create == 1:
            self.box_info = self.input_box.scene
            self.box_set = CollisionObject()
            self.box_set.header.stamp = self.get_clock().now().to_msg()
            self.box_set.header.frame_id = 'panda_link0'
            self.box_set.pose.position.x = self.pos_x
            self.box_set.pose.position.y = self.pos_y
            self.box_set.pose.position.z = self.pos_z
            self.box_set.id = "Box" + str(self.id)
            self.box_prim = SolidPrimitive()
            self.box_prim.type = 1
            self.box_prim.dimensions = [self.dim_x, self.dim_y, self.dim_z]
            self.box_set.primitives = [self.box_prim]
            self.box_info.world.collision_objects = [self.box_set]
            self.box_pub.publish(self.box_info)
            self.Flag_box_create = 0


def simple_move_entry(args=None):
    """Run SimpleMove node."""
    rclpy.init(args=args)
    node = SimpleMove()
    rclpy.spin(node)
    rclpy.shutdown()
