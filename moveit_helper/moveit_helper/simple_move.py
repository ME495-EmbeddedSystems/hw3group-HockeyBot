"""
Command to run:
    ros2 run moveit_helper marno_move_group_plan

    Plan:
    1) Delete and run and see what is not necessary
    2) Populate the messege
    3) Integrate messeges
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup
import moveit_msgs.action
import geometry_msgs.msg
import sensor_msgs.msg
from sensor_msgs.msg import JointState
import geometry_msgs
from geometry_msgs.msg import Vector3
from moveit_msgs.msg import PositionIKRequest, JointConstraint
from moveit_interface.srv import Goal, Execute
from moveit_msgs.srv import GetPositionIK
from enum import Enum, auto
from moveit_msgs.action import ExecuteTrajectory
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener


class State(Enum):
    """ The 3 states of the system.
        Determines what the main timer function and other callback should be doing on each iteration
        depending on the state
    """
    INITIAL = auto(),
    IK_CAL = auto(),
    PLAN = auto(),
    READY_EXECUTE = auto(),
    EXECUTE = auto()


class SimpleMove(Node):
    def __init__(self):
        super().__init__("simple_move")

        # Initial guess for joint angles
        # These are the home configuration joint angles
        self.initial_js = [0.0,-0.785, 0.0, -2.356, 0.0, 1.57, 0.785]

        # Service
        self.goal = self.create_service(Goal, "/goal_service", self.goal_service)
        self.execute_srv = self.create_service(Execute, "/execute_service", self.execute_service)

        # Clients
        self.ik_client = self.create_client(GetPositionIK, "compute_ik")

        # Subscribers
        self.sub = self.create_subscription(
            JointState, "/joint_states", self.update_joint_states, 10)
        self.joint_states = JointState()  # Current joint_states


        self.timer = self.create_timer(0.1, self.timer_callback)

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
        self.Flag_IK_CAL = 0
        self.Flag_PLAN = 0
        self.Flag_Execute = 0

        # Compute_IK variables
        self.joint_constr_list = []
        self.rq = PositionIKRequest()


        # Plan request variables
        self.min_corner = Vector3(x=-1.0, y=-1.0, z=-1.0)
        self.max_corner = Vector3(x=1.0, y=1.0, z=1.0)


        # Transform lister
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)




    def update_joint_states(self, data):
        """
        Subscribtion topic: /joint_states
        """
        self.joint_states = data



    
    def goal_service(self, request, response):
        """
        Service: Goal Service - Enter goal position and orientation
        """
        self.state = State.IK_CAL
        self.pose_x=request.pose_x
        self.pose_y=request.pose_y
        self.pose_z=request.pose_z
        self.ori_x=request.orientation_x
        self.ori_y=request.orientation_y
        self.ori_z=request.orientation_z
        self.ori_w=request.orientation_w
        return response

    
    def execute_service(self, request, response):
        if request.exec_bool is True:
            self.state = State.EXECUTE
        else:
            self.state = State.INITIAL
        return response



    def Compute_IK_Callback(self):
        # Reset all Flags
        self.Flag_IK_CAL = 0
        self.Flag_PLAN = 0
        self.Flag_Execute = 0

        # Compute_IK variables
        self.joint_constr_list = []

        self.rq.group_name='panda_arm' # TODO Change to franka_manipulator
        self.rq.robot_state.joint_state.header.stamp=self.get_clock().now().to_msg()
        self.rq.robot_state.joint_state.header.frame_id='panda_link0'
        self.rq.robot_state.joint_state.name=['panda_joint1', 'panda_joint2', 'panda_joint3', 'panda_joint4', 'panda_joint5', 'panda_joint6','panda_joint7' ]
        self.rq.robot_state.joint_state.position=self.initial_js
        self.rq.robot_state.multi_dof_joint_state.header.stamp=self.get_clock().now().to_msg()
        self.rq.robot_state.multi_dof_joint_state.header.frame_id='panda_link0'
        self.rq.robot_state.multi_dof_joint_state.joint_names=['panda_joint1', 'panda_joint2', 'panda_joint3', 'panda_joint4', 'panda_joint5', 'panda_joint6','panda_joint7' ]
        self.rq.robot_state.is_diff=False
        self.rq.avoid_collisions = True
        self.rq.ik_link_name = 'panda_link8'
        self.rq.pose_stamped.header.stamp = self.get_clock().now().to_msg()
        self.rq.pose_stamped.header.frame_id = 'panda_link0'
        self.rq.pose_stamped.pose.position.x = self.pose_x
        self.rq.pose_stamped.pose.position.y = self.pose_y
        self.rq.pose_stamped.pose.position.z = self.pose_z
        self.rq.pose_stamped.pose.orientation.x = self.ori_x
        self.rq.pose_stamped.pose.orientation.y = self.ori_y
        self.rq.pose_stamped.pose.orientation.z = self.ori_z
        self.rq.pose_stamped.pose.orientation.w = self.ori_w
        self.rq.ik_link_names = ['panda_hand', 'panda_hand_tcp', 'panda_leftfinger', 'panda_link0', 'panda_link1', 'panda_link2', 'panda_link3', 'panda_link4', 'panda_link5', 'panda_link6', 'panda_link7', 'panda_link8', 'panda_rightfinger']
        self.rq.pose_stamped_vector = []
        self.rq.timeout.sec = 60

        self.future_compute_IK = self.ik_client.call_async(GetPositionIK.Request(ik_request=self.rq))



    def create_jointStates(self):
        """
        Parse the Compute_IK result and populating JointConstraint list
        """
        self.ik_result = self.future_compute_IK.result()
        for i in range(len(self.ik_result.solution.joint_state.name)):
            constraint = JointConstraint()
            constraint.joint_name = self.ik_result.solution.joint_state.name[i]
            constraint.position = self.ik_result.solution.joint_state.position[i]
            constraint.tolerance_above = 0.0001
            constraint.tolerance_below = 0.0001
            constraint.weight=1.0
            self.joint_constr_list.append(constraint)



    def plan_request(self):

        plan_request_msg = moveit_msgs.action.MoveGroup.Goal()

        """ Populate plan request messege """
        # Set start position of Franka 
        plan_request_msg.request.start_state.joint_state = self.joint_states
        # Set time stamp
        plan_request_msg.request.workspace_parameters.header.stamp = self.get_clock().now().to_msg()
        # Header
        plan_request_msg.request.workspace_parameters.header.frame_id = 'panda_link0'
        # Min & Max workspace_parameters
        plan_request_msg.request.workspace_parameters.min_corner = self.min_corner
        plan_request_msg.request.workspace_parameters.max_corner = self.max_corner
        # Header multi_dof_joint_state
        plan_request_msg.request.start_state.multi_dof_joint_state.header.frame_id = 'panda_link0'
        # Additional parameters
        plan_request_msg.request.pipeline_id = 'move_group'
        plan_request_msg.request.group_name = 'panda_manipulator'
        plan_request_msg.request.num_planning_attempts = 10
        plan_request_msg.request.allowed_planning_time = 5.0
        plan_request_msg.request.max_velocity_scaling_factor = 0.1
        plan_request_msg.request.max_acceleration_scaling_factor = 0.1
        # Planning Options
        plan_request_msg.planning_options.planning_scene_diff.robot_state.is_diff = True
        plan_request_msg.planning_options.planning_scene_diff.is_diff = True
        plan_request_msg.planning_options.plan_only = True
        # Goal joint_states of Franka - From compute_ik
        plan_request_msg.request.goal_constraints = [moveit_msgs.msg.Constraints(joint_constraints = self.joint_constr_list)]

        # Future object of plan request
        self.future_plan_request = self._action_client_plan_request.send_goal_async(plan_request_msg)
        self.future_plan_request.add_done_callback(self.plan_response_callback)


    def plan_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)


    def get_result_callback(self, future):
        self.plan_result = future.result().result
        # self.state = State.READY_EXECUTE


    def execute_traj(self):

        execute_traj_msg = moveit_msgs.action.ExecuteTrajectory.Goal()

        execute_traj_msg.trajectory = self.plan_result.planned_trajectory

        self._action_client_execute_traj.send_goal_async(execute_traj_msg)



    def timer_callback(self):


        try:
            t = self.tf_buffer.lookup_transform(
                'panda_link0',
                'panda_hand',
                rclpy.time.Time())
            # Set end effector x,y,z position
            self.eeX = t.transform.translation.x
            self.eeY = t.transform.translation.y
            self.eeZ = t.transform.translation.z
        except BaseException:
            self.get_logger().info('Panda frames does not exist')

        if self.state == State.IK_CAL:
            if self.Flag_IK_CAL == 0:
                self.Compute_IK_Callback()
                self.Flag_IK_CAL = 1
            if self.future_compute_IK.done():
                self.state = State.PLAN

        if self.state == State.PLAN:
            if self.Flag_PLAN == 0:
                self.Flag_PLAN = 1
                self.create_jointStates()
                self.plan_request()

        if self.state == State.EXECUTE:
            self.execute_traj()
            self.state = State.INITIAL
             # Reset all Flags
            self.Flag_IK_CAL = 0
            self.Flag_PLAN = 0
            self.Flag_Execute = 0

            # Compute_IK variables
            self.joint_constr_list = []





def simple_move_entry(args=None):
    rclpy.init(args=args)
    node = SimpleMove()
    rclpy.spin(node)
    rclpy.shutdown()


