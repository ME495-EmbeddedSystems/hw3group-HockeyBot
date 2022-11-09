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
import std_msgs.msg
import geometry_msgs.msg
import sensor_msgs.msg
from sensor_msgs.msg import JointState
import geometry_msgs
from geometry_msgs.msg import Vector3
from moveit_msgs.msg import PositionIKRequest, JointConstraint
from moveit_interface.srv import Goal
from moveit_msgs.srv import GetPositionIK
from enum import Enum, auto



class State(Enum):
    """ The 3 states of the system.
        Determines what the main timer function and other callback should be doing on each iteration
        depending on the state
    """
    INITIAL = auto(),
    GETTING = auto(),
    GOTIT = auto(),
    PLAN = auto()


class SimpleMove(Node):
    def __init__(self):
        super().__init__("simple_move")

        # Service
        self.goal = self.create_service(Goal, "/goal_service", self.goal_service)
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



        self.state = State.INITIAL

        # Compute_IK variables
        self.joint_constr_list = []
        self.rq = PositionIKRequest()


        # Plan request variables
        self.min_corner = Vector3(x=-1.0, y=-1.0, z=-1.0)
        self.max_corner = Vector3(x=1.0, y=1.0, z=1.0)


        


    def update_joint_states(self, data):
        """
        Subscribtion topic: /joint_states
        """
        self.joint_states = data



    
    def goal_service(self, request, response):
        """
        Service: Goal Service - Enter goal position and orientation
        """
        self.state = State.GETTING
        self.pose_x=request.pose_x
        self.pose_y=request.pose_y
        self.pose_z=request.pose_z
        self.ori_x=request.orientation_x
        self.ori_y=request.orientation_y
        self.ori_z=request.orientation_z
        self.ori_w=request.orientation_w
        return response



    def load_callback(self):
        self.rq.group_name='panda_arm'
        self.rq.robot_state.joint_state.header.stamp=self.get_clock().now().to_msg()
        self.rq.robot_state.joint_state.header.frame_id='panda_link0'
        self.rq.robot_state.joint_state.name=['panda_joint1', 'panda_joint2', 'panda_joint3', 'panda_joint4', 'panda_joint5', 'panda_joint6','panda_joint7' ]
        self.rq.robot_state.joint_state.position=[0.0,-0.70, 0.0, -2.35, 0.0, 1.57, 0.79]
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
        self.get_logger().info(f'before async')
        self.future_compute_IK = self.ik_client.call_async(GetPositionIK.Request(ik_request=self.rq))



    def plan_request(self):

        plan_request_msg = moveit_msgs.action.MoveGroup.Goal(
            request=moveit_msgs.msg.MotionPlanRequest(
            goal_constraints=[moveit_msgs.msg.Constraints()]))

        # plan_request_msg = moveit_msgs.action.MoveGroup.Goal(
        #     request=moveit_msgs.msg.MotionPlanRequest(
        #     # workspace_parameters=moveit_msgs.msg.WorkspaceParameters(
        #     # # header=std_msgs.msg.Header(
        #     # # stamp=builtin_interfaces.msg.Time(
        #     # # sec=1667668827,
        #     # # nanosec=729437629),
        #     # # frame_id='panda_link0'),
        #     # min_corner=geometry_msgs.msg.Vector3(
        #     # x=-1.0,
        #     # y=-1.0,
        #     # z=-1.0),
        #     # max_corner=geometry_msgs.msg.Vector3(
        #     # x=1.0,
        #     # y=1.0,
        #     # z=1.0)
        #     # ),
        #     # start_state=moveit_msgs.msg.RobotState(
        #     # joint_state=sensor_msgs.msg.JointState(  ## We can pass objects without decomposing
        #     # header=std_msgs.msg.Header(
        #     # frame_id='panda_link0'),
        #     # name=['panda_joint1',
        #     # 'panda_joint2',
        #     # 'panda_joint3',
        #     # 'panda_joint4',
        #     # 'panda_joint5',
        #     # 'panda_joint6',
        #     # 'panda_joint7',
        #     # 'panda_finger_joint1',
        #     # 'panda_finger_joint2'],
        #     # position=[-3.05743667552222e-05,
        #     # 0.45826129385071196,
        #     # 0.36520248871411476,
        #     # -1.8763676740530548,
        #     # -0.21278170570578495,
        #     # 2.2968262699790225,
        #     # 1.2581421343073955,
        #     # 0.035,
        #     # 0.035]),
        #     # multi_dof_joint_state=sensor_msgs.msg.MultiDOFJointState(
        #     # header=std_msgs.msg.Header(
        #     # frame_id='panda_link0')),
        #     # is_diff=False),
        #     goal_constraints=[moveit_msgs.msg.Constraints(
        #     # joint_constraints=[moveit_msgs.msg.JointConstraint(
        #     # joint_name='panda_joint1',
        #     # position=-0.799720847385826,
        #     # tolerance_above=0.0001,
        #     # tolerance_below=0.0001,
        #     # weight=1.0),
        #     # moveit_msgs.msg.JointConstraint(
        #     # joint_name='panda_joint2',
        #     # position=0.6631403268111388,
        #     # tolerance_above=0.0001,
        #     # tolerance_below=0.0001,
        #     # weight=1.0),
        #     # moveit_msgs.msg.JointConstraint(
        #     # joint_name='panda_joint3',
        #     # position=0.1428222590095236,
        #     # tolerance_above=0.0001,
        #     # tolerance_below=0.0001,
        #     # weight=1.0),
        #     # moveit_msgs.msg.JointConstraint(
        #     # joint_name='panda_joint4',
        #     # position=-1.9078958035320892,
        #     # tolerance_above=0.0001,
        #     # tolerance_below=0.0001,
        #     # weight=1.0),
        #     # moveit_msgs.msg.JointConstraint(
        #     # joint_name='panda_joint5',
        #     # position=-0.16023113114248438,
        #     # tolerance_above=0.0001,
        #     # tolerance_below=0.0001,
        #     # weight=1.0),
        #     # moveit_msgs.msg.JointConstraint(
        #     # joint_name='panda_joint6',
        #     # position=2.5601750273131985,
        #     # tolerance_above=0.0001,
        #     # tolerance_below=0.0001,
        #     # weight=1.0),
        #     # moveit_msgs.msg.JointConstraint(
        #     # joint_name='panda_joint7',
        #     # position=0.2327714355052132,
        #     # tolerance_above=0.0001,
        #     # tolerance_below=0.0001,
        #     # weight=1.0)]
        #     # )]


        #     # joint_constraints = [moveit_msgs.msg.JointConstraint(joint_name='panda_joint1', position=0.4653249156877396, tolerance_above=0.0001, tolerance_below=0.0001, weight=1.0), moveit_msgs.msg.JointConstraint(joint_name='panda_joint2', position=0.6364748471474883, tolerance_above=0.0001, tolerance_below=0.0001, weight=1.0), moveit_msgs.msg.JointConstraint(joint_name='panda_joint3', position=0.4010811629510251, tolerance_above=0.0001, tolerance_below=0.0001, weight=1.0), moveit_msgs.msg.JointConstraint(joint_name='panda_joint4', position=-1.5635399671690575, tolerance_above=0.0001, tolerance_below=0.0001, weight=1.0), moveit_msgs.msg.JointConstraint(joint_name='panda_joint5', position=2.8973, tolerance_above=0.0001, tolerance_below=0.0001, weight=1.0), moveit_msgs.msg.JointConstraint(joint_name='panda_joint6', position=1.08220079358666, tolerance_above=0.0001, tolerance_below=0.0001, weight=1.0), moveit_msgs.msg.JointConstraint(joint_name='panda_joint7', position=0.2476650932285474, tolerance_above=0.0001, tolerance_below=0.0001, weight=1.0), moveit_msgs.msg.JointConstraint(joint_name='panda_finger_joint1', position=0.035, tolerance_above=0.0001, tolerance_below=0.0001, weight=1.0), moveit_msgs.msg.JointConstraint(joint_name='panda_finger_joint2', position=0.035, tolerance_above=0.0001, tolerance_below=0.0001, weight=1.0)]
        #     )]
        #     # ,
        #     # pipeline_id='move_group',
        #     # group_name='panda_manipulator',
        #     # num_planning_attempts=10,
        #     # allowed_planning_time=5.0,
        #     # max_velocity_scaling_factor=0.1,
        #     # max_acceleration_scaling_factor=0.1
        #     ),
        #     # planning_options=moveit_msgs.msg.PlanningOptions(
        #     # planning_scene_diff=moveit_msgs.msg.PlanningScene(
        #     # robot_state=moveit_msgs.msg.RobotState(
        #     # is_diff=True),
        #     # # is_diff=True
        #     # ),
        #     # # plan_only=True
        #     # )
        #     )


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
        plan_request_msg.request.path_constraints.joint_constraints = self.joint_constr_list


        self.future_plan_request = self._action_client_plan_request.send_goal_async(plan_request_msg)



    def timer_callback(self):

        if self.state == State.GETTING:
            self.load_callback()
            self.state = State.GOTIT

        if self.state == State.GOTIT:
            if self.future_compute_IK.done():
                self.state = State.PLAN
                # self.get_logger().info(f'{self.future_compute_IK.result()}')
                self.ik_result = self.future_compute_IK.result()
                # Parse the Compute_IK result and populating JointConstraint list
                for i in range(len(self.ik_result.solution.joint_state.name)):
                    constraint = JointConstraint()
                    constraint.joint_name = self.ik_result.solution.joint_state.name[i]
                    constraint.position = self.ik_result.solution.joint_state.position[i]
                    constraint.tolerance_above = 0.0001
                    constraint.tolerance_below = 0.0001
                    constraint.weight=1.0
                    self.joint_constr_list.append(constraint)

                self.get_logger().info(f'{self.joint_constr_list}')
                # Call plan request function
                self.plan_request()

        if self.state == State.PLAN:
            if self.future_plan_request.done():
                self.get_logger().info(f'{self.future_plan_request.result()}')




def simple_move_entry(args=None):
    rclpy.init(args=args)
    node = SimpleMove()
    rclpy.spin(node)
    rclpy.shutdown()


