"""
Command to run:
    ros2 run moveit_helper marno_move_group_plan 
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from moveit_msgs.action import MoveGroup
import moveit_msgs.action
import std_msgs.msg
import builtin_interfaces.msg
import geometry_msgs.msg
import sensor_msgs.msg
import octomap_msgs.msg
from sensor_msgs.msg import JointState

class Intercept(Node):
    def __init__(self):
        super().__init__("fake_movegroup")

        # Subscribers
        self.sub = self.create_subscription(
            JointState, "/joint_states", self.update_joint_states, 10)
        self.joint_states = JointState()  # Current joint_states

        self._action_client = ActionClient(
            self,
            MoveGroup,
            "move_action"
            )

    def update_joint_states(self, data):
        """
        Subscribtion topic: /joint_states
        """
        self.joint_states = data


        # self._action_server = ActionServer(
        #     self,
        #     MoveGroup,
        #     "move_action",
        #     self.move_action_callback
        # )

    # def move_action_callback(self, goal):
    #     self.get_logger().info(f"{goal.request}")
    #     result = MoveGroup.Result()
    #     return result

    def send_goal(self):
        plan_msg = moveit_msgs.action.MoveGroup.Goal(
            request=moveit_msgs.msg.MotionPlanRequest(
            workspace_parameters=moveit_msgs.msg.WorkspaceParameters(
            header=std_msgs.msg.Header(
            stamp=builtin_interfaces.msg.Time(
            sec=1667668827,
            nanosec=729437629),
            frame_id='panda_link0'),
            min_corner=geometry_msgs.msg.Vector3(
            x=-1.0,
            y=-1.0,
            z=-1.0),
            max_corner=geometry_msgs.msg.Vector3(
            x=1.0,
            y=1.0,
            z=1.0)),
            start_state=moveit_msgs.msg.RobotState(
            joint_state=sensor_msgs.msg.JointState(  ## We can pass objects without decomposing
            header=std_msgs.msg.Header(
            stamp=builtin_interfaces.msg.Time(
            sec=0,
            nanosec=0),
            frame_id='panda_link0'),
            name=['panda_joint1',
            'panda_joint2',
            'panda_joint3',
            'panda_joint4',
            'panda_joint5',
            'panda_joint6',
            'panda_joint7',
            'panda_finger_joint1',
            'panda_finger_joint2'],
            position=[-3.05743667552222e-05,
            0.45826129385071196,
            0.36520248871411476,
            -1.8763676740530548,
            -0.21278170570578495,
            2.2968262699790225,
            1.2581421343073955,
            0.035,
            0.035],
            velocity=[],
            effort=[]),
            multi_dof_joint_state=sensor_msgs.msg.MultiDOFJointState(
            header=std_msgs.msg.Header(
            stamp=builtin_interfaces.msg.Time(
            sec=0,
            nanosec=0),
            frame_id='panda_link0'),
            joint_names=[],
            transforms=[],
            twist=[],
            wrench=[]),
            attached_collision_objects=[],
            is_diff=False),
            goal_constraints=[moveit_msgs.msg.Constraints(
            name='',
            joint_constraints=[moveit_msgs.msg.JointConstraint(
            joint_name='panda_joint1',
            position=-0.799720847385826,
            tolerance_above=0.0001,
            tolerance_below=0.0001,
            weight=1.0),
            moveit_msgs.msg.JointConstraint(
            joint_name='panda_joint2',
            position=0.6631403268111388,
            tolerance_above=0.0001,
            tolerance_below=0.0001,
            weight=1.0),
            moveit_msgs.msg.JointConstraint(
            joint_name='panda_joint3',
            position=0.1428222590095236,
            tolerance_above=0.0001,
            tolerance_below=0.0001,
            weight=1.0),
            moveit_msgs.msg.JointConstraint(
            joint_name='panda_joint4',
            position=-1.9078958035320892,
            tolerance_above=0.0001,
            tolerance_below=0.0001,
            weight=1.0),
            moveit_msgs.msg.JointConstraint(
            joint_name='panda_joint5',
            position=-0.16023113114248438,
            tolerance_above=0.0001,
            tolerance_below=0.0001,
            weight=1.0),
            moveit_msgs.msg.JointConstraint(
            joint_name='panda_joint6',
            position=2.5601750273131985,
            tolerance_above=0.0001,
            tolerance_below=0.0001,
            weight=1.0),
            moveit_msgs.msg.JointConstraint(
            joint_name='panda_joint7',
            position=0.2327714355052132,
            tolerance_above=0.0001,
            tolerance_below=0.0001,
            weight=1.0)],
            position_constraints=[],
            orientation_constraints=[],
            visibility_constraints=[])],
            path_constraints=moveit_msgs.msg.Constraints(
            name='',
            joint_constraints=[],
            position_constraints=[],
            orientation_constraints=[],
            visibility_constraints=[]),
            trajectory_constraints=moveit_msgs.msg.TrajectoryConstraints(
            constraints=[]),
            reference_trajectories=[],
            pipeline_id='move_group',
            planner_id='',
            group_name='panda_manipulator',
            num_planning_attempts=10,
            allowed_planning_time=5.0,
            max_velocity_scaling_factor=0.1,
            max_acceleration_scaling_factor=0.1,
            cartesian_speed_end_effector_link='',
            max_cartesian_speed=0.0),
            planning_options=moveit_msgs.msg.PlanningOptions(
            planning_scene_diff=moveit_msgs.msg.PlanningScene(
            name='',
            robot_state=moveit_msgs.msg.RobotState(
            joint_state=sensor_msgs.msg.JointState(
            header=std_msgs.msg.Header(
            stamp=builtin_interfaces.msg.Time(
            sec=0,
            nanosec=0),
            frame_id=''),
            name=[],
            position=[],
            velocity=[],
            effort=[]),
            multi_dof_joint_state=sensor_msgs.msg.MultiDOFJointState(
            header=std_msgs.msg.Header(
            stamp=builtin_interfaces.msg.Time(
            sec=0,
            nanosec=0),
            frame_id=''),
            joint_names=[],
            transforms=[],
            twist=[],
            wrench=[]),
            attached_collision_objects=[],
            is_diff=True),
            robot_model_name='',
            fixed_frame_transforms=[],
            allowed_collision_matrix=moveit_msgs.msg.AllowedCollisionMatrix(
            entry_names=[],
            entry_values=[],
            default_entry_names=[],
            default_entry_values=[]),
            link_padding=[],
            link_scale=[],
            object_colors=[],
            world=moveit_msgs.msg.PlanningSceneWorld(
            collision_objects=[],
            octomap=octomap_msgs.msg.OctomapWithPose(
            header=std_msgs.msg.Header(
            stamp=builtin_interfaces.msg.Time(
            sec=0,
            nanosec=0),
            frame_id=''),
            origin=geometry_msgs.msg.Pose(
            position=geometry_msgs.msg.Point(
            x=0.0,
            y=0.0,
            z=0.0),
            orientation=geometry_msgs.msg.Quaternion(
            x=0.0,
            y=0.0,
            z=0.0,
            w=1.0)),
            octomap=octomap_msgs.msg.Octomap(
            header=std_msgs.msg.Header(
            stamp=builtin_interfaces.msg.Time(
            sec=0,
            nanosec=0),
            frame_id=''),
            binary=False,
            id='',
            resolution=0.0,
            data=[]))),
            is_diff=True),
            plan_only=True,
            look_around=False,
            look_around_attempts=0,
            max_safe_execution_cost=0.0,
            replan=False,
            replan_attempts=0,
            replan_delay=0.0))


        """ Populate plan request messege """
        # Set start position of Franka 
        plan_msg.request.start_state.joint_state = self.joint_states


        # goal_msg.order = order
        # self.get_logger().info(f"{plan_msg}")

        # self.get_logger().info(f"{plan_msg.request.start_state.joint_state}")
        
        self._action_client.wait_for_server()

        return self._action_client.send_goal_async(plan_msg)


    # def goal_msg(self):
    #     moveit_msgs.action.MoveGroup_Goal(request=moveit_msgs.msg.MotionPlanRequest(workspace_parameters=moveit_msgs.msg.WorkspaceParameters(header=std_msgs.msg.Header(stamp=builtin_interfaces.msg.Time(sec=1667672517, nanosec=278032304), frame_id='panda_link0'), min_corner=geometry_msgs.msg.Vector3(x=-1.0, y=-1.0, z=-1.0), max_corner=geometry_msgs.msg.Vector3(x=1.0, y=1.0, z=1.0)), start_state=moveit_msgs.msg.RobotState(joint_state=sensor_msgs.msg.JointState(header=std_msgs.msg.Header(stamp=builtin_interfaces.msg.Time(sec=0, nanosec=0), frame_id='panda_link0'), name=['panda_joint1', 'panda_joint2', 'panda_joint3', 'panda_joint4', 'panda_joint5', 'panda_joint6', 'panda_joint7', 'panda_finger_joint1', 'panda_finger_joint2'], position=[0.0, -0.7853981633974483, 0.0, -2.356194490192345, 0.0, 1.5707963267948966, 0.7853981633974483, 0.035, 0.035], velocity=[], effort=[]), multi_dof_joint_state=sensor_msgs.msg.MultiDOFJointState(header=std_msgs.msg.Header(stamp=builtin_interfaces.msg.Time(sec=0, nanosec=0), frame_id='panda_link0'), joint_names=[], transforms=[], twist=[], wrench=[]), attached_collision_objects=[], is_diff=False), goal_constraints=[moveit_msgs.msg.Constraints(name='', joint_constraints=[moveit_msgs.msg.JointConstraint(joint_name='panda_joint1', position=-0.2783118071720283, tolerance_above=0.0001, tolerance_below=0.0001, weight=1.0), moveit_msgs.msg.JointConstraint(joint_name='panda_joint2', position=0.5561769123109166, tolerance_above=0.0001, tolerance_below=0.0001, weight=1.0), moveit_msgs.msg.JointConstraint(joint_name='panda_joint3', position=-0.3284530826835772, tolerance_above=0.0001, tolerance_below=0.0001, weight=1.0), moveit_msgs.msg.JointConstraint(joint_name='panda_joint4', position=-0.8883695936747683, tolerance_above=0.0001, tolerance_below=0.0001, weight=1.0), moveit_msgs.msg.JointConstraint(joint_name='panda_joint5', position=0.17305761845989487, tolerance_above=0.0001, tolerance_below=0.0001, weight=1.0), moveit_msgs.msg.JointConstraint(joint_name='panda_joint6', position=1.4224359966379985, tolerance_above=0.0001, tolerance_below=0.0001, weight=1.0), moveit_msgs.msg.JointConstraint(joint_name='panda_joint7', position=0.2511861260062665, tolerance_above=0.0001, tolerance_below=0.0001, weight=1.0)], position_constraints=[], orientation_constraints=[], visibility_constraints=[])], path_constraints=moveit_msgs.msg.Constraints(name='', joint_constraints=[], position_constraints=[], orientation_constraints=[], visibility_constraints=[]), trajectory_constraints=moveit_msgs.msg.TrajectoryConstraints(constraints=[]), reference_trajectories=[], pipeline_id='move_group', planner_id='', group_name='panda_manipulator', num_planning_attempts=10, allowed_planning_time=5.0, max_velocity_scaling_factor=0.1, max_acceleration_scaling_factor=0.1, cartesian_speed_end_effector_link='', max_cartesian_speed=0.0), planning_options=moveit_msgs.msg.PlanningOptions(planning_scene_diff=moveit_msgs.msg.PlanningScene(name='', robot_state=moveit_msgs.msg.RobotState(joint_state=sensor_msgs.msg.JointState(header=std_msgs.msg.Header(stamp=builtin_interfaces.msg.Time(sec=0, nanosec=0), frame_id=''), name=[], position=[], velocity=[], effort=[]), multi_dof_joint_state=sensor_msgs.msg.MultiDOFJointState(header=std_msgs.msg.Header(stamp=builtin_interfaces.msg.Time(sec=0, nanosec=0), frame_id=''), joint_names=[], transforms=[], twist=[], wrench=[]), attached_collision_objects=[], is_diff=True), robot_model_name='', fixed_frame_transforms=[], allowed_collision_matrix=moveit_msgs.msg.AllowedCollisionMatrix(entry_names=[], entry_values=[], default_entry_names=[], default_entry_values=[]), link_padding=[], link_scale=[], object_colors=[], world=moveit_msgs.msg.PlanningSceneWorld(collision_objects=[], octomap=octomap_msgs.msg.OctomapWithPose(header=std_msgs.msg.Header(stamp=builtin_interfaces.msg.Time(sec=0, nanosec=0), frame_id=''), origin=geometry_msgs.msg.Pose(position=geometry_msgs.msg.Point(x=0.0, y=0.0, z=0.0), orientation=geometry_msgs.msg.Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)), octomap=octomap_msgs.msg.Octomap(header=std_msgs.msg.Header(stamp=builtin_interfaces.msg.Time(sec=0, nanosec=0), frame_id=''), binary=False, id='', resolution=0.0, data=[]))), is_diff=True), plan_only=True, look_around=False, look_around_attempts=0, max_safe_execution_cost=0.0, replan=False, replan_attempts=0, replan_delay=0.0))


def interept_entry(args=None):
    rclpy.init(args=args)
    # node = Intercept()
    # rclpy.spin(node)
    # rclpy.shutdown()

    action_client = Intercept()
    future = action_client.send_goal()
    rclpy.spin_until_future_complete(action_client, future)
