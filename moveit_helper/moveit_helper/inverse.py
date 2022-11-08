import rclpy
from rclpy.node import Node
from moveit_msgs.msg import PositionIKRequest
from moveit_msgs.srv import GetPositionIK


class InverseKinematics(Node):
    def __init__(self):
        super().__init__('inverse_kinematics')
        self.ik_client = self.create_client(GetPositionIK, "compute_ik")
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.rq = PositionIKRequest()
        self.rq.group_name='panda_arm'
        self.rq.robot_state.joint_state.header.stamp=self.get_clock().now().to_msg()
        self.rq.robot_state.joint_state.header.frame_id='panda_link0'
        self.rq.robot_state.joint_state.name=['panda_joint1', 'panda_joint2', 'panda_joint3', 'panda_joint4', 'panda_joint5', 'panda_joint6','panda_joint7' ]
        self.rq.robot_state.joint_state.position=[0.0,-0.70, 0.0, -2.35, 0.0, 1.57, 0.79]
        # self.rq.robot_state.joint_state.velocity=[]
        # self.rq.robot_state.joint_state.effort=[]
        self.rq.robot_state.multi_dof_joint_state.header.stamp=self.get_clock().now().to_msg()
        self.rq.robot_state.multi_dof_joint_state.header.frame_id='panda_link0'
        self.rq.robot_state.multi_dof_joint_state.joint_names=['panda_joint1', 'panda_joint2', 'panda_joint3', 'panda_joint4', 'panda_joint5', 'panda_joint6','panda_joint7' ]
        # self.rq.robot_state.multi_dof_joint_state.transforms=[]
        # self.rq.robot_state.multi_dof_joint_state.twist=[]
        # self.rq.robot_state.multi_dof_joint_state.wrench=[]
        # self.rq.robot_state.attached_collision_objects=[]
        self.rq.robot_state.is_diff=False
        self.rq.avoid_collisions = True
        self.rq.ik_link_name = 'panda_link8'
        self.rq.pose_stamped.header.stamp = self.get_clock().now().to_msg()
        self.rq.pose_stamped.header.frame_id = 'panda_link0'
        self.rq.pose_stamped.pose.position.x = 0.5
        self.rq.pose_stamped.pose.position.y = 0.0
        self.rq.pose_stamped.pose.position.z = 0.0
        self.rq.pose_stamped.pose.orientation.x = 0.0
        self.rq.pose_stamped.pose.orientation.y = 0.0
        self.rq.pose_stamped.pose.orientation.z = 0.0
        self.rq.pose_stamped.pose.orientation.w = 1.0
        self.rq.ik_link_names = ['panda_hand', 'panda_hand_tcp', 'panda_leftfinger', 'panda_link0', 'panda_link1', 'panda_link2', 'panda_link3', 'panda_link4', 'panda_link5', 'panda_link6', 'panda_link7', 'panda_link8', 'panda_rightfinger']
        self.rq.pose_stamped_vector = []
        self.rq.timeout.sec = 60
        self.get_logger().info(f'before async')
        self.future= self.ik_client.call_async(GetPositionIK.Request(ik_request=self.rq))

    def timer_callback(self):
        # self.get_logger().info(f'waiting')
        if self.future.done():
            self.get_logger().info(f'{self.future.result()}')



def IK_entry(args=None):
    rclpy.init(args=args)
    node= InverseKinematics()
    rclpy.spin(node)
    rclpy.shutdown()