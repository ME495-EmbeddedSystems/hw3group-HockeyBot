import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from visualization_msgs.msg import Marker, MarkerArray
from tf2_ros import TransformBroadcaster
from enum import Enum, auto
from rclpy.callback_groups import ReentrantCallbackGroup
from geometry_msgs.msg import Quaternion
from math import sqrt, sin, cos, tan
from rcl_interfaces.msg import ParameterDescriptor
from sensor_msgs.msg import JointState


class Subscribe_jointS(Node):


    def __init__(self):
        super().__init__('Subscribe_jointS')


        # Subscribers
        self.sub = self.create_subscription(
            JointState, "/joint_states", self.update_joint_states, 10)
        self.joint_states = JointState()  # Turtle current pose

        # Timer for callback function at 250Hz
        self.tmr = self.create_timer(0.004, self.timer_callback)


    def update_joint_states(self, data):
        """
        Subscribtion topic: turtle1/cmd_vel
        """
        self.joint_states = data

    


    def timer_callback(self):

        print(f"self.joint_states = {self.joint_states}")



def joint_state_entry(args=None):
    rclpy.init(args=args)
    node = Subscribe_jointS()
    rclpy.spin(node)
    rclpy.shutdown()
