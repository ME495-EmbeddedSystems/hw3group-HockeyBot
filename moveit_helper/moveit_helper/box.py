import rclpy
from rclpy.node import Node
from moveit_msgs.msg import PlanningScene
from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject
class Box(Node):
    def __init__(self, name):
        super().__init__(name)                                                          
        self.pub_box = self.create_publisher(PlanningScene, '/planning_scene', 10) 
        self.timer = self.create_timer(0.5, self.timer_callback)
        self.box=CollisionObject
        self.msg.id = "box1"
        self.msg=Pose()
        self.msg.header.frame_id = "panda_link0"
        msg.orientation.x = 0.0
        msg.orientation.y = 0.0
        msg.orientation.z = 0.0
        msg.orientation.w = 1.0
        msg.position.x = 0.28
        msg.position.y = 0.4
        msg.position.z = 0.5
    def timer_callback(self):
        msg.header.stamp = self.get_clock().now().to_msg()

        self.pub_box(self.brick)

def main(args=None):                                
    rclpy.init(args=args)                            
    node = Box()     
    rclpy.spin(node)                                 
    node.destroy_node()                              
    rclpy.shutdown() 