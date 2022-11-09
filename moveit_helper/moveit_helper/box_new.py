import rclpy
from rclpy.node import Node
from moveit_msgs.msg import CollisionObject, AttachedCollisionObject
from shape_msgs.msg import SolidPrimitive
#from geometry_msgs.msg import Polygon
from std_msgs.msg import Header
from moveit_msgs.msg import CollisionObject
from moveit_msgs.msg import AttachedCollisionObject 
from moveit_msgs.msg import PlanningScene, PlanningSceneComponents
from moveit_msgs.srv import GetPlanningScene

class AddBox(Node):
    def __init__(self):
        super().__init__('add_box')
        self.box_pub = self.create_publisher(PlanningScene, "/planning_scene", 10)
        #self.box_client = self.create_client(GetPlanningScene, 'get_planning_scene')
        #self.box_future_clinet = self.box_client(GetPlanningScene.Request())
        self.box_info = PlanningScene()
        self.box_info.name = "box"
        
        self.box_set = AttachedCollisionObject()
        self.box_set.link_name = "panda_link0"


        self.box_set.object.header.stamp = self.get_clock().now().to_msg()
        self.box_set.object.header.frame_id = "panda_link0"

        self.box_set.object.pose.position.x = 0.8
        self.box_set.object.pose.position.y = 0.0
        self.box_set.object.pose.position.z = 0.0

        self.box_set.object.id = "Box_2"

        self.box_prim = SolidPrimitive()
        self.box_prim.type = 1
        self.box_prim.dimensions = [0.2,0.2,0.2]
        self.box_set.object.primitives = [self.box_prim]
        self.box_info.robot_state.attached_collision_objects = [self.box_set]
        self.box_pub.publish(self.box_info)


def main(args=None):
    rclpy.init(args=args)
    node= AddBox()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

