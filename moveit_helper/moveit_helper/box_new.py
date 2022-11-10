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
        self.box_client = self.create_client(GetPlanningScene, 'get_planning_scene')
        self.robot_state_order = PlanningSceneComponents()
        # self.input_box=PlanningScene()
        self.robot_state_order.components = 0
        self.count = 0
        self.box_future_client = self.box_client.call_async(GetPlanningScene.Request(components=self.robot_state_order))
        # self.box_info = PlanningScene()
        # rclpy.spin_until_future_complete(self, self.box_future_clinet)
        # self.box_info = self.box_future_clinet.result()
        # self.box_pub.publish(self.box_info)
        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        if self.count == 0:
            if self.box_future_client.done():
                self.get_logger().info(f'done ')
                self.input_box = self.box_future_client.result()
                self.count += 1
        if self.count == 1:
            self.box_info= self.input_box.scene
            # self.box_info.name = "box"
            self.box_set = CollisionObject()
            self.box_set.header.stamp = self.get_clock().now().to_msg()
            self.box_set.header.frame_id = 'panda_link0'
            self.box_set.pose.position.x = 0.8
            self.box_set.pose.position.y = 0.0
            self.box_set.pose.position.z = 0.0
            self.box_set.id = "Box"
            self.box_prim = SolidPrimitive()
            self.box_prim.type = 1
            self.box_prim.dimensions = [0.2,0.2,0.2]
            self.box_set.primitives = [self.box_prim]
            self.box_info.world.collision_objects = [self.box_set]
            self.get_logger().info(f'result ')
            self.box_pub.publish(self.box_info)
            self.count += 1

def main(args=None):
    rclpy.init(args=args)
    node= AddBox()
    rclpy.spin(node)
    # node.destroy_node()
    rclpy.shutdown()
