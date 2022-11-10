import rclpy
from rclpy.node import Node
from moveit_msgs.msg import CollisionObject, AttachedCollisionObject
from shape_msgs.msg import SolidPrimitive
from moveit_interface.srv import Addobj
from std_msgs.msg import Header
from moveit_msgs.msg import CollisionObject
from moveit_msgs.msg import AttachedCollisionObject 
from moveit_msgs.msg import PlanningScene, PlanningSceneComponents
from moveit_msgs.srv import GetPlanningScene
from enum import Enum, auto

class State(Enum):
    """ The 3 states of the system.
        Determines what the main timer function and other callback should be doing on each iteration
        depending on the state
    """
    INITIAL = auto(),
    GETTING = auto(),
    GOTIT = auto()


class AddBox(Node):
    def __init__(self):
        super().__init__('add_box')
        self.state = State.INITIAL
        self.box_pub = self.create_publisher(PlanningScene, "/planning_scene", 10)
        self.box_client = self.create_client(GetPlanningScene, 'get_planning_scene')
        self.goal = self.create_service(Addobj, "/add_obj", self.obj_service)
        self.robot_state_order = PlanningSceneComponents()
        self.robot_state_order.components = 0
        self.count = 0
        self.box_future_client = self.box_client.call_async(GetPlanningScene.Request(components=self.robot_state_order))
        # self.box_info = PlanningScene()
        # rclpy.spin_until_future_complete(self, self.box_future_clinet)
        # self.box_info = self.box_future_clinet.result()
        # self.box_pub.publish(self.box_info)
        self.timer = self.create_timer(0.1, self.timer_callback)

    def obj_service(self, request, response):
        self.state = State.GETTING
        self.id = request.id
        self.pos_x=request.x
        self.pos_y=request.y
        self.pos_z=request.z
        self.dim_x=request.dim_x
        self.dim_y=request.dim_y
        self.dim_z=request.dim_z

        return response

    def timer_callback(self):
        if self.state == State.GETTING:
            if self.box_future_client.done():
                self.get_logger().info(f'done ')
                self.input_box = self.box_future_client.result()
                self.state = State.GOTIT
        if self.state == State.GOTIT:
            self.box_info= self.input_box.scene
            self.box_set = CollisionObject()
            self.box_set.header.stamp = self.get_clock().now().to_msg()
            self.box_set.header.frame_id = 'panda_link0'
            self.box_set.pose.position.x = self.pos_x
            self.box_set.pose.position.y = self.pos_y
            self.box_set.pose.position.z = self.pos_z
            self.box_set.id = "Box"+ str(self.id)
            self.box_prim = SolidPrimitive()
            self.box_prim.type = 1
            self.box_prim.dimensions = [self.dim_x,self.dim_y,self.dim_z]
            self.box_set.primitives = [self.box_prim]
            self.box_info.world.collision_objects = [self.box_set]
            self.get_logger().info(f'result ')
            self.box_pub.publish(self.box_info)
            self.state = State.INITIAL

def main(args=None):
    rclpy.init(args=args)
    node= AddBox()
    rclpy.spin(node)
    # node.destroy_node()
    rclpy.shutdown()
