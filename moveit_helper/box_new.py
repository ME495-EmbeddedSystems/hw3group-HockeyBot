import rclpy
from rclpy.node import Node
from moveit_msgs.msg import CollisionObject, AttachedCollisionObject
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose
from moveit_msgs.msg import AttachedCollisionObject 
# from moveit.planning_interface import PlanningSceneInterface
from moveit_msgs.msg import PlanningScene, PlanningSceneComponents
from moveit_msgs.srv import GetPlanningScene

class AddBox(Node):
    def __init__(self):
        super().__init__('add_box')
        self.box_client = self.create_client(GetPlanningScene, 'get_planning_scene')
        # self.pub_box= self.create_publisher(PlanningScene, 'planning_scene', 10)
        self.ps = PlanningScene()
        self.ps.name=''
        self.ps.robot_state.joint_state.header.stamp= self.get_clock().now().to_msg()
        self.ps.robot_state.joint_state.header.frame_id= 'panda_link0'
        self.ps.robot_state.joint_state.name= ['panda_link0', 'panda_joint1', 'panda_joint2', 'panda_joint3',
                                                 'panda_joint4', 'panda_joint5','panda_joint6', 'panda_joint7',
                                                 'panda_finger_joint1', 'panda_finger_joint2']                                      
        self.ps.robot_state.joint_state.position = [0.0, -0.7853981633974483, 0.0, -2.356194490192345, 0.0, 1.5707963267948966,
                                                    0.7853981633974483,0.035, 0.035]
        self.ps.robot_state.joint_state.velocity= []
        self.ps.robot_state.joint_state.effort= []
        self.ps.robot_state.multi_dof_joint_state.header.stamp = self.get_clock().now().to_msg()
        self.ps.robot_state.multi_dof_joint_state.header.frame_id= 'panda_link0'
        self.ps.robot_state.multi_dof_joint_state.joint_names= []
        self.ps.robot_state.multi_dof_joint_state.transforms= []
        self.ps.robot_state.multi_dof_joint_state.twist= []
        self.ps.robot_state.multi_dof_joint_state.wrench= []
        self.ps.robot_state.attached_collision_objects= []
        self.ps.robot_state.is_diff= False
        self.ps.robot_model_name= 'panda'
        self.ps.fixed_frame_transforms= [] 
        #     stamp:
        #     sec: 0
        #     nanosec: 0
        #     frame_id: panda_link0
        # self.ps.fixed_frame_transforms.child_frame_id: panda_link0
        # self.ps.fixed_frame_transforms.transform:
            # translation:
            # x: 0.0
            # y: 0.0
            # z: 0.0
            # rotation:
            # x: 0.0
            # y: 0.0
            # z: 0.0
            # w: 1.0
        self.ps.allowed_collision_matrix.entry_names = ['panda_hand', 'panda_leftfinger', 
                                                        'panda_link0', 'panda_link1', 'panda_link2', 'panda_link3',
                                                        'panda_link4', 'panda_link5', 'panda_link6', 'panda_link7', 
                                                        'panda_link8', 'panda_rightfinger']
        # self.ps.allowed_collision_matrix.entry_values = [[False,True, False,False, False,True, True, False, True, True, True, True], ]


        self.box = CollisionObject()
        self.box.header.stamp = self.get_clock().now().to_msg()
        self.box.header.frame_id = 'panda_link0'
        self.box.id = 'box1'
        self.primitive = SolidPrimitive()
        self.primitive.type = 1
        # self.primitive.dimensions.resize(3)
        self.primitive.dimensions = [0.5, 0.5, 0.5]
        # self.primitive.dimensions[1] = 0.1
        # self.primitive.dimensions[2] = 0.5
        self.box_pose = Pose()
        self.box_pose.orientation.w = 1.0
        self.box_pose.position.x = 0.2
        self.box_pose.position.y = 0.2
        self.box_pose.position.z = 0.25

        self.box.primitives = [self.primitive]
        self.box.primitive_poses= [self.box_pose]
        self.box.operation = bytes('0', 'utf-8')
        self.ps.world.collision_objects = [self.box]
        self.component = PlanningSceneComponents()
        self.component = 1

        # self.ps = PlanningScene
        # self.ps.world.collision_objects.push_back(self.box)
        self.ps.is_diff = True
        self.future = self.box_client.call_async(GetPlanningScene.Request())
        self.timer = self.create_timer(0.1, self.timer_callback)
    
    def timer_callback(self):
        if self.future.done():
            self.get_logger().info(f'{self.future.result()}')
        # self.pub_box.publish(self.ps)

def main(args=None):
    rclpy.init(args=args)
    node= AddBox()
    rclpy.spin(node)
    rclpy.shutdown()