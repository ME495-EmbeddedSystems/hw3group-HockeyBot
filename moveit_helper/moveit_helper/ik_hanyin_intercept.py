import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from moveit_msgs.srv import GetPositionIK
from moveit_msgs.action import MoveGroup

class Intercept(Node):
    def __init__(self):
        super().__init__("fake_movegroup")
        self._action_server = ActionServer(self,MoveGroup,"move_action",self.move_action_callback)
        # self.ik = self.create_service(GetPositionIK, "/compute_ik", self.compute_ik_callback)


    def move_action_callback(self, goal):
        self.get_logger().info(f"{goal.request}")
        result = MoveGroup.Result()
        return result
    #  def computer_ik_callback(self,request,response):
    #     return response

def interept_entry(args=None):
    rclpy.init(args=args)
    node = Intercept()
    rclpy.spin(node)
    rclpy.shutdown()
