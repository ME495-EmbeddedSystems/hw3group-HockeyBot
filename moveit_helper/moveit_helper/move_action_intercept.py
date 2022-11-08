import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer

from moveit_msgs.action import MoveGroup

class Intercept(Node):
    def __init__(self):
        super().__init__("fake_movegroup")
        self._action_server = ActionServer(
            self,
            MoveGroup,
            "move_action",
            self.move_action_callback
        )

    def move_action_callback(self, goal):
        self.get_logger().info(f"{goal.request}")
        # get_result.async()
        result = MoveGroup.Result()


        # self.get_logger().info(f"{goal.result.request}")
        return result

def interept_entry(args=None):
    rclpy.init(args=args)
    # node = Intercept()
    # rclpy.spin(node)
    # rclpy.shutdown()


    action_client = Intercept()
    future = action_client.send_goal()
    rclpy.spin_until_future_complete(action_client, future)

