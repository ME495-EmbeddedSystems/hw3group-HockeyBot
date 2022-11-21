import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import GripperCommand
from moveit_interface.srv import Initial, Goal, Execute, Addobj, GripperSrv

class Gripper(Node):
    def __init__(self):
        super().__init__("gripper")

        self.gripper_action_client = ActionClient(
            self,
            GripperCommand,
            "panda_gripper/gripper_action"
            )

        self.gripper_service = self.create_service(GripperSrv, "/gripper_service", self.gripper_service_callback)


    def gripper_service_callback(self, request, response):
        if request.open == True:
            self.gripper(open=True)
        elif request.open == False:
            self.gripper(open=False)

        return response

    def gripper(self, open):
        """
        open: boolean
        open = True if gripper should be open
        open = False if gripper should be closed
        """
        gripper_msg = GripperCommand.Goal()
        if open is False:
            gripper_msg.command.position = 0.01
            gripper_msg.command.max_effort = 60.0
        elif open is True:
            gripper_msg.command.position = 0.04

        if self.gripper_action_client.server_is_ready():
            self.future_gripper_action_request = self.gripper_action_client.send_goal_async(gripper_msg)
            self.future_gripper_action_request.add_done_callback(self.gripper_response_callback)
    
    def gripper_response_callback(self, future):
        """
        Check if the gripper response is received.

        Returns
        -------
            None

        """
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Unable to change gripper state')
            return

        self.get_logger().info('Changing gripper state')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.gripper_result_callback)
    
    def gripper_result_callback(self, future):
        """
        Store the gripper future result in a variable.

        Keyword arguments:
            gripper_result: Store gripper future result

        Returns
        -------
            None

        """
        self.gripper_result = future.result().result
        

def gripper_entry(args=None):
    rclpy.init(args=args)
    node = Gripper()
    rclpy.spin(node)
    rclpy.shutdown()
