import rclpy
from rclpy.node import Node
from std_srvs.srv import Empty
from geometry_msgs.msg import PoseStamped  
class Goal(Node):
    def __init__(self):
        super().__init__("goal")
        self.goal = self.create_service(Empty, "/goal_service", self.goal_service)
        self.timer = self.create_timer(0.05, self.timer_callback)
        self.goal_pose = self.create_publisher(PoseStamped, '/goal_pose', 10)
    def goal_service(self, request, response):
        self.goal_x=request.x
        self.goal_y=request.y
        self.goal_z=request.z
        return Empty.Response()
    def timer_callback(self):
        msg = PoseStamped()                                            
        msg.pose.position.x=self.goal_x
        msg.pose.position.y=self.goal_y
        msg.pose.position.z=self.goal_z                                  
        self.goal_pose.publish(msg)                                     
        self.get_logger().info('goal_service')
def main(args=None):
    rclpy.init(args=args)
    node = Goal()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
