import rclpy                            
from rclpy.node import Node             
from sensor_msgs.msg import Image
from tf2_msgs.msg import TFMessage
import modern_robotics as mr
from geometry_msgs.msg import TransformStamped
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
class Transform(Node):

    def __init__(self):
        super().__init__("Transform")
        self.raw_sub = self.create_subscription(Image, '/camera/color/image_raw', self.Raw_callback, 10)
        self.tf_sub = self.create_subscription(TFMessage,'/tf',self.TF_callback,10)
        self.tf_transform = self.create_publisher(TFMessage, '/tf_static', 10)
        self.apriltag_1 = StaticTransformBroadcaster(self)
        self.apriltag_2 = StaticTransformBroadcaster(self)
        self.tf_info = TransformStamped()
        self.tf_info_1 = TransformStamped()
        self.tf_info.header.stamp = self.get_clock().now().to_msg()  
        self.tf_info.header.frame_id = 'panda_link0'                      
        self.tf_info.child_frame_id  = 'left_top'   
        self.tf_info_1.header.stamp = self.get_clock().now().to_msg()  
        self.tf_info_1.header.frame_id = 'left_top'                      
        self.tf_info_1.child_frame_id  = 'left_bot'  
    def tf(self,data):
        """ 
        Function: Callback function for detecting the AprilTag
        Args:
            data (TFMessage) - transformation between detected AprilTag and camera frame
        Returns:
            None
        """
        self.k = data.transforms
        self.tf_x0 = self.tf_info[0].transform.translation.x
        self.tf_y0 = self.tf_info[0].transform.translation.y
        self.tf_z0 = self.tf_info[0].transform.translation.z
        # self.orientation1 = self.tf_info[0].transform.rotation
        # self.orientation_list1 = [self.orientation1.x, self.orientation1.y, self.orientation1.z, self.orientation1.w]
        self.tf_xr0 = self.tf_info[0].transform.rotation.x
        self.tf_yr0 = self.tf_info[0].transform.rotation.y
        self.tf_zy0 = self.tf_info[0].transform.rotation.z
        self.tf_w0 = self.tf_info[0].transform.rotation.w
        
        self.tf_x1 = self.tf_info_1[1].transform.translation.x
        self.tf_y1 = self.tf_info_1[1].transform.translation.y
        self.tf_z1 = self.tf_info_1[1].transform.translation.z
        self.tf_xr1 = self.tf_info_1[1].transform.rotation.x
        self.tf_yr1 = self.tf_info_1[1].transform.rotation.y
        self.tf_zy1 = self.tf_info_1[1].transform.rotation.z
        self.tf_w1 = self.tf_info_1[1].transform.rotation.w

        self.apriltag_1.sendTransform(self.tf_info)
        self.apriltag_2.sendTransform(self.tf_info_1)
def main(args=None):
    rclpy.init(args=args)                                
    node = Transform("static_tf_broadcaster")  
    rclpy.spin(node)                                    
    node.destroy_node()                                 
    rclpy.shutdown()