import rclpy
from rclpy.node import Node 
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from tf_transformations import quaternion_from_euler

class DynamicTFBroadcaster(Node):
    def __init__(self):
        super().__init__('dynamic_tf_broadcaster')
        self.tf_broadcaster_ = TransformBroadcaster(self)
        self.timer_ = self.create_timer(1, self.publish_transform)
        
    def publish_transform(self):
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = "camera_link"
        transform.child_frame_id = "bottle_link"
        
        transform.transform.translation.x = 0.2
        transform.transform.translation.y = 0.0
        transform.transform.translation.z = 0.5
        
        rotation_quaternion = quaternion_from_euler(0, 0, 0)
        transform.transform.rotation.x = rotation_quaternion[0]
        transform.transform.rotation.y = rotation_quaternion[1]
        transform.transform.rotation.z = rotation_quaternion[2]
        transform.transform.rotation.w = rotation_quaternion[3]
        
        self.tf_broadcaster_.sendTransform(transform)
        

def main():
    rclpy.init()
    tf_node = DynamicTFBroadcaster()
    rclpy.spin(tf_node)
    rclpy.shutdown()
        