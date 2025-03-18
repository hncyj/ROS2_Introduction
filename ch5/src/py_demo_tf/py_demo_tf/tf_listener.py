import rclpy
from rclpy.node import Node
from tf2_ros import TransformListener, Buffer
from tf_transformations import euler_from_quaternion

class TFListener(Node):
    def __init__(self):
        super().__init__('tf_listener')
        self.buffer_ = Buffer()
        self.listener_ = TransformListener(self.buffer_, self)
        self.timers_ = self.create_timer(1, self.get_transform)
        
    def get_transform(self):
        try:
            result = self.buffer_.lookup_transform(
                'base_link',
                'bottle_link',
                rclpy.time.Time(seconds=0), 
                rclpy.time.Duration(seconds=1)
            )
            
            transform = result.transform
            rotation_euler = euler_from_quaternion(
                [
                    transform.rotation.x,
                    transform.rotation.y,
                    transform.rotation.z,
                    transform.rotation.w
                ]
            )    
            self.get_logger().info(f"translation: {transform.translation}, quaternion: {transform.rotation}, euler: {rotation_euler}")
        except Exception as e:
            self.get_logger().warn(f"can not get transform : {e}")
            

def main():
    rclpy.init()
    tf_listener = TFListener()
    rclpy.spin(tf_listener)
    rclpy.shutdown()