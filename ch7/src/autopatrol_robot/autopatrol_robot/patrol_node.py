import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

from tf2_ros import TransformListener, Buffer
from tf_transformations import euler_from_quaternion, quaternion_from_euler
from rclpy.duration import Duration

from autopatrol_interfaces.srv import SpeachText

from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class PatrolNode(BasicNavigator):
    def __init__(self, node_name='patrol_node'):
        super().__init__(node_name)
        self.declare_parameter('initial_point', [0.0, 0.0, 0.0])
        self.declare_parameter('target_points', [0.0, 0.0, 0.0, 1.0, 1.0, 1.57])
        self.initial_point_ = self.get_parameter('initial_point').value
        self.target_points_ = self.get_parameter('target_points').value
        self.buffer_ = Buffer()
        self.listener_ = TransformListener(self.buffer_, self)
        self.speach_client_ = self.create_client(SpeachText, 'speech_text')
        
        # save img
        self.declare_parameter('image_save_path', '')
        self.image_save_path = self.get_parameter('image_save_path').value
        self.bridge = CvBridge()
        self.latest_image = None
        self.subscription_image = self.create_subscription(
            Image, '/camera_sensor/image_raw', self.image_callback, 10
        )
        
    def image_callback(self, msg):
        self.latest_image = msg
        
    def record_image(self):
        if self.latest_image is not None:
            try:
                pose = self.get_current_pose()
                cv_image = self.bridge.imgmsg_to_cv2(self.latest_image)
                
                import os
                save_path = self.image_save_path
                
                if not save_path:
                    save_path = os.path.join(os.getcwd(), "patrol_images")
                    self.get_logger().warn(f"No image_save_path specified, using default: {save_path}")
                
                os.makedirs(save_path, exist_ok=True)
                
                if not save_path.endswith('/') and not save_path.endswith('\\'):
                    save_path = save_path + '/'
                    
                filename = f"{save_path}image_{pose.translation.x:3.2f}_{pose.translation.y:3.2f}.png"
                success = cv2.imwrite(filename, cv_image)
                
                if success:
                    self.get_logger().info(f"Image saved to {filename}")
                else:
                    self.get_logger().error(f"Failed to write image to {filename}")
                    
            except Exception as e:
                self.get_logger().error(f"Error in record_image: {e}")
        else:
            self.get_logger().warn("No image available to save")
        
    def speach_text(self, text):
        while not self.speach_client_.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for speech service...')
            
        request = SpeachText.Request()
        request.text = text
        future = self.speach_client_.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result() is not None:
            result = future.result().result
            if result:
                self.get_logger().info(f"Speech result: {result}")
            else:
                self.get_logger().error("Failed to speak text")
        
        else:
            self.get_logger().error("Service call failed")
            
    def get_pose_by_xyyaw(self, x, y, yaw):
        """
        Get a pose by x, y, yaw

        Args:
            x (_type_): _description_
            y (_type_): _description_
            yaw (_type_): _description_
        """
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.pose.position.x = x
        pose.pose.position.y = y
        rotation_quat = quaternion_from_euler(0, 0, yaw)
        pose.pose.orientation.x = rotation_quat[0]
        pose.pose.orientation.y = rotation_quat[1]
        pose.pose.orientation.z = rotation_quat[2]
        pose.pose.orientation.w = rotation_quat[3]
        
        return pose
    
    def init_robot_pose(self):
        self.initial_point_ = self.get_parameter('initial_point').value
        self.setInitialPose(self.get_pose_by_xyyaw(
            self.initial_point_[0],
            self.initial_point_[1],
            self.initial_point_[2]
        ))
        self.waitUntilNav2Active()
    
    def get_target_points(self):
        points = []
        self.target_points_ = self.get_parameter('target_points').value
        
        for index in range(int(len(self.target_points_) / 3)):
            x = self.target_points_[index * 3 + 0]
            y = self.target_points_[index * 3 + 1]
            yaw = self.target_points_[index * 3 + 2]
            points.append([x, y, yaw])
            self.get_logger().info(f"Target point {index}: {x}, {y}, {yaw}")
            
        return points
    
    def nav_to_pose(self, target_pose):
        self.waitUntilNav2Active()
        result = self.goToPose(target_pose)
        
        while not self.isTaskComplete():
            feedback = self.getFeedback()
            if feedback:
                self.get_logger().info(f"Expect Duration: {Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9} seconds")
        
        result = self.getResult()
        if result == TaskResult.SUCCEEDED:
            self.get_logger().info("Navigation to target pose succeeded")
        elif result == TaskResult.CANCELED:
            self.get_logger().warn("Navigation to target pose canceled")
        elif result == TaskResult.FAILED:
            self.get_logger().error("Navigation to target pose failed")
        else:
            self.get_logger().error("Navigation to target pose unknown result")
            
    def get_current_pose(self):
        while rclpy.ok():
            try:
                tf = self.buffer_.lookup_transform(
                    'map', 'base_footprint', rclpy.time.Time(seconds=0), rclpy.time.Duration(seconds=1)
                )
                transform = tf.transform
                rotation_euler = euler_from_quaternion(
                    [
                        transform.rotation.x,
                        transform.rotation.y,
                        transform.rotation.z,
                        transform.rotation.w
                    ]
                )
                self.get_logger().info(f" translation: {transform.translation}, quat: {transform.rotation}, euler: {rotation_euler}")
                
                return transform
                
            except Exception as e:
                self.get_logger().error(f"Transform error: {e}")
                

def main():
    rclpy.init()
    patrol = PatrolNode()
    patrol.speach_text(text="initial pose .........")
    patrol.init_robot_pose()
    patrol.speach_text(text="initial pose done !!!!")
    
    while rclpy.ok():
        points = patrol.get_target_points()
        for point in points:
            x, y, yaw = point[0], point[1], point[2]
            target_pose = patrol.get_pose_by_xyyaw(x, y, yaw)
            patrol.speach_text(text=f"Going to point {x}, {y}, {yaw}")
            patrol.nav_to_pose(target_pose)
            patrol.speach_text(text=f"Arrive at target pos: {x}, {y}, {yaw}, ready for record image.")
            patrol.record_image()
            patrol.speach_text(text=f"image record!")
            
    rclpy.shutdown()