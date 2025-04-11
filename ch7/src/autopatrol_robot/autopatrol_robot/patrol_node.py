import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

from tf2_ros import TransformListener, Buffer
from tf_transformations import euler_from_quaternion, quaternion_from_euler
from rclpy.duration import Duration


class PatrolNode(BasicNavigator):
    def __init__(self, node_name='patrol_node'):
        super().__init__(node_name)
        self.declare_parameter('initial_point', [0.0, 0.0, 0.0])
        self.declare_parameter('target_point', [0.0, 0.0, 0.0, 1.0, 1.0, 1.57])
        self.initial_point_ = self.get_parameter('initial_point').value
        self.target_points_ = self.get_parameter('target_points').value
        self.buffer_ = Buffer()
        self.listener_ = TransformListener(self.buffer_, self)
            
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
        if result == TaskResult.SUCCESS:
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
    patrol.init_robot_pose()
    
    while rclpy.ok():
        points = patrol.get_target_points()
        for point in points:
            x, y, yaw = point[0], point[1], point[2]
            target_pose = patrol.get_pose_by_xyyaw(x, y, yaw)
            patrol.nav_to_pose(target_pose)
            
    rclpy.shutdown()
    
    