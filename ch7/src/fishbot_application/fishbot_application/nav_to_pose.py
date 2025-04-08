from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

import rclpy
from rclpy.duration import Duration

def main():
    rclpy.init()
    navigator = BasicNavigator()
    
    # 等待Nav2激活
    navigator.waitUntilNav2Active()
    
    # 设置目标
    goal_pose = PoseStamped()
    goal_pose.header.frame_id = 'map'
    goal_pose.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose.pose.position.x = 1.0
    goal_pose.pose.position.y = 1.0
    goal_pose.pose.orientation.w = 1.0
    
    # 执行导航
    navigator.goToPose(goal_pose)
    
    # 监控导航过程
    while not navigator.isTaskComplete():
        try:
            feedback = navigator.getFeedback()
            if feedback:
                remaining_time = Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9
                navigator.get_logger().info(f"Estimated time to reach goal: {remaining_time:.1f} seconds")
                
                # 超时检查
                if Duration.from_msg(feedback.navigation_time) > Duration(seconds=600.0):
                    navigator.get_logger().warn("Navigation taking too long! Canceling task...")
                    navigator.cancelTask()
        except Exception as e:
            navigator.get_logger().error(f"Error getting feedback: {e}")
            break
    
    # 获取结果
    result = navigator.getResult()
    
    if result == TaskResult.SUCCESS:
        navigator.get_logger().info("Goal succeeded!")
    elif result == TaskResult.CANCELED:
        navigator.get_logger().warn("Goal was canceled!")
    elif result == TaskResult.FAILED:
        navigator.get_logger().error("Goal failed!")
    else:
        navigator.get_logger().error("Goal has an invalid return status!")
    
    # 清理资源
    navigator.destroy_node()
    rclpy.shutdown()