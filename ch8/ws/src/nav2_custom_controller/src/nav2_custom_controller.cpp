#include "nav2_custom_controller/nav2_custom_controller.hpp"
#include <nav2_util/node_utils.hpp>
#include <nav2_util/geometry_utils.hpp>
#include <nav2_core/exceptions.hpp>

#include <iostream>
#include <algorithm>
#include <string>
#include <chrono>
#include <memory>
#include <thread>

namespace nav2_custom_controller {
    void CustomController::configure(
        const rclcpp_lifecycle::LifecycleNode::WeakPtr& parent,
        std::string name,
        std::shared_ptr<tf2_ros::Buffer> tf,
        std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros
    ) {
        this->node_ = parent.lock();
        this->tf_ = tf;
        this->plugin_name_ = name;
        this->costmap_ros_ = costmap_ros;

        nav2_util::declare_parameter_if_not_declared(
          this->node_, this->plugin_name_ + ".max_linear_speed", rclcpp::ParameterValue(0.1)   
        );
        this->node_->get_parameter(this->plugin_name_ + ".max_linear_speed", this->max_linear_speed_);

        nav2_util::declare_parameter_if_not_declared(
            this->node_, this->plugin_name_ + ".max_angular_speed", rclcpp::ParameterValue(1.0)
        );
        this->node_->get_parameter(this->plugin_name_ + ".max_angular_speed", this->max_angular_speed_);
    };

    void CustomController::cleanup() {
        RCLCPP_INFO(this->node_->get_logger(), "cleanup controller: %s", this->plugin_name_.c_str());
    };

    void CustomController::activate() {
        RCLCPP_INFO(this->node_->get_logger(), "activate controller: %s", this->plugin_name_.c_str());
    };

    void CustomController::deactivate() {
        RCLCPP_INFO(this->node_->get_logger(), "deactivate controller: %s", this->plugin_name_.c_str());
    }

    geometry_msgs::msg::TwistStamped CustomController::computeVelocityCommands(
        const geometry_msgs::msg::PoseStamped& pose, 
        const geometry_msgs::msg::Twist&,
        nav2_core::GoalChecker*
    ) {
        if (this->global_plan_.poses.empty()) { throw nav2_core::PlannerException("Global path is empty."); }
        
        geometry_msgs::msg::PoseStamped pose_in_globalframe;
        if (!nav2_util::transformPoseInTargetFrame(pose, pose_in_globalframe, *this->tf_, global_plan_.header.frame_id, (0.1))) { 
            throw nav2_core::PlannerException("Transform robot pose to global pose failed.");
        }

        auto target_pose = this->getNearestTargetPose(pose_in_globalframe);
        auto angle_diff = this->calculateAngleDiff(pose_in_globalframe, target_pose);

        geometry_msgs::msg::TwistStamped cmd_vel_;
        cmd_vel_.header.frame_id = pose_in_globalframe.header.frame_id;
        cmd_vel_.header.stamp = this->node_->get_clock()->now();

        if (fabs(angle_diff) > M_PI / 10.0) { 
            cmd_vel_.twist.linear.x = .0;
            cmd_vel_.twist.angular.z = fabs(angle_diff) / (angle_diff * this->max_angular_speed_);
        } else {
            cmd_vel_.twist.linear.x = this->max_linear_speed_;
            cmd_vel_.twist.angular.z = .0;
        }

        RCLCPP_INFO(this->node_->get_logger(), "controller: %s send speed: (%f, %f)", this->plugin_name_.c_str(), cmd_vel_.twist.linear.x, cmd_vel_.twist.angular.z);

        return cmd_vel_;
    };

    void CustomController::setPlan(const nav_msgs::msg::Path& path) {
        this->global_plan_ = path;
    };

    void CustomController::setSpeedLimit(const double& speed_limit, const bool& percentage) {
        (void) speed_limit;
        (void) percentage;
    };

    geometry_msgs::msg::PoseStamped CustomController::getNearestTargetPose(
        geometry_msgs::msg::PoseStamped& current_pose
    ) {
        using nav2_util::geometry_utils::euclidean_distance;
        int nearest_idx = 0;
        double nearest_distance = euclidean_distance(current_pose, global_plan_.poses.at(nearest_idx));

        for (uint i = 1; i < global_plan_.poses.size(); i++) {
            double distance = euclidean_distance(current_pose, global_plan_.poses.at(i));
            if (distance < nearest_distance) {
                nearest_idx = i;
                nearest_distance = distance;
            }
        }

        global_plan_.poses.erase(std::begin(global_plan_.poses), std::begin(global_plan_.poses) + nearest_idx);

        if (global_plan_.poses.size() == 1) { return global_plan_.poses.at(0);}

        return global_plan_.poses.at(1);
    };

    double CustomController::calculateAngleDiff(
        const geometry_msgs::msg::PoseStamped& current_pose, 
        const geometry_msgs::msg::PoseStamped& target_pose
    ) {
        float current_robot_yaw = tf2::getYaw(current_pose.pose.orientation);
        float target_angle = std::atan2(target_pose.pose.position.y - current_pose.pose.position.y, target_pose.pose.position.x - current_pose.pose.position.x);
        double angle_diff = target_angle - current_robot_yaw;
        
        if (angle_diff < -M_PI) { angle_diff += 2.0 * M_PI; }
        else if (angle_diff > M_PI) { angle_diff -= 2.0 * M_PI; }

        return angle_diff;
    };
}; // namespace nav2_custom_controller

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(nav2_custom_controller::CustomController, nav2_core::Controller)