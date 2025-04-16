#include <nav2_util/node_utils.hpp>
#include <cmath>
#include <memory>
#include <string>

#include <nav2_core/exceptions.hpp>
#include "../include/nav2_custom_planner/nav2_custom_planner.hpp"

namespace nav2_custom_planner {
    void  CustomPlanner::configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent, std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) {
        this->tf_ = tf;
        this->node_ = parent.lock();
        this->name_ = name;
        this->costmap_ = costmap_ros->getCostmap();
        this->global_frame_ = costmap_ros->getGlobalFrameID();
        this->node_->get_parameter(this->name_ + ".interpolation_resolution", interpolate_resolution_);
        nav2_util::declare_parameter_if_not_declared(node_, this->name_ + ".interpolation_resolution", rclcpp::ParameterValue(0.1));
    };

    void CustomPlanner::cleanup() {
        RCLCPP_INFO(node_->get_logger(), "Cleaning up plugin %s", this->name_.c_str());
        // TODO
        // costmap_ = nullptr;
    };

    void CustomPlanner::activate() {
        RCLCPP_INFO(node_->get_logger(), "Activating plugin %s", this->name_.c_str());
        // TODO
    };

    void CustomPlanner::deactivate() {
        RCLCPP_INFO(node_->get_logger(), "Deactivating plugin %s", this->name_.c_str());
        // TODO
    };

    nav_msgs::msg::Path CustomPlanner::createPlan(const geometry_msgs::msg::PoseStamped & start, const geometry_msgs::msg::PoseStamped & goal) {
        RCLCPP_INFO(node_->get_logger(), "Creating plan from %s to %s", this->name_.c_str(), this->name_.c_str());
        // init global_path
        nav_msgs::msg::Path global_path;
        global_path.poses.clear();
        global_path.header.stamp = this->node_->now();
        global_path.header.frame_id = this->global_frame_;
        
        // check start frame id and goal frame id
        if (start.header.frame_id != this->global_frame_) {
            RCLCPP_ERROR(node_->get_logger(), "Start pose frame id %s does not match global frame id %s", start.header.frame_id.c_str(), this->global_frame_.c_str());
            return global_path;
        }
        if (goal.header.frame_id != this->global_frame_) {
            RCLCPP_ERROR(node_->get_logger(), "Goal pose frame id %s does not match global frame id %s", goal.header.frame_id.c_str(), this->global_frame_.c_str());
            return global_path;
        }

        // calculate current interpolate resolution cycle count and step number
        int cycle_count = std::hypot(goal.pose.position.x - start.pose.position.x, goal.pose.position.y - start.pose.position.y) / interpolate_resolution_;
        double x_increment = (goal.pose.position.x - start.pose.position.x) / cycle_count;
        double y_increment = (goal.pose.position.y - start.pose.position.y) / cycle_count;


        // # A Pose with reference coordinate frame and timestamp
        // std_msgs/Header header
        //         builtin_interfaces/Time stamp
        //                 int32 sec
        //                 uint32 nanosec
        //         string frame_id
        // Pose pose
        //         Point position
        //                 float64 x
        //                 float64 y
        //                 float64 z
        //         Quaternion orientation
        //                 float64 x 0
        //                 float64 y 0
        //                 float64 z 0
        //                 float64 w 1

        // create path
        for (int i = 0; i < cycle_count; ++i) {
            // create target pose for every loop
            geometry_msgs::msg::PoseStamped pose;
            pose.pose.position.x = start.pose.position.x + x_increment * i;
            pose.pose.position.y = start.pose.position.y + y_increment * i;
            pose.pose.position.z = 0.0;
            pose.header.stamp = this->node_->now();
            pose.header.frame_id = this->global_frame_;
            global_path.poses.emplace_back(pose);
        }

        for (const auto& pose : global_path.poses) {
            // convert pose tf to grid tf
            unsigned int gx, gy;
            if (this->costmap_->worldToMap(pose.pose.position.x, pose.pose.position.y, gx, gy)) {
                unsigned char cost = this->costmap_->getCost(gx, gy);
                if (cost == nav2_costmap_2d::LETHAL_OBSTACLE) {
                    RCLCPP_WARN(this->node_->get_logger(), "detect obstacle in (%f, %f)", pose.pose.position.x, pose.pose.position.y);
                    throw nav2_core::PlannerException(
                        "Unable create path: " + std::to_string(goal.pose.position.x) + ", " + std::to_string(goal.pose.position.y)
                    );
                }
            }
        }

        geometry_msgs::msg::PoseStamped goal_pose = goal;
        goal_pose.header.stamp = this->node_->now();
        goal_pose.header.frame_id = this->global_frame_;
        global_path.poses.emplace_back(goal_pose);

        return global_path;
    };
};

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(nav2_custom_planner::CustomPlanner, nav2_core::GlobalPlanner)