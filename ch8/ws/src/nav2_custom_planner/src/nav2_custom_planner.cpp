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
        nav_msgs::msg::Path global_path;

        return global_path;
    };
};
