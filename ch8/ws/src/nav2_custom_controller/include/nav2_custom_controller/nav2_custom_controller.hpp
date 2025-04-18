#ifndef NAV2_CUSTOM_CONTROLLER__NAV2_CUSTOM_CONTROLLER_H
#define NAV2_CUSTOM_CONTROLLER__NAV2_CUSTOM_CONTROLLER_H

#include <memory>
#include <vector>
#include <string>

#include <nav2_core/controller.hpp>
#include <rclcpp/rclcpp.hpp>
#include <nav2_util/robot_utils.hpp>

namespace nav2_custom_controller {
    class CustomController: public nav2_core::Controller {
    public:
        CustomController() = default;
        ~CustomController() override = default;
        
        void configure(
            const rclcpp_lifecycle::LifecycleNode::WeakPtr& parent,
            std::string name,
            std::shared_ptr<tf2_ros::Buffer> tf,
            std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros
        ) override;

        void cleanup() override;
        void activate() override;
        void deactivate() override;

        geometry_msgs::msg::TwistStamped computeVelocityCommands(
            const geometry_msgs::msg::PoseStamped& pose, 
            const geometry_msgs::msg::Twist& velocity,
            nav2_core::GoalChecker* goalchecker
        ) override;

        void setPlan(const nav_msgs::msg::Path& path) override;
        void setSpeedLimit(const double& speed_limit, const bool& percentage) override;

    protected:
        std::string plugin_name_;
        std::shared_ptr<tf2_ros::Buffer> tf_; // transform buffer ptr
        std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
        nav2_util::LifecycleNode::SharedPtr node_;
        nav2_costmap_2d::Costmap2D* costmap_;
        nav_msgs::msg::Path global_plan_;
        double max_angular_speed_;
        double max_linear_speed_;

        geometry_msgs::msg::PoseStamped getNearestTargetPose(geometry_msgs::msg::PoseStamped& current_pose);
        double calculateAngleDiff(const geometry_msgs::msg::PoseStamped& current_pose, const geometry_msgs::msg::PoseStamped& target_pose);
    };
};



#endif // NAV2_CUSTOM_CONTROLLER__NAV2_CUSTOM_CONTROLLER_H