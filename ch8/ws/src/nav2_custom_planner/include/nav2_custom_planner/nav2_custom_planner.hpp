#ifndef NAV2_CUSTOM_PLANNER__NAV2_CUSTOM_PLANNER_HPP_
#define NAV2_CUSTOM_PLANNER__NAV2_CUSTOM_PLANNER_HPP_

#include <memory>
#include <string>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <nav2_core/global_planner.hpp>
#include <nav2_costmap_2d/costmap_2d_ros.hpp>
#include <nav2_util/lifecycle_node.hpp>
#include <nav2_util/robot_utils.hpp>
#include <nav_msgs/msg/path.hpp>

namespace nav2_custom_planner {
    class CustomPlanner: public nav2_core::GlobalPlanner {
    public:
        CustomPlanner() = default;
        ~CustomPlanner() = default;
        
        // plugin configure.
        void configure(
            const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent, std::string name,
            std::shared_ptr<tf2_ros::Buffer> tf, 
            std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros
        ) override;

        // plugin cleaner
        void cleanup() override;
        // plugin activator
        void activate() override;
        // plugin deactivator
        void deactivate() override;

        // create plan from specified start and goal
        nav_msgs::msg::Path createPlan(
            const geometry_msgs::msg::PoseStamped & start,
            const geometry_msgs::msg::PoseStamped & goal
        ) override;

    private:
        // tf buffer ptr
        std::shared_ptr<tf2_ros::Buffer> tf_;
        // node ptr
        nav2_util::LifecycleNode::SharedPtr node_;
        // global costmap ptr
        nav2_costmap_2d::Costmap2D* costmap_;
        // global costmap tf
        std::string global_frame_, name_;
        // interpolate resolution
        double interpolate_resolution_;
    };
}

#endif