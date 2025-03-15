#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <turtlesim/msg/pose.hpp>
#include "ch4_interfaces/srv/patrol.hpp"

using Patrol = ch4_interfaces::srv::Patrol;

class TurtleController: public rclcpp::Node {
private:
    double target_x_{1.0};
    double target_y_{1.0};
    double max_speed_{3.0};
    double k_{1.0};

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity_publisher_;
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr pose_subscription_;
    rclcpp::Service<Patrol>::SharedPtr patrol_server_;

public:
    TurtleController(): Node("turtle_controler") {
        // create turtle Twist msg publisher
        this->velocity_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10);
        // create turtle pose msg subscriber
        this->pose_subscription_ = this->create_subscription<turtlesim::msg::Pose>(
            "/turtle1/pose",
            10,
            [this](const turtlesim::msg::Pose::SharedPtr pose) {
                auto msg = geometry_msgs::msg::Twist();
                double cur_x = pose->x;
                double cur_y = pose->y;
                RCLCPP_INFO(this->get_logger(), "current location: (x, y) = (%f, %f)", cur_x, cur_y);

                double distance = std::sqrt(
                    (target_x_ - cur_x) * (target_x_ - cur_x) + (target_y_ - cur_y) * (target_y_ - cur_y)
                );

                double angle = std::atan2(target_y_ - cur_y, target_x_ - cur_x ) - pose->theta;

                if (distance > 0.1) {
                    if (std::fabs(angle) > 0.2) {
                        msg.angular.z = std::fabs(angle);
                    } else {
                        msg.linear.x = k_ * distance;
                    }
                }
                msg.linear.x = std::min(msg.linear.x, max_speed_);
                this->velocity_publisher_->publish(msg);
            }
        );
        // create Patrol Service
        this->patrol_server_ = this->create_service<Patrol>(
            "patrol",
            [&] (const std::shared_ptr<Patrol::Request> request, const std::shared_ptr<Patrol::Response> response)  -> void {
                // handle boundary
                if ((request->target_x > 0 && request->target_x < 12.0f) && (request->target_y > 0 && request->target_y < 12.0f)) {
                    this->target_x_ = request->target_x;
                    this->target_y_ = request->target_y;
                    response->result = Patrol::Response::SUCCESS;
                } else {
                    response->result = Patrol::Response::FAIL;
                }
            }
        );
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TurtleController>();
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}