#include <memory>
#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2/utils.h"

using namespace std::chrono_literals;

class TFListener: public rclcpp::Node {
private:
    std::shared_ptr<tf2_ros::Buffer> buffer_;
    std::shared_ptr<tf2_ros::TransformListener> listener_;
    rclcpp::TimerBase::SharedPtr timer_;

public:
    TFListener(): Node("tf_listener") {
        this->buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        this->listener_ = std::make_shared<tf2_ros::TransformListener>(*buffer_, this);
        this->timer_ = this->create_wall_timer(5s, std::bind(&TFListener::get_transform, this));
    }
    void get_transform() {
        try {
            const auto transform = buffer_->lookupTransform(
                "base_link", "target_point",
                this->get_clock()->now(),
                rclcpp::Duration::from_seconds(1.0f)
            );

            const auto& translation = transform.transform.translation;
            const auto& rotation = transform.transform.rotation;
            double r, p, y;
            tf2::getEulerYPR(rotation, y, p, r);
            RCLCPP_INFO(this->get_logger(), "translation: (%f, %f, %f)", translation.x, translation.y, translation.z);
            RCLCPP_INFO(this->get_logger(), "rotation: (%f, %f, %f)", r, p, y);

        } catch (tf2::TransformException& ex) {
            RCLCPP_WARN(this->get_logger(), "exception: {%s}", ex.what());
        }
    }
};


int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TFListener>();
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}