#include <cstdlib>
#include <ctime>
#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "ch4_interfaces/srv/patrol.hpp"

using namespace std::chrono_literals;
using Patrol = ch4_interfaces::srv::Patrol;

class PatrolClient: public rclcpp::Node {
private:
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Client<Patrol>::SharedPtr patrol_client_;

public:
    PatrolClient(): Node("patrol_client") {
        this->patrol_client_ = this->create_client<Patrol>("patrol");
        this->timer_ = this->create_wall_timer(10s, std::bind(&PatrolClient::timer_callback, this));
    }

    void timer_callback() {
        while (!patrol_client_->wait_for_service(std::chrono::seconds(1))) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(this->get_logger(), "break occur when waiting for server...");
                return;
            }
            RCLCPP_INFO(this->get_logger(), "wating for server...");
        }

        auto request = std::make_shared<Patrol::Request>();
        request->target_x = std::rand() % 15;
        request->target_y = std::rand() % 15;

        RCLCPP_INFO(this->get_logger(), "request for patrol, position: (%f, %f)", request->target_x, request->target_y);

        this->patrol_client_->async_send_request(
            request,
            [&](rclcpp::Client<Patrol>::SharedFuture result_future) -> void {
                auto response = result_future.get();
                if (response->result == Patrol::Response::SUCCESS) {
                    RCLCPP_INFO(this->get_logger(), "send target position success!");
                } else if (response->result == Patrol::Response::FAIL) {
                    RCLCPP_INFO(this->get_logger(), "send target position failed!");
                }
            }
        );
    }
};


int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PatrolClient>();
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}


