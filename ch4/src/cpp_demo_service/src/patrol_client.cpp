#include <cstdlib>
#include <ctime>
#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "ch4_interfaces/srv/patrol.hpp"
#include <rcl_interfaces/msg/parameter.hpp>
#include <rcl_interfaces/msg/parameter_value.hpp>
#include <rcl_interfaces/msg/parameter_type.hpp>
#include <rcl_interfaces/srv/set_parameters.hpp>

using namespace std::chrono_literals;

using setParam = rcl_interfaces::srv::SetParameters;
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

    std::shared_ptr<setParam::Response> call_set_parameters(rcl_interfaces::msg::Parameter& parameter) {
        auto param_client = this->create_client<setParam>("/turtle_controller/set_parameters");
        while (!param_client->wait_for_service(std::chrono::seconds(1))) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(this->get_logger(), "break when waiting for server...");
                return nullptr;
            }
            RCLCPP_INFO(this->get_logger(), "waiting for server...");
        }

        auto request = std::make_shared<setParam::Request>();
        request->parameters.push_back(parameter);
        auto future = param_client->async_send_request(request);
        rclcpp::spin_until_future_complete(this->get_node_base_interface(), future);
        auto response = future.get();

        return response;
    }

    void update_server_param_k(double k) {
        auto param = rcl_interfaces::msg::Parameter();
        param.name = "k";

        auto param_val = rcl_interfaces::msg::ParameterValue();
        param_val.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
        param_val.double_value = k;
        param.value = param_val;

        auto response = call_set_parameters(param);
        if (response == nullptr) {
            RCLCPP_WARN(this->get_logger(), "paramter set failed.");
            return;
        } else {
            for (auto& result : response->results) {
                if (result.successful) {
                    RCLCPP_INFO(this->get_logger(), "set parameter k as %f", k);
                } else {
                    RCLCPP_WARN(this->get_logger(), "parameter k set failed, due to : %s", result.reason.c_str());
                }
            }
        }
    }
};


int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PatrolClient>();
    node->update_server_param_k(3.0);
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}