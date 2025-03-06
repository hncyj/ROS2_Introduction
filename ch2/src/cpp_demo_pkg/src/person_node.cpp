#include <iostream>
#include <string>
#include <rclcpp/rclcpp.hpp>

class PersonNode: public rclcpp::Node {
private:
    std::string name_;
    int age_;

public:
    PersonNode(const std::string& node_name, const std::string name, const int age)
    : Node(node_name), name_(name), age_(age) {};

    void eat(const std::string& food) {
        RCLCPP_INFO(this->get_logger(), "%s, %d years old, eating %s now.", this->name_.c_str(), this->age_, food.c_str());
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PersonNode>("cpp_node", "chenyinjie", 18);
    node->eat("apple");
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}