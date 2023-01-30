#include <chrono>
#include <functional>
#include <memory>

using namespace std::chrono_literals;

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/wrench.hpp"

class ForcePublisher : public rclcpp::Node
{
    public:
    ForcePublisher() : Node("force_publisher"){
        fore_publisher_ = this->create_publisher<geometry_msgs::msg::Wrench>("/thrust/fore", 10);
        starboard_publisher_ = this->create_publisher<geometry_msgs::msg::Wrench>("/thrust/starboard", 10);
        port_publisher_ = this->create_publisher<geometry_msgs::msg::Wrench>("thrust/port", 10);
        timer_ = this->create_wall_timer(500ms, std::bind(&ForcePublisher::timer_callback, this));
    };
    private:
    void timer_callback(){
        auto wrench_fore = std::make_unique<geometry_msgs::msg::Wrench>();
        wrench_fore->force.x = 0.0;
        wrench_fore->force.y = 0.0;
        wrench_fore->force.z = -0.0;
        wrench_fore->torque.x = 0.0;
        wrench_fore->torque.y = 0.0;
        wrench_fore->torque.z = 0.0;
        fore_publisher_->publish(std::move(wrench_fore));

        auto wrench_starboard = std::make_unique<geometry_msgs::msg::Wrench>();
        wrench_starboard->force.x = 0.0;
        wrench_starboard->force.y = 0.0;
        wrench_starboard->force.z = -1.0;
        wrench_starboard->torque.x = 0.0;
        wrench_starboard->torque.y = 0.0;
        wrench_starboard->torque.z = 0.0;
        starboard_publisher_->publish(std::move(wrench_starboard));

        auto wrench_port = std::make_unique<geometry_msgs::msg::Wrench>();
        wrench_port->force.x = 0.0;
        wrench_port->force.y = 0.0;
        wrench_port->force.z = -2.0;
        wrench_port->torque.x = 0.0;
        wrench_port->torque.y = 0.0;
        wrench_port->torque.z = 0.0;
        port_publisher_->publish(std::move(wrench_port));
    };

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<geometry_msgs::msg::Wrench>::SharedPtr fore_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::Wrench>::SharedPtr starboard_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::Wrench>::SharedPtr port_publisher_;
};

int main(int argc, char * argv[]){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ForcePublisher>();
    rclcpp::spin(node);
    return 0;
}