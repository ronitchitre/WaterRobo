#include <chrono>
#include <functional>
#include <memory>
#include <vector>

using namespace std::chrono_literals;

#include "rclcpp/rclcpp.hpp"
#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/wrench.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>

using std::placeholders::_1;

class Controller : public rclcpp::Node {
    public:




    private:
        void link_states_callback(const nav_msgs::msg::Odometry::SharedPtr msg){
            geometry_msgs::msg::Vector3 vel_link = msg->twist.twist.linear;

        }
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr pose_sub;
        rclcpp::Publisher<geometry_msgs::msg::Wrench>::SharedPtr fore_pub;
        rclcpp::Publisher<geometry_msgs::msg::Wrench>::SharedPtr starboard_pub;
        rclcpp::Publisher<geometry_msgs::msg::Wrench>::SharedPtr port_pub;
        double state_array[9];

};