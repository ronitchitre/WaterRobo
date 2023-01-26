#include <chrono>
#include <functional>
#include <memory>
#include <vector>

using namespace std::chrono_literals;

#include "rclcpp/rclcpp.hpp"
#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

using std::placeholders::_1;

const tf2::Vector3 r_com(0.0, 0.19943, 0.0);
// const tf2::Vector3 r_com(0.0, 0.0, 0.0);

class StateEstimator : public rclcpp::Node {
  public:
    StateEstimator() : Node("state_estimator"){
      position_pub = this->create_publisher<geometry_msgs::msg::Point>("/cur_pos", 10);
      twist_pub = this->create_publisher<geometry_msgs::msg::Twist>("/cur_twist", 10);
      euler_ang_pub = this->create_publisher<geometry_msgs::msg::Vector3>("/cur_euler", 10);
      pose_sub = this->create_subscription<nav_msgs::msg::Odometry>("/ideal_sensor", 10,
       std::bind(&StateEstimator::sensor_callback, this, _1));
    };

  private:
    void sensor_callback(const nav_msgs::msg::Odometry::SharedPtr msg){
        tf2::Quaternion body_wrt_world;
        tf2::convert(msg->pose.pose.orientation, body_wrt_world);
        tf2::Vector3 rotated_r_com = tf2::quatRotate(body_wrt_world.inverse(), r_com);
        msg->pose.pose.position.x += rotated_r_com.x();
        msg->pose.pose.position.y += rotated_r_com.y();
        msg->pose.pose.position.z += rotated_r_com.z();

        this->cur_position = msg->pose.pose.position;
        tf2::Vector3 velocity_link_world;
        tf2::convert(msg->twist.twist.linear, velocity_link_world);
        tf2::Vector3 velocity_link_body = tf2::quatRotate(body_wrt_world, velocity_link_world);
        

        tf2::Vector3 ang_vel_world;
        tf2::convert(msg->twist.twist.angular, ang_vel_world);
        tf2::Vector3 ang_vel_body = tf2::quatRotate(body_wrt_world, ang_vel_world);

        geometry_msgs::msg::Vector3 velocity_com_body;
        velocity_com_body.x = velocity_link_body.x();
        velocity_com_body.y = velocity_link_body.y();
        velocity_com_body.z = velocity_link_body.z();

        this->cur_twist.linear = velocity_com_body;
        this->cur_twist.angular = tf2::toMsg(ang_vel_body);

        tf2::Matrix3x3 rot_mat(body_wrt_world);
        double roll, pitch, yaw;
        rot_mat.getRPY(roll, pitch, yaw);
        this->euler_angle.x = yaw;
        this->euler_angle.y = roll;
        this->euler_angle.z = pitch;

        position_pub->publish(cur_position);
        twist_pub->publish(cur_twist);
        euler_ang_pub->publish(euler_angle);
        

    }
    
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr pose_sub;
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr position_pub;
    geometry_msgs::msg::Point cur_position;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub;
    geometry_msgs::msg::Twist cur_twist;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr euler_ang_pub;
    geometry_msgs::msg::Vector3 euler_angle;
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<StateEstimator>());
  rclcpp::shutdown();
  return 0;
}