#include <chrono>
#include <functional>
#include <memory>
#include <vector>
#include <cmath>

using namespace std::chrono_literals;

#include "rclcpp/rclcpp.hpp"
#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

using std::placeholders::_1;

const tf2::Vector3 r_com(0.0, -0.19943, 0.0);
const tf2::Vector3 j(0.0, 1, 0.0);
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
        this->cur_position = msg->pose.pose.position;

        tf2::Vector3 cur_position_body;
        tf2::convert(cur_position, cur_position_body);
        tf2::quatRotate(body_wrt_world, cur_position_body);
        this->cur_position.x = cur_position_body.x();
        this->cur_position.y = cur_position_body.y() + r_com.y();
        this->cur_position.z = cur_position_body.z();

        this->cur_twist.angular = msg->twist.twist.angular;

        tf2::Vector3 velocity_link_body;
        tf2::convert(msg->twist.twist.linear, velocity_link_body);


        geometry_msgs::msg::Vector3 velocity_com_body;
        velocity_com_body.x = velocity_link_body.x() + r_com.y() * this->cur_twist.angular.z;
        velocity_com_body.y = velocity_link_body.y();
        velocity_com_body.z = velocity_link_body.z() - (this->cur_twist.angular.x * r_com.y());

        this->cur_twist.linear = tf2::toMsg(velocity_link_body);


        // double pitch, yaw;
        // double rotated_r_com_length_xy = pow((rotated_r_com.x() * rotated_r_com.x() +
        //                                     rotated_r_com.y() * rotated_r_com.y()), 0.5);
        // pitch = atan(rotated_r_com.z() / rotated_r_com.y());
        // if(rotated_r_com.y() >= 0){
        //   yaw = asin(rotated_r_com.x() / rotated_r_com_length_xy);
        // }
        // else{
        //   yaw = asin(rotated_r_com.x() / rotated_r_com_length_xy) + (M_PI / 2);
        // };

        // tf2Scalar yaw, pitch, roll;
        // tf2::Matrix3x3 mat(body_wrt_world);
        // mat.getEulerYPR(yaw, pitch, roll);



        // tf2::Matrix3x3 rot_mat(body_wrt_world);
        // double roll, pitch, yaw;
        // rot_mat.getRPY(roll, pitch, yaw);

        tf2::Vector3 j_rot = tf2::quatRotate(body_wrt_world, j);
        double yaw, pitch;
        pitch = atan(j_rot.z() / j_rot.y());
        if(j_rot.y() >= 0){
          yaw = asin(j_rot.x());
        }
        else{
          yaw = asin(j_rot.x()) + (M_PI / 2);
        };
        this->euler_angle.x = pitch;
        this->euler_angle.y = 0.0;
        this->euler_angle.z = yaw;

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