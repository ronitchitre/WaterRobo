#include <chrono>
#include <functional>
#include <memory>
#include <vector>
#include <cmath>

using namespace std::chrono_literals;

#include "rclcpp/rclcpp.hpp"
#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/wrench.hpp>
#include <nav_msgs/msg/odometry.hpp>
// #include <tf2/LinearMath/Quaternion.h>
// #include <tf2/LinearMath/Vector3.h>


using std::placeholders::_1;

double K_p_sec, K_i_sec, K_d_sec, N_sec;

double norm(geometry_msgs::msg::Vector3 v){
    double norm = (v.x * v.x) + (v.y * v.y) + (v.z * v.z);
    return pow(norm, 0.5);
};

double get_pitch(geometry_msgs::msg::Vector3 v){
    return atan(v.z / v.y);
};

double get_yaw(geometry_msgs::msg::Vector3 vr){
    double yaw;
    if(vr.x > 0 && vr.y >= 0){
        yaw = atan(vr.y / vr.x);
    }
    else if (vr.x < 0 && vr.y >= 0)
    {
        yaw = M_PI + atan(vr.y / (vr.x));
    }
    else if (vr.x < 0 && vr.y <= 0)
    {
        yaw = M_PI + atan(vr.y / (vr.x));
    }
    else if (vr.x > 0 && vr.y <= 0)
    {
        yaw = (2 * M_PI) + atan(vr.y / vr.x);
    }
    else if (vr.x == 0 && vr.y > 0)
    {
        yaw = M_PI / 2;
    }
    
    else{
        yaw = 3 * M_PI / 2;
    }   
    return yaw;
};

class Controller : public rclcpp::Node {
    public:
        Controller() : Node("controller_node"){
            des_vel_sub = this->create_subscription<geometry_msgs::msg::Vector3>(
                "/desired_velocity", 10, std::bind(&Controller::set_goal, this, _1)
            );
            twist_sub = this->create_subscription<geometry_msgs::msg::Twist>(
                "/cur_twist", 10, std::bind(&Controller::set_state, this, _1)
            );
            euler_sub = this->create_subscription<geometry_msgs::msg::Vector3>(
                "/cur_euler", 10, std::bind(&Controller::set_attitude, this, _1)
            );
            fore_pub = this->create_publisher<geometry_msgs::msg::Wrench>("/thrust/fore", 10);
            fore_reverse_pub = this->create_publisher<geometry_msgs::msg::Wrench>("/thrust/fore_reverse", 10);
            starboard_pub = this->create_publisher<geometry_msgs::msg::Wrench>("/thrust/starboard", 10);
            port_pub = this->create_publisher<geometry_msgs::msg::Wrench>("thrust/port", 10);
            velocity_error_integral = 0; pitch_error_integral = 0; yaw_error_integral = 0;
            filter_state_vel= 0; filter_state_pitch = 0; filter_state_yaw = 0;
            timer_ = this->create_wall_timer(100ms, std::bind(&Controller::control_callback, this));
            // K_p_sec = 438.452;
            // K_i_sec = 230.18;
            // K_d_sec = 206.476;
            K_p_sec = 2.0;
            K_i_sec = 0.0;
            K_d_sec = 0.0;
            // N_sec = 1142.93;
            past_pitch_error = 0.0;
        }




    private:
        void set_goal(const geometry_msgs::msg::Vector3::SharedPtr msg){
            this->cur_desired_vel = *msg;
        };

        void set_state(const geometry_msgs::msg::Twist::SharedPtr msg){
            this->cur_twist = *msg;
        };
        void set_attitude(const geometry_msgs::msg::Vector3::SharedPtr msg){
            this->cur_euler = *msg;
        }
        void control_callback(){
            double error_vel = norm(this->cur_desired_vel) - norm(this->cur_twist.linear);
            double error_pitch = -1 * this->cur_euler.x;
            double error_yaw = get_yaw(this->cur_desired_vel) - get_yaw(this->cur_twist.linear);

            double pitching_pid_prop =  K_p_sec * error_pitch + K_i_sec * pitch_error_integral;
            if(this->is_assigned){
                pitching_pid_prop += K_d_sec * (error_pitch - this->past_pitch_error) / 0.01;
            }
            else{
                this->past_pitch_error = pitch_error;
                this->is_assigned = true;
            };

            double p =this->cur_twist.angular.x;
            pitching_pid_prop += 10 * p * abs(p);
            auto wrench_fore = std::make_unique<geometry_msgs::msg::Wrench>();
            RCLCPP_INFO(this->get_logger(), "Publishing: %f", pitching_pid_prop);
            wrench_fore->force.x = 0.0;
            wrench_fore->force.y = 0.0;
            wrench_fore->force.z = pitching_pid_prop / (0.36017 - 0.0445);
            wrench_fore->torque.x = 0.0;
            wrench_fore->torque.y = 0.0;
            wrench_fore->torque.z = 0.0;
            fore_pub->publish(std::move(wrench_fore));

            auto wrench_fore_reverse = std::make_unique<geometry_msgs::msg::Wrench>();
            // RCLCPP_INFO(this->get_logger(), "Publishing: %f", pitching_pid_prop);
            wrench_fore_reverse->force.x = 0.0;
            wrench_fore_reverse->force.y = 0.0;
            wrench_fore_reverse->force.z = -1 * pitching_pid_prop / (0.36017 - 0.0445);
            wrench_fore_reverse->torque.x = 0.0;
            wrench_fore_reverse->torque.y = 0.0;
            wrench_fore_reverse->torque.z = 0.0;
            fore_reverse_pub->publish(std::move(wrench_fore_reverse));
            

            this->pitch_error_integral += 0.01 * error_pitch;


        }

        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_sub;
        rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr des_vel_sub;
        rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr euler_sub;
        rclcpp::Publisher<geometry_msgs::msg::Wrench>::SharedPtr fore_pub;
        rclcpp::Publisher<geometry_msgs::msg::Wrench>::SharedPtr fore_reverse_pub;
        rclcpp::Publisher<geometry_msgs::msg::Wrench>::SharedPtr starboard_pub;
        rclcpp::Publisher<geometry_msgs::msg::Wrench>::SharedPtr port_pub;
        double velocity_error, pitch_error, yaw_error;
        double velocity_error_integral, pitch_error_integral, yaw_error_integral;
        double filter_state_vel, filter_state_pitch, filter_state_yaw;
        double past_pitch_error = 0;
        geometry_msgs::msg::Vector3 cur_desired_vel;
        geometry_msgs::msg::Twist cur_twist;
        geometry_msgs::msg::Vector3 cur_euler;

        bool is_assigned = false;

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Controller>());
  rclcpp::shutdown();
  return 0;
}