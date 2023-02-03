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

double K_p_pitch, K_i_pitch, K_d_pitch, N_pitch;
double K_p_yaw, K_i_yaw, K_d_yaw;
double K_p_vel, K_i_vel, K_d_vel;
double r_back_thrust = 0.251383;

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
            K_p_pitch = 4.0;
            K_i_pitch = 4.0;
            K_d_pitch = 2.0;

            K_p_yaw = 4.0;
            K_i_yaw = 4.0;
            K_d_yaw = 2.0;

            K_p_vel = 4.0;
            K_i_vel = 4.0;
            K_d_vel = 2.0;

            // N_sec = 1142.93;
            past_pitch_error = 0.0;
            past_yaw_error = 0.0;
            past_vel_error = 0.0;
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
            double error_pitch = get_yaw(this->cur_desired_vel) - this->cur_euler.x;
            double error_yaw = get_pitch(this->cur_desired_vel) - this->cur_euler.z;
        
            double pitching_pid =  K_p_pitch * error_pitch + K_i_pitch * pitch_error_integral;
            double yawing_pid = K_p_yaw * error_yaw + K_i_yaw * yaw_error_integral;
            double vel_pid = K_p_vel * error_vel + K_i_vel * velocity_error_integral;

            if(this->is_assigned){
                pitching_pid += K_d_pitch * (error_pitch - this->past_pitch_error) / 0.01;
                yawing_pid += K_d_yaw * (error_yaw - this->past_yaw_error) / 0.01;
                vel_pid += K_d_vel * (error_vel - this->past_vel_error) / 0.01;
                this->past_pitch_error = error_pitch;
                this->past_yaw_error = error_yaw;
                this->past_vel_error = error_vel;
            }
            else{
                this->past_pitch_error = error_pitch;
                this->past_yaw_error = error_yaw;
                this->past_vel_error = error_vel;
                this->is_assigned = true;
            };

            double p = this->cur_twist.angular.x;
            double r = this->cur_twist.angular.z;
            double u = this->cur_twist.linear.x;
            double w = this->cur_twist.linear.z;
            double v = this->cur_twist.linear.y;
            double pitching_thrust = pitching_pid + (10 * p * abs(p));
            double yawing_mom = yawing_pid + (10 * r * abs(r));
            double surging_thrust = vel_pid + (10 * v * abs(v)) + (10 * u * r) + (10 * w * p);
            auto wrench_fore = std::make_unique<geometry_msgs::msg::Wrench>();
            auto wrench_fore_reverse = std::make_unique<geometry_msgs::msg::Wrench>();
            auto wrench_port = std::make_unique<geometry_msgs::msg::Wrench>();
            auto wrench_starboard = std::make_unique<geometry_msgs::msg::Wrench>();
            // RCLCPP_INFO(this->get_logger(), "Publishing: %f", error_pitch);
            wrench_fore->force.x = 0.0;
            wrench_fore->force.y = 0.0;
            wrench_fore->force.z = pitching_thrust / (0.36017 - 0.0445);
            wrench_fore->torque.x = 0.0;
            wrench_fore->torque.y = 0.0;
            wrench_fore->torque.z = 0.0;
            fore_pub->publish(std::move(wrench_fore));

            wrench_fore_reverse->force.x = 0.0;
            wrench_fore_reverse->force.y = 0.0;
            wrench_fore_reverse->force.z = -1 * pitching_thrust / (0.36017 - 0.0445);
            wrench_fore_reverse->torque.x = 0.0;
            wrench_fore_reverse->torque.y = 0.0;
            wrench_fore_reverse->torque.z = 0.0;
            fore_reverse_pub->publish(std::move(wrench_fore_reverse));

            wrench_port->force.x = 0.0;
            wrench_port->force.y = 0.0;
            wrench_port->force.z = -1 * (surging_thrust + (yawing_mom / r_back_thrust));
            wrench_port->torque.x = 0.0;
            wrench_port->torque.y = 0.0;
            wrench_port->torque.z = 0.0;
            port_pub->publish(std::move(wrench_port));

            wrench_starboard->force.x = 0.0;
            wrench_starboard->force.y = 0.0;
            wrench_starboard->force.z = -1 * (surging_thrust - (yawing_mom / r_back_thrust));
            wrench_starboard->torque.x = 0.0;
            wrench_starboard->torque.y = 0.0;
            wrench_starboard->torque.z = 0.0;
            starboard_pub->publish(std::move(wrench_starboard));
            
            // RCLCPP_INFO(this->get_logger(), "Publishing: %f", surging_thrust);
            this->pitch_error_integral += 0.01 * error_pitch;
            this->yaw_error_integral += 0.01 * error_yaw;
            this->velocity_error_integral += 0.01 * error_vel;
        }

        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_sub;
        rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr des_vel_sub;
        rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr euler_sub;
        rclcpp::Publisher<geometry_msgs::msg::Wrench>::SharedPtr fore_pub;
        rclcpp::Publisher<geometry_msgs::msg::Wrench>::SharedPtr fore_reverse_pub;
        rclcpp::Publisher<geometry_msgs::msg::Wrench>::SharedPtr starboard_pub;
        rclcpp::Publisher<geometry_msgs::msg::Wrench>::SharedPtr port_pub;
        double velocity_error_integral, pitch_error_integral, yaw_error_integral;
        double filter_state_vel, filter_state_pitch, filter_state_yaw;
        double past_pitch_error, past_yaw_error, past_vel_error;
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