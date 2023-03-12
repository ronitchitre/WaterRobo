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
double max_thrust = 50;

double norm(geometry_msgs::msg::Vector3 v){
    double norm = (v.x * v.x) + (v.y * v.y) + (v.z * v.z);
    return pow(norm, 0.5);
};

double get_pitch(geometry_msgs::msg::Vector3 v){
    if(abs(norm(v) >= 0.001)){
        return asin(v.z / norm(v));
    }
    else{
        return 0;
    }
};

double get_yaw(geometry_msgs::msg::Vector3 vr){
    double yaw;
    if(vr.y > 0 && vr.x >= 0){
        yaw = atan(vr.x / vr.y);
    }
    else if (vr.y < 0 && vr.x >= 0)
    {
        yaw = M_PI + atan(vr.x / (vr.y));
    }
    else if (vr.y < 0 && vr.x <= 0)
    {
        yaw = M_PI + atan(vr.x / (vr.y));
    }
    else if (vr.y > 0 && vr.x <= 0)
    {
        yaw = (2 * M_PI) + atan(vr.x / vr.y);
    }
    else if (vr.y == 0 && vr.x > 0)
    {
        yaw = M_PI / 2;
    }
    
    else if(vr.y == 0 && vr.x < 0){
        yaw = 3 * M_PI / 2;
    }
    else{
        yaw = 0.0;
    }
    return yaw;
};

double norm_xy(geometry_msgs::msg::Vector3 v){
    double norm = (v.x * v.x) + (v.y * v.y);
    return pow(norm, 0.5);
};

double yaw_between(geometry_msgs::msg::Vector3 a, geometry_msgs::msg::Vector3 b){
    if(norm_xy(b) == 0 || norm_xy(a) == 0){
        return 0.0;
    }
    double dot_prod = a.x * b.x + a.y * b.y;
    double dot_prod_unit = dot_prod / (norm_xy(a) * norm_xy(b));
    return acos(dot_prod_unit);
}

class Controller : public rclcpp::Node {
    public:
        Controller() : Node("controller_node"){
            des_vel_sub = this->create_subscription<geometry_msgs::msg::Vector3>(
                "/desired_velocity", 10, std::bind(&Controller::set_goal, this, _1)
            );
            twist_sub = this->create_subscription<geometry_msgs::msg::Twist>(
                "/cur_twist", 10, std::bind(&Controller::set_state, this, _1)
            );
            j_world_sub = this->create_subscription<geometry_msgs::msg::Vector3>(
                "/j_world", 10, std::bind(&Controller::set_attitude, this, _1)
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
            K_p_pitch = 40.0;
            K_i_pitch = 10.0;
            K_d_pitch = 10.0;

            K_p_yaw = 30.0;
            K_i_yaw = 0.0;
            K_d_yaw = 5.0;

            K_p_vel = 30.0;
            K_i_vel = 5.0;
            K_d_vel = 5.0;

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
            this->cur_j_world = *msg;
        }
        void control_callback(){
            double error_vel = norm(this->cur_desired_vel) - cur_twist.linear.y;
            double error_pitch = get_pitch(this->cur_desired_vel) - get_pitch(this->cur_j_world);
            // geometry_msgs::msg::Vector3 vec;
            // vec.x = 1 / pow(2, 0.5); vec.y = -1 / pow(2, 0.5); vec.z = 0;
            double error_yaw = get_yaw(this->cur_desired_vel) - get_yaw(this->cur_j_world);
            // RCLCPP_INFO(this->get_logger(), "Publishing: after %f", error_pitch);
            if(error_yaw >= M_PI){
                error_yaw = error_yaw - (2.0 * M_PI); 
            }
            if(error_yaw <= -1 * M_PI){
                error_yaw = error_yaw + (2.0 * M_PI);
            }

            // if(norm(this->cur_desired_vel) == 0.0){
            //     return;
            // };
        
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
            double pitching_thrust = pitching_pid;
            // pitching_thrust = 0;
            double yawing_mom = yawing_pid + (10 * r * abs(r));
            // yawing_mom = 0;
            double surging_thrust = vel_pid;
            // surging_thrust = 0;

            if(abs(surging_thrust) >= max_thrust){
                surging_thrust = max_thrust * surging_thrust / abs(surging_thrust);
            }
            if(abs(yawing_mom) >= max_thrust){
                yawing_mom = max_thrust * yawing_mom / abs(yawing_mom);
            }
            if(abs(pitching_thrust) >= max_thrust){
                pitching_thrust = max_thrust * pitching_thrust / abs(pitching_thrust);
            }

            auto wrench_fore = std::make_unique<geometry_msgs::msg::Wrench>();
            auto wrench_fore_reverse = std::make_unique<geometry_msgs::msg::Wrench>();
            auto wrench_port = std::make_unique<geometry_msgs::msg::Wrench>();
            auto wrench_starboard = std::make_unique<geometry_msgs::msg::Wrench>();

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
            
            // RCLCPP_INFO(this->get_logger(), "Publishing: %f", error_yaw * 180 /M_PI);
            this->pitch_error_integral += 0.01 * error_pitch;
            this->yaw_error_integral += 0.01 * error_yaw;
            this->velocity_error_integral += 0.01 * error_vel;
            // RCLCPP_INFO(this->get_logger(), "Publishing: after %f %f", surging_thrust, yawing_mom);
        }

        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_sub;
        rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr des_vel_sub;
        rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr j_world_sub;
        rclcpp::Publisher<geometry_msgs::msg::Wrench>::SharedPtr fore_pub;
        rclcpp::Publisher<geometry_msgs::msg::Wrench>::SharedPtr fore_reverse_pub;
        rclcpp::Publisher<geometry_msgs::msg::Wrench>::SharedPtr starboard_pub;
        rclcpp::Publisher<geometry_msgs::msg::Wrench>::SharedPtr port_pub;
        double velocity_error_integral, pitch_error_integral, yaw_error_integral;
        double filter_state_vel, filter_state_pitch, filter_state_yaw;
        double past_pitch_error, past_yaw_error, past_vel_error;
        geometry_msgs::msg::Vector3 cur_desired_vel;
        geometry_msgs::msg::Twist cur_twist;
        geometry_msgs::msg::Vector3 cur_j_world;

        bool is_assigned = false;

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Controller>());
  rclcpp::shutdown();
  return 0;
}