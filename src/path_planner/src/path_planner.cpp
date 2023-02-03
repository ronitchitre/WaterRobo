#include <chrono>
#include <functional>
#include <memory>
#include <vector>

using namespace std::chrono_literals;

#include "rclcpp/rclcpp.hpp"
#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/pose.hpp>

using std::placeholders::_1;

// geometry_msgs::msg::Point rotate(geometry_msgs::msg::Point point, geometry_msgs::msg::Quaternion quaternion){
//     float6
// }

const double zeta = 1.0;
double goal_x = 5.0; double goal_y = 5.0; double goal_z = 0.5;
const double eta = 1.0;
const double q_star = 2.0;

double distance(std::vector<double> q1, std::vector<double> q2){
    return pow(pow(q1[0] - q2[0], 2) + pow(q1[1] - q1[1], 2) + pow(q1[2] - q1[2], 2), 0.5);
}

std::vector<double> unit_vec(std::vector<double> q1, std::vector<double> q2){
    double normalize = distance(q1, q2);
    if(std::abs(normalize) <= std::numeric_limits<double>::epsilon()){
        return std::vector<double> {0.0, 0.0, 0.0};
    }
    std::vector<double> unit_vec = {(q2[0] - q1[0])/normalize, (q2[1] - q1[1])/normalize, (q2[2] - q1[2])/normalize};
    return unit_vec;
}

class VelocityPublisher : public rclcpp::Node
{
    public:
        VelocityPublisher() : Node("path_planner"){
            cur_pose_sub = this->create_subscription<geometry_msgs::msg::Point>(
                "/cur_pos", 10, std::bind(&VelocityPublisher::set_position_callback, this, _1));

            vel_publisher = this->create_publisher<geometry_msgs::msg::Vector3>("/desired_velocity", 10);

            timer_ = this->create_wall_timer(2000ms, std::bind(&VelocityPublisher::desired_vel_publish, this));

            this->obstacle_list.push_back(this->obstacle); 

        };

    private:
        void desired_vel_publish(){
            // if(!is_assigned){
            //     return;
            // };
            this->desired_velocity.x = -1 * zeta * (this->cur_pose.x - goal_x);
            this->desired_velocity.y = -1 * zeta * (this->cur_pose.y - goal_y);
            this->desired_velocity.z = -1 * zeta * (this->cur_pose.z - goal_z);
            for(int i = 0; i < int(this->obstacle_list.size()); i++){
                std::vector<double> cur_obstacle = this->obstacle_list[i];
                std::vector<double> cur_postion = {this->cur_pose.x, this->cur_pose.y, this->cur_pose.z};
                double distance_to_ob = distance(cur_postion, cur_obstacle);
                if(distance_to_ob <= q_star){
                    std::vector<double> unit_vec_ob = unit_vec(cur_obstacle, cur_postion);
                    double scaling = eta * ((1 / q_star) - (1 / distance_to_ob)) * pow(distance_to_ob, -2);
                    this->desired_velocity.x += scaling * unit_vec_ob[0];
                    this->desired_velocity.y += scaling * unit_vec_ob[1];
                    this->desired_velocity.z += scaling * unit_vec_ob[2];
                }
            };

            this->vel_publisher->publish(desired_velocity);

        }

        void set_position_callback(const geometry_msgs::msg::Point::SharedPtr msg) {
            // this->cur_pose.x = msg->x;
            // this->cur_pose.y = msg->y;
            // this->cur_pose.z = msg->z;
            this->cur_pose = *msg;
        }


        rclcpp::TimerBase::SharedPtr timer_;
        // rclcpp::Publisher<geometry_msgs::msg::Vector3> publisher_;
        std::vector<std::vector<double>> obstacle_list;
        std::vector<double> obstacle = {3.0, 3.0, 0.0};      
        rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr cur_pose_sub; 
        geometry_msgs::msg::Point cur_pose;
        geometry_msgs::msg::Vector3 desired_velocity;
        bool is_assigned = false;
        rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr vel_publisher;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VelocityPublisher>());
  rclcpp::shutdown();
  return 0;
}