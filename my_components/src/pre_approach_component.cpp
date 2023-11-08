#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/subscription.hpp"
#include "rmw/types.h"
#include "sensor_msgs/msg/detail/laser_scan__struct.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <functional>
#include <memory>
#include <vector>
#include <geometry_msgs/msg/transform_stamped.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include "my_components/pre_approach_component.hpp"
#include <thread>

using namespace std::chrono_literals;
namespace my_components
{
    PreApproach::PreApproach(const rclcpp::NodeOptions & options) : Node("pre_approach_component", options)
    {
        // this->declare_parameter("degrees", 0);
        // this->declare_parameter("obstacle", 0.0);
        odom_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        rclcpp::SubscriptionOptions options_odom;
        options_odom.callback_group = odom_callback_group_;
        scan_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        rclcpp::SubscriptionOptions options_scan;
        options_scan.callback_group = scan_callback_group_;
        rclcpp::QoS qos_odom_profile(10);
        // qos_odom_profile.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
        // qos_odom_profile.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);
        // qos_odom_profile.liveliness(RMW_QOS_POLICY_LIVELINESS_AUTOMATIC);
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("/diffbot_base_controller/odom", 10, std::bind(&PreApproach::odomCallback, this, std::placeholders::_1));
        vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/diffbot_base_controller/cmd_vel_unstamped", 10);
        timer_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        timer_ = this->create_wall_timer(40ms,std::bind(&PreApproach::controlLoop, this));
        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>("/scan", 10, std::bind(&PreApproach::scanCallback, this, std::placeholders::_1));
    }

    // Implement the member functions of your node, e.g., scanCallback, odomCallback, controlLoop, moveRobot, and normalizeAngle.

    double PreApproach::normalizeAngle(double angle)
    {
        while (angle > M_PI)
        {
            angle -= 2.0 * M_PI;
        }
        while (angle < -M_PI)
        {
            angle += 2.0 * M_PI;
        }
        return angle;
    }

    void PreApproach::scanCallback(const std::shared_ptr<sensor_msgs::msg::LaserScan> msg)
    {
        range = msg->ranges;
        scan_front = range[540];
        
        RCLCPP_DEBUG(this->get_logger(), "Size %ld", range.size());
        // RCLCPP_DEBUG(this->get_logger(), "Scan at front: %f", range[540]);
        RCLCPP_DEBUG(this->get_logger(), "HI SCANNER");
    }

    void PreApproach::odomCallback(const std::shared_ptr<nav_msgs::msg::Odometry> msg)
    {
        double qx = msg->pose.pose.orientation.x;
        double qy = msg->pose.pose.orientation.y;
        double qz = msg->pose.pose.orientation.z;
        double qw = msg->pose.pose.orientation.w;
        double unnormalized_yaw = atan2(2.0 * (qw * qz + qx * qy), qw * qw + qx * qx - qy * qy - qz * qz);
        current_yaw = normalizeAngle(unnormalized_yaw);
        // RCLCPP_DEBUG(this->get_logger(),"Current Yaw: %f", current_yaw);
        RCLCPP_DEBUG(this->get_logger(), "HI ODOM");
    }

    void PreApproach::controlLoop()
    {
        // this->timer_->cancel();
        // degrees = this->get_parameter("degrees").get_parameter_value().get<int>();
        // obstacle = this->get_parameter("obstacle").get_parameter_value().get<double>();
        degrees = -90;
        obstacle = 0.3;
        moveRobot(degrees, obstacle);
    }

    void PreApproach::moveRobot(int deg, double obstacle)
    {
         RCLCPP_DEBUG(this->get_logger(), "HI MOVE ROBOT");
        // geometry_msgs::msg::Twist vel_msg;
        // bool goal_reached = false;
        // bool linear_goal_reached = false;
        // bool angular_goal_reached = false;
        // initial_yaw = current_yaw;
        // geometry_msgs::msg::Twist vel_msg;
        delta_yaw = deg * M_PI / 180.0;
        delta_yaw_n = normalizeAngle(delta_yaw);
        // RCLCPP_DEBUG(this->get_logger(),"Delta Yaw: %f", delta_yaw_n);
        target_yaw = initial_yaw + delta_yaw_n;
        target_yaw_n = normalizeAngle(target_yaw);
        // RCLCPP_DEBUG(this->get_logger(),"Target Yaw: %f", target_yaw_n);
        // bool goal_reached = false;
        // std::this_thread::sleep_for(std::chrono::seconds(3));
        // rclcpp::Rate loop_rate(100);
        if(!goal_reached)
        {
            if (!linear_goal_reached)
            {
                RCLCPP_DEBUG(this->get_logger(),"scan_front: %f", scan_front);
                RCLCPP_DEBUG(this->get_logger(),"obstacle: %f", obstacle);
                if (scan_front>=obstacle && obstacle!=0.0)
                {
                    vel_msg.linear.x = 0.5;
                    vel_pub_->publish(vel_msg);
                    RCLCPP_DEBUG(this->get_logger(),"GOING FORWARD");
                }
                else if(obstacle==0.0){
                    vel_msg.linear.x = 0.0;
                    vel_pub_->publish(vel_msg);
                    linear_goal_reached=true;
                    RCLCPP_DEBUG(this->get_logger(), "No Linear goal Given");

                }
                else{
                    vel_msg.linear.x = 0.0;
                    vel_pub_->publish(vel_msg);
                    linear_goal_reached=true;
                    RCLCPP_DEBUG(this->get_logger(), "Linear Goal Reached");

                }
            }

            else if (!angular_goal_reached && linear_goal_reached)
            {
                
                if ((current_yaw - target_yaw_n) >= -0.08 && std::abs(delta_yaw)<=((2*M_PI)-0.0872665)) 
                {   
                    RCLCPP_DEBUG(this->get_logger(),"delta_yaw: %f", delta_yaw_n);
                    RCLCPP_DEBUG(this->get_logger(),"current_yaw: %f", current_yaw);
                    RCLCPP_DEBUG(this->get_logger(),"Target Yaw: %f", target_yaw_n);
                    RCLCPP_DEBUG(this->get_logger(), "ERROR: %f", std::abs(current_yaw - target_yaw_n));
                    if (delta_yaw_n > 0) 
                    {
                        vel_msg.angular.z = 0.5;
                        vel_pub_->publish(vel_msg);
                        // RCLCPP_DEBUG(this->get_logger(),"TURNING LEFT"); 
                    } 
                    else 
                    {
                        vel_msg.angular.z = -0.5;
                        vel_pub_->publish(vel_msg);
                        // RCLCPP_DEBUG(this->get_logger(),"TURNING RIGHT"); 
                    }
                        
                }
                else{
                    vel_msg.angular.z = 0.0;
                    vel_pub_->publish(vel_msg);
                    angular_goal_reached=true;
                    RCLCPP_DEBUG(this->get_logger(), "Angular Goal Reached");

                }
            }
            else if (linear_goal_reached && angular_goal_reached)
            {
                goal_reached=true;
                RCLCPP_INFO(this->get_logger(), "Pre Approach Complete");
                this->timer_->cancel();
            }
            // loop_rate.sleep();
        }
        RCLCPP_DEBUG(this->get_logger(),"Waiting for further action");
    }
}

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(my_components::PreApproach)


// int main(int argc, char **argv)
// {
//     rclcpp::init(argc, argv);
//     auto node = std::make_shared<PreApproach>();
//     rclcpp::executors::MultiThreadedExecutor executor;
//     executor.add_node(node);
//     executor.spin();
//     rclcpp::shutdown();
// }