#include "rclcpp/logging.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/subscription.hpp"
#include "sensor_msgs/msg/detail/laser_scan__struct.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/detail/empty__struct.hpp"
#include "std_msgs/msg/empty.hpp"
#include <cmath>
#include <functional>
#include <memory>
#include <vector>
#include <geometry_msgs/msg/transform_stamped.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include "attach_shelf/srv/go_to_loading.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include <thread>

using namespace std::chrono_literals;

class FinalApproachService : public rclcpp::Node
{
    public:
        FinalApproachService() : Node("approach_service_server_node")
        {
            this->approach_server_ = this->create_service<attach_shelf::srv::GoToLoading>("/approach_shelf", std::bind(&FinalApproachService::approach_callback, this, std::placeholders::_1, std::placeholders::_2)); 
            scan_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
            rclcpp::SubscriptionOptions options_scan;
            options_scan.callback_group = scan_callback_group_;
            scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>("/scan", 10, std::bind(&FinalApproachService::scanCallback, this, std::placeholders::_1), options_scan);
            elevator_pub_ = this->create_publisher<std_msgs::msg::Empty>("/elevator_up", 1);
            robot_cmd_vel_publisher = this->create_publisher<geometry_msgs::msg::Twist>("/robot/cmd_vel", 1);
            tf_static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
            tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
            tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        }




    private:
        rclcpp::Service<attach_shelf::srv::GoToLoading>::SharedPtr approach_server_;
        rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr elevator_pub_;
        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
        std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_;
        rclcpp::CallbackGroup::SharedPtr scan_callback_group_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
        std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr robot_cmd_vel_publisher;
        int firstSetLastValue;
        int secondSetFirstValue;
        double kp_yaw = 0.7;
        double kp_distance = 0.5;
        double x;
        double y;
        std::vector<float> range;
        std::vector<float> intensity;


        void scanCallback(const std::shared_ptr<sensor_msgs::msg::LaserScan> msg)
        {   
            firstSetLastValue = 0;
            secondSetFirstValue = 0;
            bool foundFirstSet = false;
            range = msg->ranges;
            // scan_front = range[540];
            intensity = msg->intensities;
            for (size_t i =1;i<=intensity.size();i++)
            {
                // if (intensity[i]!=0.0){
                // // RCLCPP_INFO(this->get_logger(),"Index: %d", i);
                // // RCLCPP_INFO(this->get_logger(),"intensity: %f", intensity[i]);
                // }
                
                if (intensity[i-1] > 0 && intensity[i+1] == 0 && !foundFirstSet) 
                {
                    firstSetLastValue = i;
                    foundFirstSet = true;
                } 
                else if (intensity[i-1]==0 && intensity[i+1]>0 && foundFirstSet && intensity[i]>0)
                {
                    secondSetFirstValue = i;
                    break;
                }
                
            }

            double angle_right = msg->angle_min+(firstSetLastValue*msg->angle_increment);
            double angle_left = msg->angle_min+(secondSetFirstValue*msg->angle_increment);
            double side_right = range[firstSetLastValue];
            double side_left = range[secondSetFirstValue];

            this->x = side_right*cos(angle_right);
            double x1 = side_left*cos(angle_left);
            double right = side_right*sin(angle_right);
            double left = side_left*sin(angle_left);
            this->y = (right+left)/2;
            RCLCPP_DEBUG(this->get_logger(),"first_last: %d", firstSetLastValue);
            RCLCPP_DEBUG(this->get_logger(),"second_first: %d", secondSetFirstValue);
            RCLCPP_DEBUG(this->get_logger(),"X: %f", x);
            RCLCPP_DEBUG(this->get_logger(),"X1: %f", x1);
            RCLCPP_DEBUG(this->get_logger(),"right: %f", right);
            RCLCPP_DEBUG(this->get_logger(),"left: %f", left);
            RCLCPP_DEBUG(this->get_logger(),"Y: %f", y);
            // publishStaticTransform(x, y);



            // int index = msg->angle_min + ((range.size())/2) * msg->angle_increment
            RCLCPP_DEBUG(this->get_logger(), "Size %ld", range.size());
            RCLCPP_DEBUG(this->get_logger(), "Scan at front: %f", range[540]);
        }

        void publishStaticTransform(double x, double y) {
            // Create a transform message
            geometry_msgs::msg::TransformStamped broadcast_transformStamped;
            broadcast_transformStamped.header.stamp = this->now();
            broadcast_transformStamped.header.frame_id = "robot_front_laser_base_link";  // Set the parent frame ID
            broadcast_transformStamped.child_frame_id = "cart_frame";  // Set the child frame ID

            // Set the translation (x, y, z)
            broadcast_transformStamped.transform.translation.x = x;
            broadcast_transformStamped.transform.translation.y = y;
            broadcast_transformStamped.transform.translation.z = 0.0;  // Z is usually 0 for 2D transforms

            // Set the rotation (no rotation in a static transform)
            broadcast_transformStamped.transform.rotation.x = 0.0;
            broadcast_transformStamped.transform.rotation.y = 0.0;
            broadcast_transformStamped.transform.rotation.z = 0.0;
            broadcast_transformStamped.transform.rotation.w = 1.0;

            // Publish the static transform
            tf_static_broadcaster_->sendTransform(broadcast_transformStamped);
        }

        void controlLoop() 
        {   
            bool final_goal = false;
            bool should_set_final_goal = false;
            rclcpp::Rate loop_rate(100);
            while(!final_goal)
            {   
                publishStaticTransform(this->x, this->y);
                if ((tf_buffer_->canTransform("robot_base_link", "cart_frame", tf2::TimePoint(), tf2::durationFromSec(0.5)))&& (!std::isinf(this->x)|| !std::isinf(this->y)) && !should_set_final_goal)
                {   
                    try {
                        geometry_msgs::msg::TransformStamped transform;
                        transform = tf_buffer_->lookupTransform("robot_base_link", "cart_frame", tf2::TimePoint(), tf2::durationFromSec(1.0));

                        double error_distance = calculateDistanceError(transform);
                        double error_yaw = calculateYawError(transform);
                        RCLCPP_DEBUG(this->get_logger(),"error_d : %f", error_distance);
                        RCLCPP_DEBUG(this->get_logger(),"error_y : %f", error_yaw);
                        geometry_msgs::msg::Twist twist;
                        if (error_distance < 0.5 && error_yaw < 0.1){
                            should_set_final_goal = true;
                        }
                        else{
                            twist.linear.x = kp_distance * error_distance; 
                            twist.angular.z = kp_yaw * error_yaw;
                            robot_cmd_vel_publisher->publish(twist);
                        }


                    } catch (tf2::TransformException &ex) {
                        RCLCPP_INFO(this->get_logger(), "TF Exception: %s", ex.what());
                    }
                }
                else if (should_set_final_goal)
                {   
                    // final_goal=true;
                    
                    rclcpp::Time start_time = this->now();
                    std::chrono::seconds duration(4); // Move for 4 seconds
                    rclcpp::Rate small(100);
                    while (rclcpp::ok() && (this->now() - start_time) < duration) {
                        // Your control logic here
                        geometry_msgs::msg::Twist default_twist;
                        default_twist.linear.x = 0.2;  
                        default_twist.angular.z = 0.0;  
                        robot_cmd_vel_publisher->publish(default_twist);
                        small.sleep();
                    }

                    // Stop the robot after 3 seconds
                    geometry_msgs::msg::Twist stop_twist;
                    stop_twist.linear.x = 0.0;
                    stop_twist.angular.z = 0.0;
                    robot_cmd_vel_publisher->publish(stop_twist);
                    final_goal = true;
                }   
                loop_rate.sleep();
            }
        }

        double calculateDistanceError(const geometry_msgs::msg::TransformStamped &transform) {
            return sqrt(((transform.transform.translation.x)+0.3) * ((transform.transform.translation.x)+0.3) +
                        transform.transform.translation.y * transform.transform.translation.y);
        }

        double calculateYawError(const geometry_msgs::msg::TransformStamped &transform) {
            return atan2(transform.transform.translation.y, transform.transform.translation.x);
        }

        void approach_callback(const std::shared_ptr<attach_shelf::srv::GoToLoading::Request> req, const std::shared_ptr<attach_shelf::srv::GoToLoading::Response> res) 
        {
            bool attach_action = req->attach_to_shelf;
            if (attach_action)
            {
                if (firstSetLastValue!=0 && secondSetFirstValue!=0)
                {
                    RCLCPP_INFO(this->get_logger(),"Approaching Shelf and picking it up.");
                    controlLoop();
                    std::this_thread::sleep_for(std::chrono::seconds(2));
                    auto message = std_msgs::msg::Empty();
                    elevator_pub_->publish(message);
                    res->complete = true;
                }
                else 
                {
                    RCLCPP_INFO(this->get_logger(),"Only one or no leg detected.");
                    res->complete=false;
                }
            }
            else 
            {
                publishStaticTransform(x, y);
                res->complete = true;
            }
        }
};
int main(int argc, char **argv)
{
    rclcpp::init(argc,argv);
    auto node = std::make_shared<FinalApproachService>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    // rclcpp::spin(node);

    rclcpp::shutdown();
}