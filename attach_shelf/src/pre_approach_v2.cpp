#include "attach_shelf/srv/detail/go_to_loading__struct.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/subscription.hpp"
#include "sensor_msgs/msg/detail/laser_scan__struct.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <functional>
#include <memory>
#include <vector>
#include <geometry_msgs/msg/transform_stamped.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include "attach_shelf/srv/go_to_loading.hpp"

using namespace std::chrono_literals;

class AttachShelf : public rclcpp::Node
{
    public:
        AttachShelf() : Node("attach_shelf_node")
        {   
            this->declare_parameter("degrees", 0);
            this->declare_parameter("obstacle", 0.0);
            this->declare_parameter("final_approach", false);
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
            // tf_static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
            odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("/odom", 10, std::bind(&AttachShelf::odomCallback, this, std::placeholders::_1), options_odom);
            vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/robot/cmd_vel", 10);
            timer_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
            timer_ = this->create_wall_timer(50ms,std::bind(&AttachShelf::controlLoop, this), timer_callback_group_);
            scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>("/scan", 10, std::bind(&AttachShelf::scanCallback, this, std::placeholders::_1), options_scan);
            client_ = this->create_client<attach_shelf::srv::GoToLoading>("/approach_shelf");
        }


    private:
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
        rclcpp::CallbackGroup::SharedPtr odom_callback_group_;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::CallbackGroup::SharedPtr timer_callback_group_;
        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
        rclcpp::CallbackGroup::SharedPtr scan_callback_group_;
        std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_;
        rclcpp::Client<attach_shelf::srv::GoToLoading>::SharedPtr client_;
        int degrees;
        double obstacle;
        bool final_approach;
        double scan_front;
        std::vector<float> intensity;
        double initial_yaw = 0.0;
        double current_yaw = 0.0;
        double target_yaw;
        double target_yaw_n;
        double delta_yaw; 
        double delta_yaw_n;
        double x;
        double y;
        // int firstSetLastValue = 0;
        // int secondSetFirstValue = 0;
        // bool foundFirstSet = false;
        std::vector<float> range;

        double normalizeAngle(double angle) {
            while (angle > M_PI) {
                angle -= 2.0 * M_PI;
            }
            while (angle < -M_PI) {
                angle += 2.0 * M_PI;
            }
            return angle;
        }

        void scanCallback(const std::shared_ptr<sensor_msgs::msg::LaserScan> msg)
        {   
            range = msg->ranges;
            scan_front = range[540];
            intensity = msg->intensities;
        }

        void odomCallback(const std::shared_ptr<nav_msgs::msg::Odometry> msg)
        {
            double qx = msg->pose.pose.orientation.x;
            double qy = msg->pose.pose.orientation.y;
            double qz = msg->pose.pose.orientation.z;
            double qw = msg->pose.pose.orientation.w;
            double unnormalized_yaw = atan2(2.0 * (qw * qz + qx * qy), qw * qw + qx * qx - qy * qy - qz * qz);
            current_yaw = normalizeAngle(unnormalized_yaw);
            RCLCPP_DEBUG(this->get_logger(),"Current Yaw: %f", current_yaw);

            // USING TF2

            // tf2::Quaternion q(
            //     msg->pose.pose.orientation.x,
            //     msg->pose.pose.orientation.y,
            //     msg->pose.pose.orientation.z,
            //     msg->pose.pose.orientation.w);
            // tf2::Matrix3x3 m(q);
            // double roll, pitch, yaw;
            // m.getRPY(roll, pitch, yaw);
        }
        void controlLoop()
        {
            this->timer_->cancel();
            degrees = this->get_parameter("degrees").get_parameter_value().get<int>();
            obstacle = this->get_parameter("obstacle").get_parameter_value().get<double>();
            final_approach = this->get_parameter("final_approach").get_parameter_value().get<bool>();

            RCLCPP_DEBUG(this->get_logger(),"Degree Param: %d", degrees);
            RCLCPP_DEBUG(this->get_logger(),"Obstacle Param: %f", obstacle);
            // RCLCPP_INFO(this->get_logger(),"approach Param: %s"final_approach ? "true" : "false");

            moveRobot(degrees, obstacle, final_approach);
        }

        void moveRobot(int deg, double obstacle, bool final_approach)
        {
            geometry_msgs::msg::Twist vel_msg;
            bool goal_reached = false;
            bool linear_goal_reached = false;
            bool angular_goal_reached = false;
            initial_yaw = current_yaw;
            // geometry_msgs::msg::Twist vel_msg;
            delta_yaw = deg * M_PI / 180.0;
            delta_yaw_n = normalizeAngle(delta_yaw);
            // RCLCPP_DEBUG(this->get_logger(),"Delta Yaw: %f", delta_yaw_n);
            target_yaw = initial_yaw + delta_yaw_n;
            target_yaw_n = normalizeAngle(target_yaw);
            // RCLCPP_DEBUG(this->get_logger(),"Target Yaw: %f", target_yaw_n);
            // bool goal_reached = false;
            rclcpp::Rate loop_rate(100);
            while (!goal_reached && rclcpp::ok())
            {
                if (!linear_goal_reached)
                {
                    RCLCPP_DEBUG(this->get_logger(),"scan_front: %f", scan_front);
                    RCLCPP_DEBUG(this->get_logger(),"obstacle: %f", obstacle);
                    if (scan_front>=obstacle && obstacle!=0.0)
                    {
                        vel_msg.linear.x = 0.5;
                        vel_pub_->publish(vel_msg);
                        // RCLCPP_INFO(this->get_logger(),"GOING FORWARD");
                    }
                    else if(obstacle==0.0){
                        vel_msg.linear.x = 0.0;
                        vel_pub_->publish(vel_msg);
                        linear_goal_reached=true;
                        RCLCPP_INFO(this->get_logger(), "Linear goal not specified: %s", linear_goal_reached ? "true" : "false");

                    }
                    else{
                        vel_msg.linear.x = 0.0;
                        vel_pub_->publish(vel_msg);
                        linear_goal_reached=true;
                        RCLCPP_INFO(this->get_logger(), "Linear goal reached: %s", linear_goal_reached ? "true" : "false");

                    }
                }

                else if (!angular_goal_reached && linear_goal_reached)
                {
                    
                    if (std::abs(current_yaw - target_yaw_n) >= 0.01 && std::abs(delta_yaw)<=((2*M_PI)-0.0872665)) 
                    {   
                        RCLCPP_DEBUG(this->get_logger(),"delta_yaw: %f", delta_yaw_n);
                        RCLCPP_DEBUG(this->get_logger(),"current_yaw: %f", current_yaw);
                        RCLCPP_DEBUG(this->get_logger(),"Target Yaw: %f", target_yaw_n);
                        RCLCPP_DEBUG(this->get_logger(), "ERROR: %f", std::abs(current_yaw - target_yaw_n));
                        if (delta_yaw_n > 0) 
                        {
                            vel_msg.angular.z = 0.5;
                            vel_pub_->publish(vel_msg);
                            // RCLCPP_INFO(this->get_logger(),"TURNING LEFT"); 
                        } 
                        else 
                        {
                            vel_msg.angular.z = -0.5;
                            vel_pub_->publish(vel_msg);
                            // RCLCPP_INFO(this->get_logger(),"TURNING RIGHT"); 
                        }
                            
                    }
                    else{
                        vel_msg.angular.z = 0.0;
                        vel_pub_->publish(vel_msg);
                        angular_goal_reached=true;
                        RCLCPP_INFO(this->get_logger(), "Angular goal reached: %s", angular_goal_reached ? "true" : "false");

                    }
                }
                else if (linear_goal_reached && angular_goal_reached)
                {
                    goal_reached=true;
                    RCLCPP_INFO(this->get_logger(), "Pre Approach Goal Reached: %s", goal_reached ? "true" : "false");
                }
                loop_rate.sleep();
            }
            auto request = std::make_shared<attach_shelf::srv::GoToLoading::Request>();
            request->attach_to_shelf = final_approach;
            auto result_future = this->client_->async_send_request(request, std::bind(&AttachShelf::response_callback, this,std::placeholders::_1));
        }

        void response_callback(rclcpp::Client<attach_shelf::srv::GoToLoading>::SharedFuture future) 
        {       
            auto status = future.wait_for(1s);
            if (status == std::future_status::ready) {
                auto result = future.get();    
                
        
                if (result) 
                {
                    RCLCPP_DEBUG(this->get_logger(), "Service returned: %s", result->complete ? "true" : "false");
                    if (result->complete == true)
                    {
                        if (final_approach==true){
                            RCLCPP_INFO(this->get_logger(), "Approach Completed and Shelf loaded");
                        }
                        else {
                            RCLCPP_INFO(this->get_logger(), "Approach Completed and Transform Generated..... check 'rviz2 -d ~/ros2_ws/src/attach_shelf/rviz/config.rviz'");
                        }
                    }
                    else
                    {
                        RCLCPP_INFO(this->get_logger(), "Only one shelf leg or none detected");
                    }
                } 
                else 
                {
                    RCLCPP_ERROR(this->get_logger(), "Service call failed.");
                }
            } 
            else 
            {
                RCLCPP_INFO(this->get_logger(), "Service In-Progress...");
            }
        }

};

int main(int argc, char **argv)
{
    rclcpp::init(argc,argv);
    auto node = std::make_shared<AttachShelf>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    // rclcpp::spin(node);

    rclcpp::shutdown();
}