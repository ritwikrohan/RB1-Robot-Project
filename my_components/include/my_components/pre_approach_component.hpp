#ifndef PRE_APPROACH_HPP
#define PRE_APPROACH_HPP

#include "my_components/visibility_control.h"
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

namespace my_components
{

    class PreApproach : public rclcpp::Node
    {
    public:
        COMPOSITION_PUBLIC
        explicit PreApproach(const rclcpp::NodeOptions & options);

    private:
        // Declare your class members, including subscriptions, publishers, and callback functions.
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
        rclcpp::CallbackGroup::SharedPtr odom_callback_group_;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::CallbackGroup::SharedPtr timer_callback_group_;
        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
        rclcpp::CallbackGroup::SharedPtr scan_callback_group_;
        int degrees;
        double obstacle;
        double scan_front;
        double initial_yaw = 0.0;
        double current_yaw = 0.0;
        double target_yaw;
        double target_yaw_n;
        double delta_yaw;
        double delta_yaw_n;
        std::vector<float> range;
        geometry_msgs::msg::Twist vel_msg;
        bool goal_reached = false;
        bool linear_goal_reached = false;
        bool angular_goal_reached = false;
        // initial_yaw = current_yaw;

        // Add your member functions, e.g., scanCallback, odomCallback, controlLoop, moveRobot, and others.
        void scanCallback(const std::shared_ptr<sensor_msgs::msg::LaserScan> msg);
        void odomCallback(const std::shared_ptr<nav_msgs::msg::Odometry> msg);
        void controlLoop();
        void moveRobot(int deg, double obstacle);
        // void movescan(double obstacle);
        // void moveodom(int deg);
        double normalizeAngle(double angle);
    };
}

#endif  // PRE_APPROACH_HPP