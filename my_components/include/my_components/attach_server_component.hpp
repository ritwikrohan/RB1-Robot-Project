#ifndef ATTACH_SERVER_HPP
#define ATTACH_SERVER_HPP

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
#include "attach_shelf/srv/go_to_loading.hpp"
#include "std_msgs/msg/empty.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "std_msgs/msg/string.hpp"
namespace my_components
{

    class AttachServer : public rclcpp::Node
    {
    public:
        COMPOSITION_PUBLIC
        explicit AttachServer(const rclcpp::NodeOptions & options);

    private:
        rclcpp::Service<attach_shelf::srv::GoToLoading>::SharedPtr approach_server_;
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr elevator_pub_;
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
        bool messagePublished = false;
        std::vector<float> range;
        std::vector<float> intensity;

        void scanCallback(const std::shared_ptr<sensor_msgs::msg::LaserScan> msg);
        void publishStaticTransform(double x, double y);
        void controlLoop();
        double calculateDistanceError(const geometry_msgs::msg::TransformStamped &transform);
        double calculateYawError(const geometry_msgs::msg::TransformStamped &transform);
        void approach_callback(const std::shared_ptr<attach_shelf::srv::GoToLoading::Request> req, const std::shared_ptr<attach_shelf::srv::GoToLoading::Response> res);
    };
}

#endif  // ATTACH_SERVER_HPP