#ifndef ATTACH_CLIENT_HPP
#define ATTACH_CLIENT_HPP

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

namespace my_components
{

    class AttachClient : public rclcpp::Node
    {
    public:
        COMPOSITION_PUBLIC
        explicit AttachClient(const rclcpp::NodeOptions & options);

    private:
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Client<attach_shelf::srv::GoToLoading>::SharedPtr client_;
        bool final_approach;

        void controlLoop();
        void response_callback(rclcpp::Client<attach_shelf::srv::GoToLoading>::SharedFuture future);

    };
}

#endif  // ATTACH_CLIENT_HPP