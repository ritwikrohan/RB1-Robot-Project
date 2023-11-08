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
#include "my_components/attach_client_component.hpp"

using namespace std::chrono_literals;

namespace my_components
{

    AttachClient::AttachClient(const rclcpp::NodeOptions & options) : Node("attach_client_component", options)
    {   
        timer_ = this->create_wall_timer(50ms,std::bind(&AttachClient::controlLoop, this));
        client_ = this->create_client<attach_shelf::srv::GoToLoading>("/approach_shelf");
    }      


    void AttachClient::controlLoop()
    {
        this->timer_->cancel();
        final_approach = true;

        auto request = std::make_shared<attach_shelf::srv::GoToLoading::Request>();
        request->attach_to_shelf = final_approach;
        auto result_future = this->client_->async_send_request(request, std::bind(&AttachClient::response_callback, this,std::placeholders::_1));
    }

    void AttachClient::response_callback(rclcpp::Client<attach_shelf::srv::GoToLoading>::SharedFuture future) 
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

}


#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(my_components::AttachClient)



// int main(int argc, char **argv)
// {
//     rclcpp::init(argc,argv);
//     auto node = std::make_shared<AttachClient>();
//     rclcpp::executors::MultiThreadedExecutor executor;
//     executor.add_node(node);
//     executor.spin();
//     // rclcpp::spin(node);

//     rclcpp::shutdown();
// }