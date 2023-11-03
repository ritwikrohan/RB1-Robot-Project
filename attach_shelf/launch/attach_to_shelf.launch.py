from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():

    degrees_arg = DeclareLaunchArgument(
        "degrees", default_value="0"
    )
    obstacle_arg = DeclareLaunchArgument(
        "obstacle", default_value="0.0"
    )

    final_arg = DeclareLaunchArgument(
        "final_approach", default_value="false"
    )

    degrees_f = LaunchConfiguration('degrees')
    obstacle_f = LaunchConfiguration('obstacle')
    final_approach_f = LaunchConfiguration('final_approach')

    pre_approach_v2 = Node(
        package='attach_shelf',
        executable='pre_approach_node_v2',
        output='screen',
        name='pre_approach_v2',
        emulate_tty=True,
        parameters=[{'degrees': degrees_f, 'obstacle': obstacle_f, "final_approach": final_approach_f}]
        # arguments=["-degrees", degrees_f,
        #            "-obstacle", obstacle_f
        #            # "-final_approach", final_approach_f,
        #            ]

    )

    approach_service_server = Node(
        package='attach_shelf',
        executable='approach_service_server_node',
        output='screen',
        name='approach_service_server',
        emulate_tty=True,
        
    )

    # create and return launch description object
    return LaunchDescription(
        [
            degrees_arg,
            obstacle_arg,
            final_arg,
            pre_approach_v2,
            approach_service_server
        ]
    )
