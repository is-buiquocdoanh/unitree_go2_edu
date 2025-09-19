from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Đường dẫn tới config
    pkg_share = get_package_share_directory('go2_robot_sdk')


    return LaunchDescription([
        # Node chạy cartographer
        Node(
            package='cartographer_ros',
            executable='cartographer_node',
            name='cartographer_node',
            output='screen',
            parameters=[{
                "use_sim_time": False
            }],
            arguments=[
                "-configuration_directory", os.path.join(pkg_share, "config"),
                "-configuration_basename", "lds_2d.lua"
            ],
            remappings=[
                ("/scan", "/scan")  # nếu laser topic khác thì sửa lại
            ]
        ),

        # Node phát map cho RViz/Nav2
        Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        name='cartographer_occupancy_grid_node',
        output='screen',
        parameters=[{"resolution": 0.05, "publish_period_sec": 1.0}]
        ),
        
        # Node chạy RViz
        # Node(
        #     package='rviz2',
        #     executable='rviz2',
        #     name='rviz2',
        #     output='screen',
        #     arguments=['-d', os.path.join(pkg_share, 'rviz', 'cartographer.rviz')],
        #     parameters=[{"use_sim_time": False}]
        # ) 
    ])
