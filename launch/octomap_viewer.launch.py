#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'octomap_file',
            default_value='/home/wheeltec/lierrong_ws/octomaps/map.bt',
            description='Path to OctoMap .bt file'
        ),
        DeclareLaunchArgument(
            'frame_id',
            default_value='map',
            description='Frame ID for the octomap'
        ),
        DeclareLaunchArgument(
            'resolution',
            default_value='0.05',
            description='OctoMap resolution'
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time'
        ),
        
        # Octomap服务器节点
        Node(
            package='octomap_server',
            executable='octomap_server_node',
            name='octomap_server',
            output='screen',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'frame_id': LaunchConfiguration('frame_id'),
                'octomap_path': LaunchConfiguration('octomap_file'),
                'resolution': LaunchConfiguration('resolution'),
                'publish_2d_map': True,
                'publish_3d_map': True,
                'publish_free_space': False,
                'latch': True,
                'pointcloud_min_z': -1.0,
                'pointcloud_max_z': 10.0,
                'occupancy_min_z': -1.0,
                'occupancy_max_z': 10.0,
                'filter_ground': False,
                'ground_filter_distance': 0.04,
                'ground_filter_angle': 0.15,
                'ground_filter_plane_distance': 0.07,
            }]
        ),
    ])
