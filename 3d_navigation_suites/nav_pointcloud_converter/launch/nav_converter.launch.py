#!/usr/bin/env python3
"""
启动导航点云转换器
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='使用仿真时间'
        ),
        
        LogInfo(msg='启动导航点云转换器'),
        
        # 点云转换器节点
        Node(
            package='nav_pointcloud_converter',
            executable='nav_converter_node',
            name='nav_pointcloud_converter',
            output='screen',
            parameters=[{
                'input_topic': '/unilidar/cloud',
                'output_topic': '/unilidar/filter_cloud',
                'debug_topic': '/unilidar/cloud_debug',
                'robot_self_topic': '/robot_self_points',
                
                # 激光雷达安装参数（从URDF获取）
                'lidar_position.x': 0.08484,
                'lidar_position.y': 0.0,
                'lidar_position.z': 0.289262,
                'lidar_orientation.pitch': -0.6978888,  # 40度向下倾斜
                
                # 机器人尺寸（base_link坐标系）
                'robot_size.x': 0.6,  # X方向尺寸
                'robot_size.y': 0.5,  # Y方向尺寸
                'robot_size.z': 0.4,  # Z方向尺寸
                
                # 过滤参数
                'min_distance': 0.1,
                'max_distance': 30.0,
            }]
        ),
    ])