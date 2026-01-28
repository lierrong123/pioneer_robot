#!/usr/bin/env python3
"""
优化版 PioneerBot Gazebo建图启动文件
关键：添加camera_init到odom的变换
"""

from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node


def generate_launch_description():
    """
    启动描述
    """
    
    # ===== 1. 关键TF变换 =====
    # camera_init -> odom 的变换
    # 参数根据实际情况调整：
    # - 默认：完全对齐 [0, 0, 0, 0, 0, 0]
    # - 如果雷达有高度：调整z值
    tf_camera_to_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_camera_to_odom',
        arguments=['0', '0', '0', '0', '0', '0', 'camera_init', 'odom'],
        output='screen'
    )
    
    # ===== 2. 启动所有组件 =====
    
    return LaunchDescription([
        # 先启动TF变换（确保坐标系存在）
        tf_camera_to_odom,
        
        # 同时启动所有其他节点
        ExecuteProcess(
            cmd=['bash', '-c', 'ros2 launch wheeltec_joy only_joy.launch.py'],
            output='screen'
        ),
        
        ExecuteProcess(
            cmd=['bash', '-c', 'ros2 launch pioneerbot_description gazebo_sim.launch.py'],
            output='screen'
        ),
        
        ExecuteProcess(
            cmd=[
                'python3',
                '/home/wheeltec/lierrong_ws/src/unitree_l2_converter/unitree_l2_converter/unitree_l2_converter_node.py'
            ],
            output='screen'
        ),
        
        ExecuteProcess(
            cmd=['bash', '-c', 'ros2 launch point_lio mapping_unilidar_l2.launch.py'],
            output='screen'
        ),
    ])