#!/usr/bin/env python3
"""
方案A：使用原始 octomap_server 输出话题 - 根据官方例程修正
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    LogInfo,
    OpaqueFunction,
)
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os


def launch_setup(context, *args, **kwargs):
    nodes = []
    
    # 获取参数值
    use_sim_time_str = LaunchConfiguration('use_sim_time').perform(context)
    use_sim_time = use_sim_time_str.lower() == 'false'
    octomap_file = os.path.expanduser(LaunchConfiguration('octomap_file').perform(context))
    
    # 启动信息
    nodes.append(LogInfo(msg='根据官方例程修正：使用nav2_recoveries'))
    
    # ========== 1. 必需的TF变换 ==========
    nodes.append(Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_map_to_odom',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    ))
    
    nodes.append(Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_odom_to_base_link',
        arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link'],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    ))
    
    # ========== 2. octomap_server修正配置 ==========
    octomap_params = {
        'frame_id': 'map',
        'base_frame_id': 'base_link',
        'resolution': 0.05,
        'octomap_path': octomap_file,
        'latch': True,
        'publish_2d_map': True,
        'use_height_map': True,
        'publish_free_space': True,
        'occupancy_min_z': 0.1,
        'occupancy_max_z': 2.0,
        'point_cloud_min_z': 0.1,
        'point_cloud_max_z': 2.0,
        'filter_ground_plane': False,
        'use_sim_time': use_sim_time,
    }
    
    nodes.append(Node(
        package='octomap_server',
        executable='octomap_server_node',
        name='octomap_server',
        output='screen',
        parameters=[octomap_params],
        remappings=[
            ('cloud_in', '/none'),
        ]
    ))
    
    # ========== 3. 点云转激光 ==========
    nodes.append(Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'target_frame': 'base_link',
            'transform_tolerance': 0.01,
            'min_height': 0.1,
            'max_height': 1.0,
            'angle_min': -3.14159,
            'angle_max': 3.14159,
            'angle_increment': 0.0175,
            'scan_time': 0.1,
            'range_min': 0.1,
            'range_max': 15.0,
            'use_inf': True,
        }],
        remappings=[
            ('cloud_in', '/unilidar/filter_cloud'),
            ('scan', '/scan')
        ]
    ))
    
    # ========== 4. 加载Nav2参数文件 ==========
    nav2_params_file = PathJoinSubstitution([
        FindPackageShare('scheme_a_2d_projection'),
        'config',
        'nav2_params.yaml'
    ])
    
    # ========== 5. Nav2 导航系统（按照官方例程） ==========
    
    # 5.1 AMCL定位（官方例程没有包含，但我们保留）
    nodes.append(Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[nav2_params_file],
        remappings=[
            ('scan', '/scan'),
            ('map', '/projected_map'),
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static')
        ]
    ))
    
    # 5.2 控制器服务器（按照官方例程）
    nodes.append(Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[nav2_params_file],
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static'),
            ('/map', '/projected_map')
        ]
    ))
    
    # 5.3 规划器服务器（按照官方例程）
    nodes.append(Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[nav2_params_file],
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static'),
            ('/map', '/projected_map')
        ]
    ))
    
    # 5.4 恢复服务器（关键修改：使用recoveries_server）
    nodes.append(Node(
        package='nav2_recoveries',  # 注意：这里是nav2_recoveries，不是nav2_behaviors
        executable='recoveries_server',
        name='recoveries_server',
        output='screen',
        parameters=[nav2_params_file],
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static'),
            ('/map', '/projected_map')
        ]
    ))
    
    # 5.5 行为树导航器（关键：接收goal_pose）
    nodes.append(Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[nav2_params_file],
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static'),
            ('/map', '/projected_map')
        ]
    ))
    
    # 5.6 生命周期管理器（按照官方例程的节点列表）
    lifecycle_nodes = [
        'controller_server',
        'planner_server',
        'recoveries_server',  # 注意：这里是recoveries_server，不是behavior_server
        'bt_navigator',
        # 'waypoint_follower'  # 可选，如果你不需要路径点跟踪
    ]
    
    nodes.append(Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': True,
            'node_names': ['amcl'] + lifecycle_nodes  # 添加amcl到节点列表
        }]
    ))
    
    # ========== 6. RViz可视化 ==========
    nodes.append(Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', PathJoinSubstitution([
            FindPackageShare('scheme_a_2d_projection'),
            'config',
            'navigation_with_projected_map.rviz'
        ])],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    ))

    return nodes


def generate_launch_description():
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='使用仿真时间'
    )
    
    declare_octomap_file = DeclareLaunchArgument(
        'octomap_file',
        default_value='/home/wheeltec/lierrong_ws/src/3d_navigation_suites/scheme_a_2d_projection/maps/output.bt',
        description='八叉树地图文件路径'
    )
    
    return LaunchDescription([
        declare_use_sim_time,
        declare_octomap_file,
        OpaqueFunction(function=launch_setup),
    ])