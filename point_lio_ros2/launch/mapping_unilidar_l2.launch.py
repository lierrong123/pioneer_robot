from launch import LaunchDescription
from launch.actions import GroupAction, DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # 声明参数
    rviz_arg = DeclareLaunchArgument(
        'rviz', default_value='true',
        description='是否启动RViz')
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='是否使用仿真时间')
    
    # 参数：输入输出话题
    input_topic_arg = DeclareLaunchArgument(
        'input_topic', default_value='/unilidar/cloud',
        description='原始点云输入话题')
    
    output_topic_arg = DeclareLaunchArgument(
        'output_topic', default_value='/point_lio/cloud_registered',
        description='转换后点云输出话题')

    # Unitree L2点云转换节点
    pointcloud_converter = Node(
        package='unitree_l2_converter',  # 改为新包的名称
        executable='unitree_l2_converter',  # 可执行文件名
        name='unitree_l2_converter',
        output='screen',
        parameters=[{
            'input_topic': LaunchConfiguration('input_topic'),
            'output_topic': LaunchConfiguration('output_topic'),
            'scan_lines': 40,
            'scan_rate': 5.5,
        }]
        # 不需要remappings，因为在节点内部已经通过参数设置了话题
    )

    # Point-LIO节点配置
    laser_mapping_params = [
        PathJoinSubstitution([
            FindPackageShare('point_lio'),
            'config', 'unilidar_l2.yaml'  # 使用新的配置文件
        ]),
        {
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'use_imu_as_input': False,
            'prop_at_freq_of_imu': True,
            'check_satu': True,
            'init_map_size': 10,
            'point_filter_num': 1,
            'space_down_sample': True,
            'filter_size_surf': 0.3,  # 增大滤波尺寸，减少噪声
            'filter_size_map': 0.3,
            'cube_side_length': 1000.0,
            'runtime_pos_log_enable': False,
            # 关键：设置Point-LIO内部使用的坐标系名称
            'odom_header_frame_id': 'camera_init',    # 地图坐标系
            'odom_child_frame_id': 'aft_mapped',      # 机器人坐标系
            # 确保使用转换后的话题
            'lidar_topic': LaunchConfiguration('output_topic'),
            'imu_topic': '/unitree_lidar/imu',
        }
    ]

    # Point-LIO主节点
    laser_mapping_node = Node(
        package='point_lio',
        executable='pointlio_mapping',
        name='laserMapping',
        output='screen',
        parameters=laser_mapping_params,
        remappings=[
            # 添加必要的话题重映射
            ('/point_lio/cloud_registered', LaunchConfiguration('output_topic')),
            ('/unitree_lidar/imu', '/unitree_lidar/imu')
        ]
    )

    # RViz节点
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        arguments=['-d', PathJoinSubstitution([
            FindPackageShare('point_lio'),
            'rviz_cfg', 'loam_livox.rviz'
        ])],
        condition=IfCondition(LaunchConfiguration('rviz')),
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    # TF静态变换发布器（如果需要）
    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='velodyne_to_base_link',
        arguments=['0', '0', '0.2', '0', '0', '0', 'base_link', 'velodyne_link'],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    # 组装Launch描述
    ld = LaunchDescription([
        rviz_arg,
        use_sim_time_arg,
        input_topic_arg,
        output_topic_arg,
        pointcloud_converter,
        laser_mapping_node,
        static_tf_node,
        GroupAction(
            actions=[rviz_node],
            condition=IfCondition(LaunchConfiguration('rviz'))
        ),
    ])

    return ld
