import launch
import launch_ros
import os
from ament_index_python.packages import get_package_share_directory
import launch_ros.parameter_descriptions
from launch.event_handlers import OnProcessExit
from launch.actions import RegisterEventHandler, ExecuteProcess

def generate_launch_description():
    # 获取功能包的路径
    urdf_package_path = get_package_share_directory('pioneerbot_description')

    default_xacro_path = os.path.join(urdf_package_path, 'urdf', 'pioneer', 'pioneer_robot.xacro')
    default_gazebo_world_path = os.path.join(urdf_package_path, 'world', 'custom_room.world')
    
    # 声明一个urdf目录的参数，方便修改
    action_declare_arg_mode_path = launch.actions.DeclareLaunchArgument(
        name='model', 
        default_value=str(default_xacro_path), 
        description='加载的模型文件路径'
    )
    
    # 修复：正确的 Command 使用方式
    command_result = launch.substitutions.Command([
        'xacro ', 
        launch.substitutions.LaunchConfiguration('model')
    ])
    
    description_value = launch_ros.parameter_descriptions.ParameterValue(
        command_result, 
        value_type=str
    )
    
    # 机器人状态发布器
    action_robot_state_publisher = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': description_value}]
    )

    # Gazebo 启动
    action_launch_gazebo = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            [os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]
        ),
        launch_arguments=[('world', default_gazebo_world_path), ('verbose', 'true')]
    )
    
    # 在Gazebo中生成机器人
    action_spawn_entity = launch_ros.actions.Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', '/robot_description',
        '-entity', 'pioneerbot',
        ],
        output='screen'
    )

    return launch.LaunchDescription([
        action_declare_arg_mode_path,
        action_robot_state_publisher,
        action_launch_gazebo,
        action_spawn_entity,
    ])