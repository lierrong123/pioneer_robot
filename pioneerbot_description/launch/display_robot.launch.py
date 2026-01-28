import launch
import launch_ros
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 获取默认的urdf路径
    urdf_package_path = get_package_share_directory('pioneerbot_description')
    default_urdf_path = os.path.join(urdf_package_path, 'urdf', 'pioneer_robot.urdf')
    default_rviz_config_path = os.path.join(urdf_package_path,'config','display_robot_model.rviz')
    # 声明一个urdf目录的参数，方便修改
    action_declare_arg_mode_path = launch.actions.DeclareLaunchArgument(
        name='model', 
        default_value=str(default_urdf_path), 
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
    
    # 修复：exec_name 改为 executable
    action_robot_state_publisher = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',  # 修复：exec_name -> executable
        parameters=[{'robot_description': description_value}]
    )

    action_joint_state_publisher = launch_ros.actions.Node(
        package='joint_state_publisher',
        executable='joint_state_publisher'  # 修复：exec_name -> executable
    )

    action_rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',  # 修复：exec_name -> executable
        arguments= ['-d',default_rviz_config_path]
    )

    return launch.LaunchDescription([
        action_declare_arg_mode_path,
        action_robot_state_publisher,
        action_joint_state_publisher,
        action_rviz_node
    ])