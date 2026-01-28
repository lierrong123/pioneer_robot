import os
from pathlib import Path
from launch import LaunchDescription
import launch_ros.actions
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    # 更新参数名称和添加新参数
    axis_linear = LaunchConfiguration('axis_linear', default='1')
    axis_strafe = LaunchConfiguration('axis_strafe', default='0')  # 新增横向移动轴
    axis_angular = LaunchConfiguration('axis_angular', default='2')  # 改为右摇杆
    v_linear = LaunchConfiguration('v_linear', default='0.8')  # 参数名更新
    v_angular = LaunchConfiguration('v_angular', default='1.0')  # 参数名更新
      
    return LaunchDescription([
        launch_ros.actions.Node(
            package='joy', 
            executable='joy_node',
            name='joy_node', 
            output="screen",),

        launch_ros.actions.Node(
            package='wheeltec_joy', 
            executable='wheeltec_joy',
            name='wheeltec_joy',  
            parameters=[{
                'axis_linear': axis_linear, 
                'axis_strafe': axis_strafe,  # 新增参数
                'axis_angular': axis_angular, 
                'v_linear': v_linear,  # 参数名更新
                'v_angular': v_angular  # 参数名更新
            }],
            output="screen",)
    ])
