import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
def generate_launch_description():
    
    #LAUNCH PARAMS
    config_use_joystick = LaunchConfiguration('joystick')
    config_use_hardware = LaunchConfiguration('hardware')

    declare_joystick_arg = DeclareLaunchArgument(
            'joystick',
            default_value='true',
            description="Argument used to enable/disable joystick control of the robot.")
    
    declare_hardware_arg = DeclareLaunchArgument(
            'hadrware',
            default_value='true',
            description="Argument used to enable/disable real hardware.")

    #NODES    
    yara_driver_node = Node(
            package='yara_driver',
            executable='yara_driver',
            name='yara_driver_node',
            parameters=[{"joystick": config_use_joystick}, {"hardware":config_use_hardware}],        
            output='screen')
    
    #LAUNCH FILES
    yara_moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [get_package_share_directory('yara_moveit'), '/launch/demo.launch.py']),
        launch_arguments={}.items()
    )

    return LaunchDescription([
        declare_joystick_arg,  
        declare_hardware_arg,
        yara_driver_node,
        yara_moveit_launch,
    ])