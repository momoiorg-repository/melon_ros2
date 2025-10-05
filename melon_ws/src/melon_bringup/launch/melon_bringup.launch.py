#!/usr/bin/env python3
"""
MoveItとNavigation2を同時に起動するlaunchファイル
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    
    # Launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use simulation clock if true",
    )
    
    start_rviz_arg = DeclareLaunchArgument(
        "start_rviz", 
        default_value="true",
        description="Whether to start RViz"
    )
    
    start_moveit_arg = DeclareLaunchArgument(
        "start_moveit",
        default_value="true", 
        description="Whether to start MoveIt"
    )
    
    start_navigation_arg = DeclareLaunchArgument(
        "start_navigation",
        default_value="true",
        description="Whether to start Navigation2"
    )
    
    ros2_control_hardware_type_arg = DeclareLaunchArgument(
        "ros2_control_hardware_type",
        default_value="isaac",
        description="ROS2 control hardware interface type to use for the launch file -- possible values: [mock_components, isaac]",
    )
    
    map_yaml_file_arg = DeclareLaunchArgument(
        "map_yaml_file",
        default_value=PathJoinSubstitution([
            FindPackageShare("melon_navigation2"),
            "map",
            "factory_base_v3.4.yaml"
        ]),
        description="Full path to map file to load"
    )
    
    params_file_arg = DeclareLaunchArgument(
        "params_file", 
        default_value=PathJoinSubstitution([
            FindPackageShare("melon_navigation2"),
            "params",
            "nav2_paramas.yaml"
        ]),
        description="Full path to the ROS2 parameters file to use for all launched nodes"
    )

    # Launch configurations
    use_sim_time = LaunchConfiguration("use_sim_time")
    start_rviz = LaunchConfiguration("start_rviz")
    start_moveit = LaunchConfiguration("start_moveit")
    start_navigation = LaunchConfiguration("start_navigation")
    ros2_control_hardware_type = LaunchConfiguration("ros2_control_hardware_type")
    map_yaml_file = LaunchConfiguration("map_yaml_file")
    params_file = LaunchConfiguration("params_file")

    # MoveIt launch
    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("melon_moveit_config"),
                "launch",
                "melon_moveit.launch.py"
            ])
        ]),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "ros2_control_hardware_type": ros2_control_hardware_type,
        }.items(),
        condition=IfCondition(start_moveit)
    )

    # Navigation2 launch
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("melon_navigation2"),
                "launch", 
                "navigation.launch.py"
            ])
        ]),
        launch_arguments={
            "use_sim": use_sim_time,
            "map_yaml_file": map_yaml_file,
            "params_file": params_file,
            "start_rviz": "true", 
            "autostart": "true",
            "use_composition": "True",
            "use_respawn": "false",
        }.items(),
        condition=IfCondition(start_navigation)
    )


    return LaunchDescription([
        # Launch arguments
        use_sim_time_arg,
        start_rviz_arg,
        start_moveit_arg,
        start_navigation_arg,
        ros2_control_hardware_type_arg,
        map_yaml_file_arg,
        params_file_arg,
        
        # MoveIt group
        GroupAction([
            moveit_launch
        ]),
        
        # Navigation group  
        GroupAction([
            navigation_launch
        ]),
    ])