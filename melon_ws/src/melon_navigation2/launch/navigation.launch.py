from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    package_name = 'melon_navigation2'
    
    start_rviz = LaunchConfiguration('start_rviz')
    use_sim = LaunchConfiguration('use_sim')
    map_yaml_file = LaunchConfiguration('map_yaml_file')
    params_file = LaunchConfiguration('params_file')
    default_bt_xml_filename = LaunchConfiguration('default_bt_xml_filename')
    autostart = LaunchConfiguration('autostart')
    use_composition = LaunchConfiguration('use_composition')
    use_respawn = LaunchConfiguration('use_respawn')

    map_yaml_file = LaunchConfiguration(
        'map_yaml_file',
        default=PathJoinSubstitution(
            [
                FindPackageShare(package_name),
                'map',
                'factory_base_v3.4.yaml'
            ]
        )
    )

    params_file = LaunchConfiguration(
        'params_file',
        default=PathJoinSubstitution(
            [
                FindPackageShare(package_name),
                'params',
                'nav2_paramas.yaml'
            ]
        )
    )

    nav2_launch_file_dir = PathJoinSubstitution(
        [
            FindPackageShare('nav2_bringup'),
            'launch',
        ]
    )

    rviz_config_file = PathJoinSubstitution(
        [
            FindPackageShare(package_name),
            'rviz',
            'navigation2.rviz'
        ]
    )

    default_bt_xml_filename = PathJoinSubstitution(
        [
            FindPackageShare('nav2_bt_navigator'),
            'behavior_trees',
            'navigate_w_replanning_and_recovery.xml'
        ]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'start_rviz',
            default_value='true',
            description='Whether execute rviz2'),

        DeclareLaunchArgument(
            'use_sim',
            default_value='true',
            description='Start robot in Gazebo simulation'),

        DeclareLaunchArgument(
            'map_yaml_file',
            default_value=map_yaml_file,
            description='Full path to map file to load'),

        DeclareLaunchArgument(
            'params_file',
            default_value=params_file,
            description='Full path to the ROS2 parameters file to use for all launched nodes'),

        DeclareLaunchArgument(
            'default_bt_xml_filename',
            default_value=default_bt_xml_filename,
            description='Full path to the behavior tree xml file to use'),

        DeclareLaunchArgument(
            'autostart',
            default_value='true',
            description='Automatically startup the nav2 stack'),

        DeclareLaunchArgument(
            'use_composition',
            default_value='True',
            description='Whether to use composed bringup'),

        DeclareLaunchArgument(
            'use_respawn',
            default_value='false',
            description='Whether to respawn if a node crashes. \
                Applied when composition is disabled.'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([nav2_launch_file_dir, '/bringup_launch.py']),
            launch_arguments={
                'map': map_yaml_file,
                'use_sim_time': use_sim,
                'params_file': params_file,
                'default_bt_xml_filename': default_bt_xml_filename,
                'autostart': autostart,
                'use_composition': use_composition,
                'use_respawn': use_respawn,
            }.items(),
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_file],
            output='screen',
            condition=IfCondition(start_rviz)),
        Node(
                package='pointcloud_to_laserscan', executable='pointcloud_to_laserscan_node',
                remappings=[('cloud_in', ['/point_cloud']),
                            ('scan', ['/scan'])],
                parameters=[{
                    'target_frame': 'base_scan',
                    'transform_tolerance': 0.01,
                    'min_height': -0.4,
                    'max_height': 1.5,
                    'angle_min': -1.5708,  # -M_PI/2
                    'angle_max': 1.5708,  # M_PI/2
                    'angle_increment': 0.0087,  # M_PI/360.0
                    'scan_time': 0.3333,
                    'range_min': 0.05,
                    'range_max': 100.0,
                    'use_inf': True,
                    'inf_epsilon': 1.0,
                    # 'concurrency_level': 1,
                }],
                name='pointcloud_to_laserscan'
            )
    ])