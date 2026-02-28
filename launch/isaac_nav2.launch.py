import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('isaac_sim_nav2')
    
    # Config files
    nav2_params = os.path.join(pkg_share, 'config', 'nav2_params_isaac.yaml')
    rviz_config = os.path.join(pkg_share, 'config', 'rviz', 'nav2_isaac_view.rviz')
    
    # Arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    map_yaml_file = LaunchConfiguration('map', default=os.path.join(pkg_share, 'maps', 'warehouse.yaml'))
    autostart = LaunchConfiguration('autostart', default='true')
    use_navigator = LaunchConfiguration('use_navigator', default='false')
    
    # Nav2 bringup
    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'bringup_launch.py')
        ]),
        launch_arguments={
            'params_file': nav2_params,
            'use_sim_time': use_sim_time,
            'map': map_yaml_file,
            'autostart': autostart
        }.items()
    )
    
    # Multi-goal navigator (optional, delayed start)
    goal_file = os.path.join(pkg_share, 'tests', 'test_goals.yaml')
    navigator_node = Node(
        package='isaac_sim_nav2',
        executable='multi_goal_nav',
        name='multi_goal_navigator',
        output='screen',
        parameters=[{
            'goal_file': goal_file,
            'iteration_count': 1,
            'record_bag': False,
            'use_sim_time': use_sim_time
        }],
        condition=IfCondition(use_navigator)
    )
    
    # Delayed navigator start to allow Nav2 to initialize
    delayed_navigator = TimerAction(
        period=10.0,
        actions=[navigator_node]
    )
    
    # RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )
    
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true', description='Use simulation time'),
        DeclareLaunchArgument('map', default_value=map_yaml_file, description='Full path to map yaml file'),
        DeclareLaunchArgument('autostart', default_value='true', description='Automatically startup the nav2 stack'),
        DeclareLaunchArgument('use_navigator', default_value='false', description='Launch multi-goal navigator'),
        nav2_bringup,
        rviz_node,
        delayed_navigator
    ])


from launch.conditions import IfCondition
