import os

import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription, TimerAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    patrol_share = get_package_share_directory('autopatrol_robot')
    slam_share = get_package_share_directory('fishbot_slam')
    nav_share = get_package_share_directory('fishbot_navigation2')
    default_waypoints = os.path.join(patrol_share, 'config', 'waypoints.yaml')
    default_model = (
        '/home/cds/Code/ros2/dev_ws/src/ros2_rust_tutorials/'
        'autopatrol_robot/tts/vits-melo-tts-zh_en')

    use_sim_time = LaunchConfiguration('use_sim_time')
    return launch.LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument(
            'waypoints_file', default_value=default_waypoints),
        DeclareLaunchArgument(
            'image_save_dir',
            default_value=os.path.join(os.getcwd(), 'autopatrol_images')),
        DeclareLaunchArgument('model_dir', default_value=default_model),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            os.path.join(slam_share, 'launch', 'gazebo_sim.launch.py'))),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                nav_share, 'launch', 'navigation2.launch.py')),
            launch_arguments={'use_sim_time': use_sim_time}.items()),
        TimerAction(period=8.0, actions=[launch_ros.actions.Node(
            package='fishbot_application_cpp',
            executable='init_robot_pose', output='screen')]),
        launch_ros.actions.Node(
            package='autopatrol_robot', executable='tts_server.py',
            output='screen',
            parameters=[{
                'model_dir': LaunchConfiguration('model_dir')}]),
        TimerAction(period=12.0, actions=[launch_ros.actions.Node(
            package='autopatrol_robot', executable='patrol_controller',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'waypoints_file': LaunchConfiguration('waypoints_file'),
                'image_save_dir': LaunchConfiguration('image_save_dir')}])]),
    ])
