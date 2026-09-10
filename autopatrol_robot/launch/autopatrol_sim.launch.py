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
    default_map = os.path.join(nav_share, 'maps', 'room.yaml')
    default_params_file = os.path.join(
        nav_share, 'config', 'nav2_params.yaml')
    default_model = (
        '/home/cds/Code/ros2/dev_ws/src/ros2_rust_tutorials/'
        'autopatrol_robot/tts/vits-melo-tts-zh_en')

    use_sim_time = LaunchConfiguration('use_sim_time')
    return launch.LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('initial_x', default_value='0.0'),
        DeclareLaunchArgument('initial_y', default_value='0.0'),
        DeclareLaunchArgument('initial_yaw', default_value='0.0'),
        DeclareLaunchArgument('amcl_service', default_value='/amcl/set_initial_pose'),
        DeclareLaunchArgument('initial_pose_topic', default_value='/initialpose'),
        DeclareLaunchArgument(
            'waypoints_file', default_value=default_waypoints),
        DeclareLaunchArgument('map', default_value=default_map),
        DeclareLaunchArgument('params_file', default_value=default_params_file),
        DeclareLaunchArgument(
            'image_save_dir',
            default_value=os.path.join(os.getcwd(), 'autopatrol_images')),
        DeclareLaunchArgument(
            'image_topic', default_value='/camera_sensor/image_raw'),
        DeclareLaunchArgument(
            'speech_service', default_value='/speech_text'),
        DeclareLaunchArgument('arrival_tolerance', default_value='0.35'),
        DeclareLaunchArgument('audio_player', default_value='auto'),
        DeclareLaunchArgument('tts_speed', default_value='0.75'),
        DeclareLaunchArgument('tts_silence_after', default_value='0.35'),
        DeclareLaunchArgument('model_dir', default_value=default_model),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            os.path.join(slam_share, 'launch', 'gazebo_sim.launch.py'))),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                nav_share, 'launch', 'navigation2.launch.py')),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'map': LaunchConfiguration('map'),
                'params_file': LaunchConfiguration('params_file')}.items()),
        TimerAction(period=8.0, actions=[launch_ros.actions.Node(
            package='fishbot_application_cpp',
            executable='init_robot_pose', output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'initial_x': LaunchConfiguration('initial_x'),
                'initial_y': LaunchConfiguration('initial_y'),
                'initial_yaw': LaunchConfiguration('initial_yaw'),
                'amcl_service': LaunchConfiguration('amcl_service'),
                'initial_pose_topic': LaunchConfiguration(
                    'initial_pose_topic')}])]),
        launch_ros.actions.Node(
            package='autopatrol_robot', executable='tts_server.py',
            output='screen',
            parameters=[{
                'model_dir': LaunchConfiguration('model_dir'),
                'service_name': LaunchConfiguration('speech_service'),
                'audio_player': LaunchConfiguration('audio_player'),
                'speed': LaunchConfiguration('tts_speed'),
                'silence_after': LaunchConfiguration(
                    'tts_silence_after')}]),
        TimerAction(period=12.0, actions=[launch_ros.actions.Node(
            package='autopatrol_robot', executable='patrol_controller',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'waypoints_file': LaunchConfiguration('waypoints_file'),
                'image_topic': LaunchConfiguration('image_topic'),
                'speech_service': LaunchConfiguration('speech_service'),
                'image_save_dir': LaunchConfiguration('image_save_dir'),
                'arrival_tolerance': LaunchConfiguration(
                    'arrival_tolerance')}])]),
    ])
