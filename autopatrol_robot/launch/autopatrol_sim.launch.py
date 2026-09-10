"""启动完整的自动巡检仿真系统。

该文件组合 Gazebo 仿真、Nav2 导航、初始位姿、语音服务和巡检控制器。
仿真启动文件提供机器人、传感器、控制器及里程计 TF；导航启动文件提供
地图服务器、AMCL、规划器、控制器、行为服务器和生命周期管理器。

巡检相关节点按以下顺序启动：

1. 语音服务立即启动，使模型加载与 Gazebo 初始化并行进行；
2. 初始位姿节点延迟八秒启动，并自行等待仿真时间和里程计 TF；
3. 巡检控制器延迟十二秒启动，并自行等待 Nav2 与定位 TF。

定时延迟只用于减少启动阶段的轮询日志，不是就绪条件。
不同计算机上的 Gazebo 启动耗时不同，因此各节点仍必须执行运行时检查。

主要参数分为四组：

- 仿真与定位：use_sim_time、initial_x、initial_y、initial_yaw；
- 地图与导航：map、params_file、arrival_tolerance；
- 巡检与图像：waypoints_file、image_topic、image_save_dir；
- 语音：speech_service、model_dir、audio_player、tts_speed、tts_silence_after。

客户端和服务端共用 speech_service 参数，避免接口名称不一致。
包内资源通过 ament 索引定位，只有大体积语音模型保留可覆盖的源码路径。
"""

import os

import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription, TimerAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """构造并返回完整的巡检启动描述。"""
    # 通过 ament 获取安装路径，避免依赖启动命令所在目录。
    patrol_share = get_package_share_directory('autopatrol_robot')
    slam_share = get_package_share_directory('fishbot_slam')
    nav_share = get_package_share_directory('fishbot_navigation2')
    # 巡检点配置随 autopatrol_robot 一起安装。
    default_waypoints = os.path.join(patrol_share, 'config', 'waypoints.yaml')
    # 地图和 Nav2 参数由导航功能包统一维护。
    default_map = os.path.join(nav_share, 'maps', 'room.yaml')
    default_params_file = os.path.join(
        nav_share, 'config', 'nav2_params.yaml')
    # ONNX 模型体积较大，默认保留源码路径，并允许启动时覆盖。
    default_model = (
        '/home/cds/Code/ros2/dev_ws/src/ros2_rust_tutorials/'
        'autopatrol_robot/tts/vits-melo-tts-zh_en')

    # 所有仿真节点复用同一个时间参数，避免时钟来源不一致。
    use_sim_time = LaunchConfiguration('use_sim_time')
    return launch.LaunchDescription([
        # 仿真时间和初始位姿参数。
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('initial_x', default_value='0.0'),
        DeclareLaunchArgument('initial_y', default_value='0.0'),
        DeclareLaunchArgument('initial_yaw', default_value='0.0'),
        DeclareLaunchArgument('amcl_service', default_value='/amcl/set_initial_pose'),
        DeclareLaunchArgument('initial_pose_topic', default_value='/initialpose'),
        # 巡检任务、地图、导航参数及图像输出路径。
        DeclareLaunchArgument(
            'waypoints_file', default_value=default_waypoints),
        DeclareLaunchArgument('map', default_value=default_map),
        DeclareLaunchArgument('params_file', default_value=default_params_file),
        # 相机与语音接口名称会传递给对应客户端和服务端。
        DeclareLaunchArgument(
            'image_save_dir',
            default_value=os.path.join(os.getcwd(), 'autopatrol_images')),
        DeclareLaunchArgument(
            'image_topic', default_value='/camera_sensor/image_raw'),
        DeclareLaunchArgument(
            'speech_service', default_value='/speech_text'),
        # 应用层到达补偿容差独立于 Nav2 自身的目标容差。
        DeclareLaunchArgument('arrival_tolerance', default_value='0.35'),
        # 语音合成与播放参数。
        DeclareLaunchArgument('audio_player', default_value='auto'),
        DeclareLaunchArgument('tts_speed', default_value='0.75'),
        DeclareLaunchArgument('tts_silence_after', default_value='0.35'),
        DeclareLaunchArgument('model_dir', default_value=default_model),
        # 启动Gazebo仿真机器人、传感器和控制器。
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            os.path.join(slam_share, 'launch', 'gazebo_sim.launch.py'))),
        # 使用指定地图启动 AMCL 和完整 Nav2 导航栈。
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                nav_share, 'launch', 'navigation2.launch.py')),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'map': LaunchConfiguration('map'),
                'params_file': LaunchConfiguration('params_file')}.items()),
        # 初始位姿节点内部仍会等待仿真时钟和里程计 TF。
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
        # 语音模型加载与仿真初始化并行执行。
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
        # 最后启动任务控制器；控制器内部仍会检查 Nav2 和 TF 是否就绪。
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
