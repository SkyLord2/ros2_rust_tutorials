import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    urdf_pkg_path = get_package_share_directory('fishbot_description')
    default_urdf_path = os.path.join(urdf_pkg_path, 'urdf', 'fishbot', 'fishbot.urdf.xacro')
    # default_rviz_path = os.path.join(urdf_pkg_path, 'config', 'display_robot_model.rviz')

    default_gazebo_world_path = os.path.join(urdf_pkg_path, 'world', 'custom_room.world')

    action_declare_arg_mode_path = launch.actions.DeclareLaunchArgument(
        name='model',
        default_value=default_urdf_path,
        description='Path to robot urdf file'
    )

    substitutions_command_result = launch.substitutions.Command(['xacro ', launch.substitutions.LaunchConfiguration('model')])

    robot_description_value = launch_ros.parameter_descriptions.ParameterValue(
        substitutions_command_result,
        value_type=str
    )

    action_robot_state_publisher = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_value}]
    )

    # action_joint_state_publisher = launch_ros.actions.Node(
    #     package='joint_state_publisher',
    #     executable='joint_state_publisher',
    #     output='screen',
    # )

    # action_rviz_node = launch_ros.actions.Node(
    #     package='rviz2',
    #     executable='rviz2',
    #     arguments=['-d', default_rviz_path],
    #     output='screen',
    # )

    action_launch_gazebo = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            [get_package_share_directory('gazebo_ros'), '/launch', '/gazebo.launch.py']
        ),
        launch_arguments=[('world', default_gazebo_world_path), ('verbose', 'true')]

    )

    action_spawn_entity = launch_ros.actions.Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'fishbot'],
        output='screen'
    )

    # 加载并激活 fishbot_joint_state_broadcaster 控制器
    load_joint_state_controller = launch.actions.ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
            'fishbot_joint_state_broadcaster'],
        output='screen'
    )

    # 加载并激活 fishbot_effort_controller 控制器
    load_fishbot_effort_controller = launch.actions.ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active','fishbot_effort_controller'], 
        output='screen')

    return launch.LaunchDescription([
        action_declare_arg_mode_path,
        action_robot_state_publisher,
        # action_joint_state_publisher,
        # action_rviz_node,
        action_launch_gazebo,
        action_spawn_entity,
        # 事件动作，当加载机器人结束后执行    
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=action_spawn_entity,
                on_exit=[load_joint_state_controller],)
        ),
        # 事件动作，load_fishbot_diff_drive_controller
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
            target_action=load_joint_state_controller,
            on_exit=[load_fishbot_effort_controller],)
        ),
    ])