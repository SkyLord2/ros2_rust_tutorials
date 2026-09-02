import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    urdf_pkg_path = get_package_share_directory('fishbot_description')
    default_urdf_path = os.path.join(urdf_pkg_path, 'urdf', 'first_robot.urdf')
    default_rviz_path = os.path.join(urdf_pkg_path, 'config', 'display_robot_model.rviz')

    action_declare_arg_mode_path = launch.actions.DeclareLaunchArgument(
        name='mode',
        default_value=default_urdf_path,
        description='Path to robot urdf file'
    )

    substitutions_command_result = launch.substitutions.Command(['cat ', launch.substitutions.LaunchConfiguration('mode')])

    robot_description_value = launch_ros.parameter_descriptions.ParameterValue(
        substitutions_command_result,
        value_type=str
    )

    action_robot_state_publisher = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description_value}]
    )

    aaction_joint_state_publisher = launch_ros.actions.Node(
        package='joint_state_publisher',
        executable='joint_state_publisher'
    )

    action_rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', default_rviz_path]
    )

    return launch.LaunchDescription([
        action_declare_arg_mode_path,
        action_robot_state_publisher,
        aaction_joint_state_publisher,
        action_rviz_node
    ])