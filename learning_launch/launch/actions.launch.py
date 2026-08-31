import launch_ros
import launch
import launch.launch_description_sources
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    action_declare_startup_rqt = launch.actions.DeclareLaunchArgument(
        name="startup_rqt",
        default_value="true",
        description="Start RQT on launch"
    )

    startup_rqt = launch.substitutions.LaunchConfiguration('startup_rqt')

    multisim_launch_path = [get_package_share_directory('turtlesim'), '/launch/', 'multisim.launch.py']
    action_include_launch = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            multisim_launch_path
        )
    )

    action_log_info = launch.actions.LogInfo(
        msg=str(multisim_launch_path)
    )

    action_topic_list = launch.actions.ExecuteProcess(
        cmd=['ros2', 'topic', 'list'],
        output='screen'
    )

    rqt_condition = launch.conditions.IfCondition(startup_rqt)

    action_rqt = launch.actions.ExecuteProcess(
        cmd=['rqt'],
        condition=rqt_condition,
        output='screen'
    )

    action_group = launch.actions.GroupAction([
        launch.actions.TimerAction(
            period=2.0,
            actions=[
                action_include_launch
            ]
        ),
        launch.actions.TimerAction(
            period=4.0,
            actions=[
                action_topic_list
            ]
        ),
        launch.actions.TimerAction(
            period=6.0,
            actions=[
                action_rqt
            ]
        ),
    ])

    return launch.LaunchDescription([
        action_declare_startup_rqt,
        action_log_info,
        action_group
    ])
    