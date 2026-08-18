from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 启动 turtlesim 节点
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='sim'
        ),
        # 启动第一个广播器，设置参数 turtlename 为 turtle1
        Node(
            package='learning_tf',
            executable='turtle_tf_broadcaster',
            name='broadcaster1',
            parameters=[
                {'turtlename': 'turtle1'}
            ]
        ),
        # 声明 Launch 参数 target_frame，默认值为 turtle1
        DeclareLaunchArgument(
            'target_frame', default_value='turtle1',
            description='Target frame name.'
        ),
        # 启动第二个广播器，设置参数 turtlename 为 turtle2
        Node(
            package='learning_tf',
            executable='turtle_tf_broadcaster',
            name='broadcaster2',
            parameters=[
                {'turtlename': 'turtle2'}
            ]
        ),
        # 启动监听跟随节点，并将外部 LaunchConfiguration 参数传入
        Node(
            package='learning_tf',
            executable='turtle_following',
            name='listener',
            parameters=[
                {'target_frame': LaunchConfiguration('target_frame')}
            ]
        ), 
    ])