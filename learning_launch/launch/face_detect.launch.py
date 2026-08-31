import launch
import launch_ros

def generate_launch_description():

    face_detect_client = launch_ros.actions.Node(
        package="face_detect_cpp",
        executable="face_detect_client"
    )

    face_detect_server = launch_ros.actions.Node(
        package="face_detect_cpp",
        executable="face_detect_server"
    )

    return launch.LaunchDescription([
        face_detect_client,
        face_detect_server
    ])