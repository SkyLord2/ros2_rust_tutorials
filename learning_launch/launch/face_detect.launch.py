import launch
import launch_ros

def generate_launch_description():

    launch_model_path = launch.actions.DeclareLaunchArgument(
        name="launch_model_path",
        default_value="/home/cds/Code/ros2/dev_ws/src/ros2_rust_tutorials/face_detection_yunet_2022mar.onnx",
        description="Path to YuNet ONNX model file"
    )
    face_detect_client = launch_ros.actions.Node(
        package="face_detect_cpp",
        executable="face_detect_client",
        output='screen'
    )

    face_detect_server = launch_ros.actions.Node(
        package="face_detect_cpp",
        executable="face_detect_server",
        parameters=[{'model_path': launch.substitutions.LaunchConfiguration('launch_model_path')}],
        output='screen'
    )

    return launch.LaunchDescription([
        launch_model_path,
        face_detect_client,
        face_detect_server
    ])