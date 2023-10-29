from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():  

    camera_opencv_lf = Node(
        package="line_follower",
        executable="camera_opencv_lf",
    )

    sub_bw_image = Node(
        package="line_follower",
        executable="sub_bw_image",
    )

    pid_control_server = Node(
        package="line_follower",
        executable="pid_control_server",
    )

    follow_line = Node(
        package="line_follower",
        executable="follow_line",
    )

    return LaunchDescription([
        camera_opencv_lf,
        sub_bw_image,
        pid_control_server,
        follow_line,
    ])