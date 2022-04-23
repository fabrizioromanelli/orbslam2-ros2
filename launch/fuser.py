from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Set arguments from command line
    camera_pitch_launch_arg = DeclareLaunchArgument(
        "camera_pitch",
        description='Camera pitch angle in radians'
    )

    # Node to start
    fuser_node = Node(
        package="fuser",
        executable="fuser",
        output='both',
        emulate_tty=True,
        arguments=[
                "/usr/local/share/ORB_SLAM2/Vocabulary/orb_mur.fbow",
                "/usr/local/share/ORB_SLAM2/Config/RealSense-D435i-IRD.yaml",
                LaunchConfiguration("camera_pitch")
        ]
    )

    return LaunchDescription([
        camera_pitch_launch_arg,
        fuser_node
    ])
