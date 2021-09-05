from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Set arguments from command line
    start_pad_launch_arg = DeclareLaunchArgument(
        "start_pad",
        description='Starting pad ID',
        choices=["1",
                 "2",
                 "3",
                 "4",
                 "5",
                 "6",
                 "7",
                 "8",
                 "9"]
    )

    # Node to start
    orbslam2_node = Node(
        package="orbslam2",
        executable="orbslam2",
        arguments=[
                "/usr/local/share/ORB_SLAM2/Vocabulary/orb_mur.fbow",
                "/usr/local/share/ORB_SLAM2/Config/RealSense-D435i-IRD.yaml",
                "IRD",
                "OFF",
                LaunchConfiguration("start_pad")
        ]
    )

    return LaunchDescription([
        start_pad_launch_arg,
        orbslam2_node
    ])
