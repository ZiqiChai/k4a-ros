from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    params_file = LaunchConfiguration("params_file")
    namespace = LaunchConfiguration("namespace")
    use_rviz = LaunchConfiguration("use_rviz")
    package_share = FindPackageShare("k4a_ros")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=PathJoinSubstitution([package_share, "config", "k4a.yaml"]),
                description="ROS 2 parameter file for the Azure Kinect driver",
            ),
            DeclareLaunchArgument("namespace", default_value="k4a"),
            DeclareLaunchArgument("use_rviz", default_value="false"),
            Node(
                package="k4a_ros",
                executable="k4a_driver_node",
                name="driver",
                namespace=namespace,
                parameters=[params_file],
                output="screen",
                emulate_tty=True,
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="k4a_rviz",
                arguments=[
                    "-d",
                    PathJoinSubstitution([package_share, "rviz", "k4a_driver.rviz"]),
                ],
                condition=IfCondition(use_rviz),
                output="screen",
            ),
        ]
    )
