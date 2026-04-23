from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    perception_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('handover_bringup'),
                'launch',
                'perception.launch.py'
            ])
        )
    )

    robot_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('handover_bringup'),
                'launch',
                'robot_control.launch.py'
            ])
        )
    )

    return LaunchDescription([
        perception_launch,

        TimerAction(
            period=2.0,
            actions=[robot_control_launch]
        ),
    ])