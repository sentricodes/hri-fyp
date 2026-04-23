from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    ur_control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('ur_robot_driver'),
                'launch',
                'ur_control.launch.py'
            ])
        ),
        launch_arguments={
            'ur_type': 'ur5e',
            'robot_ip': '192.168.77.2',
            'launch_rviz': 'false',
        }.items()
    )

    ur_moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('ur_moveit_config'),
                'launch',
                'ur_moveit.launch.py'
            ])
        ),
        launch_arguments={
            'ur_type': 'ur5e',
            'launch_rviz': 'true',
        }.items()
    )

    moveit_executor = Node(
        package='handover_control',
        executable='moveit_executor_node',
        name='moveit_executor_node',
        output='screen'
    )

    return LaunchDescription([
        ur_control,

        TimerAction(
            period=5.0,
            actions=[ur_moveit]
        ),

        TimerAction(
            period=10.0,
            actions=[moveit_executor]
        ),
    ])