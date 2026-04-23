from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='vzense_camera',
            executable='DCAM710_node',
            name='vzense_camera',
            output='screen'
        ),

        TimerAction(
            period=2.0,
            actions=[
                Node(
                    package='handover_perception',
                    executable='mediapipe_hand_node',
                    name='mediapipe_hand_node',
                    output='screen'
                )
            ]
        ),

        TimerAction(
            period=3.0,
            actions=[
                Node(
                    package='handover_perception',
                    executable='handover_zone_node',
                    name='handover_zone_node',
                    output='screen'
                )
            ]
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='camera_static_tf',
            arguments=[
                '0.00', '-0.30', '0.13',
                '-1.5708', '3.14159', '1.5708',
                'base_link', 'vzense_rgb_frame'
            ],
            output='screen'
        ),

        TimerAction(
            period=4.0,
            actions=[
                Node(
                    package='handover_perception',
                    executable='palm_transform_node',
                    name='palm_transform_node',
                    output='screen'
                )
            ]
        ),

        TimerAction(
            period=5.0,
            actions=[
                Node(
                    package='handover_perception',
                    executable='approach_pose_node',
                    name='approach_pose_node',
                    output='screen'
                )
            ]
        ),
    ])