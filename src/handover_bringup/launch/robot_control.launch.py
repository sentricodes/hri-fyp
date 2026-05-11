from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    robot_ip = LaunchConfiguration('robot_ip')

    robot_ip_arg = DeclareLaunchArgument(
        'robot_ip',
        default_value='192.168.77.2',
        description='IP address of the UR robot or simulator'
    )

    custom_description_file = PathJoinSubstitution([
        FindPackageShare('fyp_ur_description'),
        'urdf',
        'ur5e_c403.urdf.xacro'
    ])

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
            'robot_ip': robot_ip,
            # 'description_package': 'fyp_ur_description',
            # 'description_file': custom_description_file,
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
            # 'description_package': 'fyp_ur_description',
            # 'description_file': custom_description_file,
            # 'moveit_config_package': 'fyp_ur_description',
            # 'moveit_config_file': 'ur_c403.srdf.xacro',
            'launch_rviz': 'true',
        }.items()
    )

    rg2_gripper = Node(
        package="onrobot_rg2",
        executable="onrobot_rg2_node",
        name="onrobot_rg2_node",
        output="screen",
    )

    # moveit_executor_node = Node(
    #     package='handover_control',
    #     executable="moveit_executor_node",
    #     name='moveit_executor_node',
    #     output='screen'
    # )

    moveit_motion_server_node = Node(
        package='handover_control',
        executable="moveit_motion_server_node",
        # name='moveit_motion_server_node',
        output='screen'
    )

    handover_state_node = Node(
    package="handover_control",
    executable="handover_state_node",
    name="handover_state_node",
    output="screen",
    )

    return LaunchDescription([
        robot_ip_arg,

        ur_control,

        TimerAction(
            period=5.0,
            actions=[ur_moveit]
        ),

        TimerAction(
            period=6.0,
            actions=[rg2_gripper],
        ),

        # TimerAction(
        #     period=10.0,
        #     actions=[moveit_executor_node]
        # ),

        TimerAction(
            period=10.0,
            actions=[moveit_motion_server_node]
        ),

        TimerAction(
            period=11.0,
            actions=[handover_state_node],
        ),
    ])