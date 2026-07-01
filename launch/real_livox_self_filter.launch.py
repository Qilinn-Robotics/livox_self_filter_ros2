from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    self_filter = Node(
        package="livox_self_filter_ros2",
        executable="livox_custom_msg_filter",
        name="livox_custom_msg_filter",
        output="screen",
        parameters=[
            LaunchConfiguration("filter_params_file"),
            {
                "publish_debug_clouds": ParameterValue(
                    LaunchConfiguration("publish_debug_clouds"),
                    value_type=bool,
                ),
            },
        ],
    )

    lidar_static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="base_to_livox_mid360_static_tf",
        output="screen",
        arguments=[
            "--x",
            LaunchConfiguration("lidar_x"),
            "--y",
            LaunchConfiguration("lidar_y"),
            "--z",
            LaunchConfiguration("lidar_z"),
            "--roll",
            LaunchConfiguration("lidar_roll"),
            "--pitch",
            LaunchConfiguration("lidar_pitch"),
            "--yaw",
            LaunchConfiguration("lidar_yaw"),
            "--frame-id",
            LaunchConfiguration("base_frame"),
            "--child-frame-id",
            LaunchConfiguration("lidar_frame"),
        ],
        condition=IfCondition(LaunchConfiguration("publish_lidar_static_tf")),
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", LaunchConfiguration("rviz_config_file")],
        condition=IfCondition(LaunchConfiguration("use_rviz")),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "filter_params_file",
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare("livox_self_filter_ros2"),
                        "config",
                        "real_livox_self_filter.yaml",
                    ]
                ),
            ),
            DeclareLaunchArgument("publish_debug_clouds", default_value="false"),
            DeclareLaunchArgument("use_rviz", default_value="false"),
            DeclareLaunchArgument(
                "rviz_config_file",
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare("livox_self_filter_ros2"),
                        "rviz",
                        "real_livox_self_filter.rviz",
                    ]
                ),
            ),
            DeclareLaunchArgument("publish_lidar_static_tf", default_value="true"),
            DeclareLaunchArgument("base_frame", default_value="base_link"),
            DeclareLaunchArgument("lidar_frame", default_value="livox_mid360_link"),
            DeclareLaunchArgument("lidar_x", default_value="0.21"),
            DeclareLaunchArgument("lidar_y", default_value="0.0"),
            DeclareLaunchArgument("lidar_z", default_value="0.13"),
            DeclareLaunchArgument("lidar_roll", default_value="0.0"),
            DeclareLaunchArgument("lidar_pitch", default_value="0.0"),
            DeclareLaunchArgument("lidar_yaw", default_value="1.5708"),
            lidar_static_tf,
            self_filter,
            rviz,
        ]
    )
