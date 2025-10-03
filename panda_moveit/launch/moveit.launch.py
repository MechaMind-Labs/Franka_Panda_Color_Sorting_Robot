import os
from launch import LaunchDescription
from moveit_configs_utils import MoveItConfigsBuilder
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import PathJoinSubstitution, Command, FindExecutable


def generate_launch_description():

    # Arguments
    is_sim = LaunchConfiguration("is_sim")
    is_ignition = LaunchConfiguration("is_ignition")

    is_sim_arg = DeclareLaunchArgument(
        "is_sim", default_value="true",
        description="Use simulation time if true"
    )

    is_ignition_arg = DeclareLaunchArgument(
        "is_ignition", default_value="true",
        description="Use Ignition Gazebo if true"
    )

    # Build MoveIt config
    moveit_config = (
        MoveItConfigsBuilder("panda", package_name="panda_moveit")
        .robot_description(
            file_path=os.path.join(
                get_package_share_directory("panda_description"),
                "urdf",
                "panda.urdf.xacro"
            ),
            mappings={
                "is_ignition": is_ignition
            }
        )
        .robot_description_semantic(file_path="config/panda.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .to_moveit_configs()
    )

    # Move Group Node
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {"use_sim_time": is_sim},
            {"publish_robot_description_semantic": True}
        ],
        arguments=["--ros-args", "--log-level", "info"],
    )

    # RViz
    rviz_config = os.path.join(
        get_package_share_directory("panda_moveit"),
        "rviz",
        "moveit.rviz",
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
        ],
    )

    return LaunchDescription([
        is_sim_arg,
        is_ignition_arg,
        move_group_node,
        rviz_node
    ])
