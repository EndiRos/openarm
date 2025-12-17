import os
import xacro

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription, LaunchContext
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def robot_state_publisher_spawner(context: LaunchContext, arm_type, ee_type, bimanual):
    arm_type_str = context.perform_substitution(arm_type)
    ee_type_str = context.perform_substitution(ee_type)
    bimanual_str = context.perform_substitution(bimanual)

    xacro_path = os.path.join(
        get_package_share_directory("openarm_description"),
        "urdf", "robot", f"{arm_type_str}.urdf.xacro"
    )

    robot_description = xacro.process_file(
        xacro_path,
        mappings={
            "arm_type": arm_type_str,
            "ee_type": ee_type_str,
            "bimanual": bimanual_str,
        }
    ).toprettyxml(indent="  ")

    return [
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[{"robot_description": robot_description}],
        )
    ]


def generate_launch_description():
    arm_type_arg = DeclareLaunchArgument(
        "arm_type",
        default_value="v10",
        description="Type of arm (xacro filename without extension)"
    )

    ee_type_arg = DeclareLaunchArgument(
        "ee_type",
        default_value="openarm_hand",
        description="End-effector type"
    )

    bimanual_arg = DeclareLaunchArgument(
        "bimanual",
        default_value="true",
        description="Whether to use bimanual configuration"
    )

    arm_type = LaunchConfiguration("arm_type")
    ee_type = LaunchConfiguration("ee_type")
    bimanual = LaunchConfiguration("bimanual")

    robot_state_publisher_loader = OpaqueFunction(
        function=robot_state_publisher_spawner,
        args=[arm_type, ee_type, bimanual],
    )

    return LaunchDescription([
        arm_type_arg,
        ee_type_arg,
        bimanual_arg,
        robot_state_publisher_loader,
    ])