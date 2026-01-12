from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os, subprocess, yaml

def generate_launch_description():
    # Rutas de URDF/SRDF
    urdf_path = os.path.join(
        get_package_share_directory("openarm_description"),
        "urdf", "robot", "v10.urdf.xacro"
    )
    srdf_path = os.path.join(
        get_package_share_directory("openarm_bimanual_moveit_config"),
        "config", "openarm_bimanual.srdf"
    )
    # Generar URDF desde xacro con bimanual:=true
    urdf_xml = subprocess.check_output([
        "ros2", "run", "xacro", "xacro", urdf_path, "bimanual:=true"
    ]).decode()
    with open(srdf_path, "r") as f:
        srdf_xml = f.read()

    # Cargar cinemática
    kinematics_path = os.path.join(
        get_package_share_directory("openarm_bimanual_moveit_config"),
        "config", "kinematics.yaml"
    )
    with open(kinematics_path, "r") as f:
        kinematics_yaml = yaml.safe_load(f)

    # Incluir move_group de MoveIt
    """ move_group_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("openarm_bimanual_moveit_config"),
                "launch", "move_group.launch.py"
            )
        )
        # Si el launch acepta argumentos para descriptions, se pueden pasar aquí con launch_arguments=...
    ) """

    # Tu aplicación con los mismos parámetros
    app_node = Node(
        package="openarm_application",
        executable="main",
        output="screen",
        parameters=[
            {"robot_description": urdf_xml},
            {"robot_description_semantic": srdf_xml},
            kinematics_yaml,
            {"use_sim_time": True},
        ],
    )

    return LaunchDescription([app_node])