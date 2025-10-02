from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Carga el URDF y SRDF como string
    urdf_path = os.path.join(
        get_package_share_directory("openarm_description"),
        "urdf", "robot", "v10.urdf.xacro"
    )
    srdf_path = os.path.join(
        get_package_share_directory("openarm_bimanual_moveit_config"),
        "config", "openarm_bimanual.srdf"
    )
    # Procesa el xacro a urdf pasando el argumento bimanual:=true
    import subprocess
    urdf_xml = subprocess.check_output([
        "ros2", "run", "xacro", "xacro", urdf_path, "bimanual:=true"
    ]).decode()
    with open(srdf_path, "r") as f:
        srdf_xml = f.read()
    # Carga kinematics.yaml como dict
    import yaml
    kinematics_path = os.path.join(
        get_package_share_directory("openarm_bimanual_moveit_config"),
        "config", "kinematics.yaml"
    )
    with open(kinematics_path, "r") as f:
        kinematics_yaml = yaml.safe_load(f)

    return LaunchDescription([
        Node(
            package="openarm_p_p",
            executable="openarm_p_p",
            output="screen",
            parameters=[
                {"robot_description": urdf_xml},
                {"robot_description_semantic": srdf_xml},
                {"use_sim_time": False},
        
            ]
        )
    ])