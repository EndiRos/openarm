from launch import LaunchDescription, LaunchContext
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def print_values(context: LaunchContext, *args):
    """Función que imprime los valores de los argumentos y la ruta al xacro."""
    print("\n" + "="*50)
    print("VALORES DE LOS ARGUMENTOS DE LAUNCH:")
    print("="*50)
    
    # Recuperar y mostrar cada argumento
    for arg in args:
        arg_name = str(arg).split("'")[1] if "'" in str(arg) else str(arg)
        arg_value = context.perform_substitution(arg)
        print(f"{arg_name}: {arg_value}")

    # Obtener la ruta al xacro usando FindPackageShare y PathJoinSubstitution
    description_package = args[0]  # Suponiendo que el primer argumento es el nombre del paquete
    xacro_path_subst = PathJoinSubstitution([
        FindPackageShare(description_package),
        "urdf",
        "robot",
        "v10.urdf.xacro"
    ])

    
    xacro_path = context.perform_substitution(xacro_path_subst)
    print(f"Ruta completa al openarm.xacro: {xacro_path}")

    print("="*50 + "\n")
    return []


def generate_launch_description():
    # Declarar argumentos
    declared_arguments = [
        DeclareLaunchArgument(
            "description_package",
            default_value="openarm_description",
            description="Paquete donde está el modelo"
        ),
        DeclareLaunchArgument(
            "robot_name",
            default_value="openarm_robot",
            description="Nombre del robot"
        ),
        DeclareLaunchArgument(
            "use_sim",
            default_value="true",
            description="Usar simulación",
            choices=["true", "false"]
        ),
        DeclareLaunchArgument(
            "namespace",
            default_value="/robot",
            description="Namespace del robot"
        ),
    ]

    # Recuperar configuraciones
    description_package = LaunchConfiguration("description_package")
    robot_name = LaunchConfiguration("robot_name")
    use_sim = LaunchConfiguration("use_sim")
    namespace = LaunchConfiguration("namespace")

    # Función que imprime los valores y la ruta al xacro
    print_function = OpaqueFunction(
        function=print_values,
        args=[description_package, robot_name, use_sim, namespace]
    )
    """ print (LaunchDescription (
        declared_arguments + [print_function]
    )) """
    return LaunchDescription(
        declared_arguments + [print_function]
    )