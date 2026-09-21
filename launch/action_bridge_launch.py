import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    default_params = os.path.join(
        get_package_share_directory("athena_action_bridge"), "config", "action_bridge.yaml"
    )

    namespace = LaunchConfiguration("namespace")
    params_file = LaunchConfiguration("params_file")
    use_sim_time = LaunchConfiguration("use_sim_time")
    log_level = LaunchConfiguration("log_level")

    return LaunchDescription([
        DeclareLaunchArgument(
            "namespace",
            default_value="",
            description="Namespace of the robot, for example husky",
        ),
        DeclareLaunchArgument(
            "params_file",
            default_value=default_params,
            description="Parameter file with the action names",
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use the simulation clock",
        ),
        DeclareLaunchArgument(
            "log_level",
            default_value="info",
            description="Log level: debug, info, warn, error",
        ),
        Node(
            package="athena_action_bridge",
            executable="action_bridge_node",
            name="action_bridge",
            namespace=namespace,
            parameters=[
                params_file,
                {"use_sim_time": ParameterValue(use_sim_time, value_type=bool)},
            ],
            arguments=["--ros-args", "--log-level", log_level],
            output="screen",
        ),
    ])
