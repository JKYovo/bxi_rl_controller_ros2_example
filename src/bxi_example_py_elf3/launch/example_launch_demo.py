import os
from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    xml_file_name = "data/mujoco_simulation/elf3.xml"
    default_xml_file = os.path.join(
        get_package_share_path("bxi_example_py_elf3"), xml_file_name
    )
    xml_file = LaunchConfiguration("model_file")
    state_machine_config = os.path.join(
        get_package_share_path("bxi_example_py_elf3"),
        "config/elf3_state_machine.yaml",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "model_file",
                default_value=default_xml_file,
                description="Absolute MuJoCo XML path; defaults to the official ELF3 model",
            ),
            Node(
                package="mujoco",
                executable="simulation",
                name="simulation_mujoco",
                output="screen",
                parameters=[
                    {"simulation/model_file": xml_file},
                ],
                emulate_tty=True,
            ),
            Node(
                package="bxi_example_py_elf3",
                executable="bxi_example_py_elf3_demo",
                name="bxi_example_py_elf3_demo",
                output="screen",
                parameters=[
                    {"/topic_prefix": "simulation/"},
                    {"/state_machine_config": state_machine_config},
                ],
                emulate_tty=True,
            ),
        ]
    )
