import os
import yaml
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import xacro

def load_file(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, "r") as file:
            return file.read()
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, "r") as file:
            return yaml.safe_load(file)
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None


def generate_launch_description():
    robot_description_config = xacro.process_file(
        os.path.join(
            get_package_share_directory("moveit_resources_panda_moveit_config"),
            "config",
            "panda.urdf.xacro",
        )
    )
    robot_description = {"robot_description": robot_description_config.toxml()}

    robot_description_semantic_config = load_file(
        "moveit_resources_panda_moveit_config", "config/panda.srdf"
    )
    robot_description_semantic = {
        "robot_description_semantic": robot_description_semantic_config
    }

    kinematics_yaml = load_yaml(
        "moveit_resources_panda_moveit_config", "config/kinematics.yaml"
    )

    # 第四节
    # move_group_demo = Node(
    #     package="demo_moveit",
    #     executable="demo_moveit",
    #     name="demo_moveit",
    #     output="screen",
    #     parameters=[robot_description, 
    #                 robot_description_semantic, 
    #                 kinematics_yaml,
    #                 ],
    # )

    # 第五节
    # move_group_demo = Node(
    #     package="demo_moveit",
    #     executable="visualize_in_rviz",
    #     name="visualize_in_rviz",
    #     output="screen",
    #     parameters=[robot_description, 
    #                 robot_description_semantic, 
    #                 kinematics_yaml,
    #                 ],
    # )

    # 第六节
    move_group_demo = Node(
        package="demo_moveit",
        executable="planning_around_objects",
        name="planning_around_objects",
        output="screen",
        parameters=[robot_description, 
                    robot_description_semantic, 
                    kinematics_yaml,
                    ],
    )

    return LaunchDescription([move_group_demo])
