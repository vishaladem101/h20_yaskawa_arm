import os
import yaml
from launch import LaunchDescription
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

def load_yaml(package_name, relative_path):
    pkg_share = get_package_share_directory(package_name)
    with open(os.path.join(pkg_share, relative_path)) as f:
        return yaml.safe_load(f)

def generate_launch_description():
    pkg_hc20_support = get_package_share_directory("motoman_hc20_support")
    pkg_executor = get_package_share_directory("goal_pose_executor")

    # Robot Description (Required by MoveGroupInterface)
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare("motoman_hc20_support"), "urdf", "hc20.xacro"]),
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    # Semantic Description (SRDF)
    srdf_file = os.path.join(pkg_hc20_support, "config", "hc20.srdf")
    with open(srdf_file) as f:
        srdf_string = f.read()
    robot_description_semantic = {"robot_description_semantic": srdf_string}

    # Kinematics
    kinematics_yaml = load_yaml("motoman_hc20_support", "config/kinematics.yaml")

    # Goal Pose Executor Node
    poses_yaml = os.path.join(pkg_executor, "poses.yaml")
    
    goal_pose_executor_node = Node(
        package="goal_pose_executor",
        executable="goal_pose_executor",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            {"robot_description_kinematics": kinematics_yaml},
            poses_yaml,
            {"use_sim_time": True},
        ],
    )

    return LaunchDescription([
        goal_pose_executor_node,
    ])
