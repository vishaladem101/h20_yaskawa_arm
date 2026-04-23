import os
import subprocess
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def load_yaml(package_name, relative_path):
    pkg_share = get_package_share_directory(package_name)
    with open(os.path.join(pkg_share, relative_path)) as f:
        return yaml.safe_load(f)

def load_xacro(xacro_file_path):
    """Run xacro and return the URDF as a plain string."""
    result = subprocess.run(
        ["xacro", xacro_file_path],
        capture_output=True, text=True, check=True,
    )
    return result.stdout

def generate_launch_description():
    pkg_hc20_support = get_package_share_directory("motoman_hc20_support")
    
    # We'll try to find poses.yaml in the share directory after installation
    try:
        pkg_goal_executor = get_package_share_directory("goal_pose_executor")
        poses_config = os.path.join(pkg_goal_executor, "poses.yaml")
    except Exception:
        # Fallback for local development if not yet installed
        poses_config = os.path.join(
            os.getcwd(), "src", "goal_pose_executor", "poses.yaml"
        )

    # 1. Robot Description (URDF)
    xacro_file = os.path.join(pkg_hc20_support, "urdf", "hc20.xacro")
    robot_description = {"robot_description": load_xacro(xacro_file)}

    # 2. Semantic Description (SRDF)
    srdf_file = os.path.join(pkg_hc20_support, "config", "hc20.srdf")
    with open(srdf_file, "r") as f:
        robot_description_semantic = {"robot_description_semantic": f.read()}

    # 3. Kinematics
    kinematics_yaml = load_yaml("motoman_hc20_support", "config/kinematics.yaml")
    robot_description_kinematics = {"robot_description_kinematics": kinematics_yaml}

    # 4. Joint Limits
    joint_limits_yaml = load_yaml("motoman_hc20_support", "config/joint_limits.yaml")
    robot_description_planning = {"robot_description_planning": joint_limits_yaml}

    # 5. Planning Pipeline
    ompl_planning_yaml_path = os.path.join(
        pkg_hc20_support, "config", "ompl_planning.yaml"
    )

    return LaunchDescription([
        Node(
            package="goal_pose_executor",
            executable="goal_pose_executor",
            name="goal_pose_executor",
            output="screen",
            parameters=[
                robot_description,
                robot_description_semantic,
                robot_description_kinematics,
                robot_description_planning,
                ompl_planning_yaml_path,
                poses_config,
                {"use_sim_time": True}
            ],
        )
    ])
