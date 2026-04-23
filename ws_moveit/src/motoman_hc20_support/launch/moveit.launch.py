import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import FindExecutable


def load_yaml(package_name, relative_path):
    pkg_share = get_package_share_directory(package_name)
    with open(os.path.join(pkg_share, relative_path)) as f:
        return yaml.safe_load(f)


def generate_launch_description():
    pkg_hc20_support = get_package_share_directory("motoman_hc20_support")

    # ── Robot Description (via xacro) ────────────────────────────────────────
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare("motoman_hc20_support"), "urdf", "hc20.xacro"]),
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    # ── Semantic Description (SRDF) ───────────────────────────────────────────
    srdf_file = os.path.join(pkg_hc20_support, "config", "hc20.srdf")
    with open(srdf_file) as f:
        srdf_string = f.read()
    robot_description_semantic = {"robot_description_semantic": srdf_string}

    # ── Kinematics & Joint Limits ─────────────────────────────────────────────
    kinematics_yaml = load_yaml("motoman_hc20_support", "config/kinematics.yaml")
    joint_limits_yaml = load_yaml("motoman_hc20_support", "config/joint_limits.yaml")

    # ── Planning pipeline YAML path (file path preserves nested YAML keys) ────
    ompl_planning_yaml_path = os.path.join(
        pkg_hc20_support, "config", "ompl_planning.yaml"
    )

    # ── Trajectory Execution ──────────────────────────────────────────────────
    trajectory_execution = {
        "moveit_manage_controllers": True,
        "trajectory_execution.allowed_execution_duration_scaling": 1.2,
        "trajectory_execution.allowed_goal_duration_margin": 0.5,
        "trajectory_execution.allowed_start_tolerance": 0.01,
    }

    # ── MoveIt Controller Manager ─────────────────────────────────────────────
    moveit_controllers = {
        "moveit_simple_controller_manager": {
            "controller_names": ["manipulator_controller", "gripper_controller"],
        },
        "manipulator_controller": {
            "type": "FollowJointTrajectory",
            "action_ns": "follow_joint_trajectory",
            "default": True,
            "joints": [
                "joint_1_s",
                "joint_2_l",
                "joint_3_u",
                "joint_4_r",
                "joint_5_b",
                "joint_6_t",
            ],
        },
        "gripper_controller": {
            "type": "FollowJointTrajectory",
            "action_ns": "follow_joint_trajectory",
            "default": True,
            "joints": ["robotiq_85_left_knuckle_joint"],
        },
    }

    # ── Move Group Node Parameters ────────────────────────────────────────────
    move_group_params = {}
    move_group_params.update(robot_description)
    move_group_params.update(robot_description_semantic)
    move_group_params.update({"robot_description_kinematics": kinematics_yaml})
    move_group_params.update({"robot_description_planning": joint_limits_yaml})
    move_group_params.update(trajectory_execution)
    move_group_params.update(moveit_controllers)
    move_group_params.update({"use_sim_time": True})

    # ── Move Group Node ───────────────────────────────────────────────────────
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[move_group_params, ompl_planning_yaml_path],
    )

    # ── RViz ─────────────────────────────────────────────────────────────────
    rviz_config_file = os.path.join(pkg_hc20_support, "launch", "display.rviz")
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_file],
        parameters=[
            robot_description,
            robot_description_semantic,
            {"robot_description_kinematics": kinematics_yaml},
            {"use_sim_time": True},
        ],
    )

    return LaunchDescription([
        move_group_node,
        rviz_node,
    ])
