"""
demo.launch.py — HC30PL Workcell MoveIt Demo

Fixes vs previous version:
  - Command(["xacro ..."]) replaced with subprocess.run('xacro')
    → avoids the "got '()' of type tuple" ParameterValue error in Jazzy
  - All parameters are plain Python scalars / dicts (no substitution objects)

Nodes:
  1. joint_state_publisher        → /joint_states  (all arm joints at 0)
  2. robot_state_publisher        → TF tree from URDF + joint_states
  3. static_tf  map → world       → silences RViz "no transform to [map]"
  4. move_group                   → MoveIt planning (fake execution)
  5. rviz2  -f world              → visualisation, fixed frame = world
"""

import os
import subprocess
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


# ─────────────────────────────────────────────────────────────────────────────
def load_yaml(package_name, relative_path):
    pkg_share = get_package_share_directory(package_name)
    with open(os.path.join(pkg_share, relative_path)) as f:
        return yaml.safe_load(f)


def load_xacro(xacro_file_path):
    """Run xacro and return the resulting URDF as a plain string."""
    result = subprocess.run(
        ["xacro", xacro_file_path],
        capture_output=True, text=True, check=True
    )
    return result.stdout
# ─────────────────────────────────────────────────────────────────────────────


def generate_launch_description():

    desc_pkg   = get_package_share_directory("hc30pl_workcell_description")
    config_pkg = get_package_share_directory("hc30pl_workcell_moveit_config")

    # ── Robot Description  (plain string — no substitution objects) ───────────
    xacro_file = os.path.join(desc_pkg, "urdf", "workcell.xacro")
    urdf_string = load_xacro(xacro_file)          # plain str ✓
    robot_description = {"robot_description": urdf_string}

    # ── Semantic Description (SRDF) ──────────────────────────────────────────
    srdf_file = os.path.join(config_pkg, "config", "hc30pl_workcell.srdf")
    with open(srdf_file) as f:
        srdf_string = f.read()
    robot_description_semantic = {"robot_description_semantic": srdf_string}

    # ── Kinematics & Joint Limits ────────────────────────────────────────────
    kinematics_yaml   = load_yaml("hc30pl_workcell_moveit_config", "config/kinematics.yaml")
    joint_limits_yaml = load_yaml("hc30pl_workcell_moveit_config", "config/joint_limits.yaml")

    robot_description_kinematics = {"robot_description_kinematics": kinematics_yaml}
    joint_limits = {"robot_description_planning": joint_limits_yaml}

    # ── Planning (OMPL) ──────────────────────────────────────────────────────
    ompl_planning_pipeline = {
        "planning_plugin": "ompl_interface/OMPLPlanner",
        "request_adapters": (
            "default_planning_request_adapters/ResolveConstraintFrames "
            "default_planning_request_adapters/ValidateWorkspaceBounds "
            "default_planning_request_adapters/CheckStartStateBounds "
            "default_planning_request_adapters/CheckStartStateCollision"
        ),
        "start_state_max_bounds_error": 0.1,
    }

    # ── Trajectory Execution / Fake Hardware ─────────────────────────────────
    trajectory_execution = {
        "moveit_manage_controllers": True,
        "trajectory_execution.allowed_execution_duration_scaling": 1.2,
        "trajectory_execution.allowed_goal_duration_margin": 0.5,
        "trajectory_execution.allowed_start_tolerance": 0.01,
    }

    planning_scene_monitor = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
        "monitor_dynamics": False,
    }

    moveit_controllers = {
        "moveit_simple_controller_manager": {
            "controller_names": ["manipulator_controller"],
        },
        "manipulator_controller": {
            "type":      "FollowJointTrajectory",
            "action_ns": "follow_joint_trajectory",
            "default":   True,
            "joints": [
                "joint_1_s",
                "joint_2_l",
                "joint_3_u",
                "joint_4_r",
                "joint_5_b",
                "joint_6_t",
            ],
        },
    }

    # ═══════════════════════════════════════════════════════════════════════════
    # NODE 1 — joint_state_publisher
    # ═══════════════════════════════════════════════════════════════════════════
    joint_state_publisher_node = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        output="screen",
        parameters=[robot_description, {"rate": 50.0}],
    )

    # ═══════════════════════════════════════════════════════════════════════════
    # NODE 2 — robot_state_publisher
    # ═══════════════════════════════════════════════════════════════════════════
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[robot_description, {"publish_frequency": 50.0}],
    )

    # ═══════════════════════════════════════════════════════════════════════════
    # NODE 3 — static TF  map → world
    #   RViz's default Fixed Frame is 'map'. Publishing an identity TF
    #   map→world prevents "No transform from [X] to [map]" warnings.
    #   (parent=map  child=world  so world is reachable from map)
    # ═══════════════════════════════════════════════════════════════════════════
    static_tf_map_to_world = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_tf_map_to_world",
        output="screen",
        arguments=["0", "0", "0", "0", "0", "0", "1", "map", "world"],
    )

    # ═══════════════════════════════════════════════════════════════════════════
    # NODE 4 — move_group
    # ═══════════════════════════════════════════════════════════════════════════
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            joint_limits,
            ompl_planning_pipeline,
            trajectory_execution,
            planning_scene_monitor,
            moveit_controllers,
            {"use_sim_time": False},
            {
                "moveit_controller_manager":
                    "moveit_fake_controller_manager/MoveItFakeControllerManager"
            },
            {"fake_execution_type": "interpolate"},
        ],
    )

    # ═══════════════════════════════════════════════════════════════════════════
    # NODE 5 — RViz2  (-f world forces the correct fixed frame)
    # ═══════════════════════════════════════════════════════════════════════════
    rviz_config_file = os.path.join(config_pkg, "config", "moveit.rviz")
    rviz_args = ["-f", "world"]
    if os.path.exists(rviz_config_file):
        rviz_args = ["-d", rviz_config_file] + rviz_args

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=rviz_args,
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
        ],
    )

    return LaunchDescription([
        joint_state_publisher_node,   # 1
        robot_state_publisher_node,   # 2
        static_tf_map_to_world,       # 3
        move_group_node,              # 4
        rviz_node,                    # 5
    ])
