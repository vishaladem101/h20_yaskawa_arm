import os
import subprocess
import yaml

from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    ExecuteProcess,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


# ─────────────────────────────────────────────────────────────────────────────
def load_yaml(package_name, relative_path):
    pkg_share = get_package_share_directory(package_name)
    with open(os.path.join(pkg_share, relative_path)) as f:
        return yaml.safe_load(f)


def load_xacro(xacro_file_path):
    """Run xacro and return the URDF as a plain string."""
    result = subprocess.run(
        ["xacro", xacro_file_path],
        capture_output=True, text=True, check=True,
        env=os.environ,
    )
    return result.stdout
# ─────────────────────────────────────────────────────────────────────────────


def generate_launch_description():

    pkg_hc20_support   = get_package_share_directory("motoman_hc20_support")
    pkg_hc20_prefix    = get_package_prefix("motoman_hc20_support")
    pkg_robotiq_prefix = get_package_prefix("robotiq_description")

    # ── Robot Description ─────────────────────────────────────────────────────
    xacro_file  = os.path.join(pkg_hc20_support, "urdf", "hc20.xacro")
    urdf_string = load_xacro(xacro_file)
    robot_description = {"robot_description": urdf_string}

    # ── Semantic Description (SRDF) ───────────────────────────────────────────
    srdf_file = os.path.join(pkg_hc20_support, "config", "hc20.srdf")
    with open(srdf_file) as f:
        srdf_string = f.read()
    robot_description_semantic = {"robot_description_semantic": srdf_string}

    # ── Kinematics & Joint Limits ─────────────────────────────────────────────
    kinematics_yaml   = load_yaml("motoman_hc20_support", "config/kinematics.yaml")
    joint_limits_yaml = load_yaml("motoman_hc20_support", "config/joint_limits.yaml")

    robot_description_kinematics = {"robot_description_kinematics": kinematics_yaml}
    joint_limits                 = {"robot_description_planning":   joint_limits_yaml}

    # ── Planning Pipeline YAML path ───────────────────────────────────────────
    # Passed as a file path so rclcpp reads the nested ompl.planning_plugins /
    # ompl.request_adapters keys directly without launch_ros flattening them.
    ompl_planning_yaml_path = os.path.join(
        pkg_hc20_support, "config", "ompl_planning.yaml"
    )

    # ── Trajectory Execution ──────────────────────────────────────────────────
    trajectory_execution = {
        "moveit_manage_controllers": True,
        "trajectory_execution.allowed_execution_duration_scaling": 1.2,
        "trajectory_execution.allowed_goal_duration_margin": 0.5,
        "trajectory_execution.allowed_start_tolerance": 0.01,
        "moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager",
    }

    # ── Planning Scene Monitor ────────────────────────────────────────────────
    planning_scene_monitor = {
        "publish_planning_scene":     True,
        "publish_geometry_updates":   True,
        "publish_state_updates":      True,
        "publish_transforms_updates": True,
        "monitor_dynamics":           False,
    }

    # ── MoveIt Controller Manager ─────────────────────────────────────────────
    # NOTE: launch_ros only flattens ONE level of nesting. Nested dicts inside
    # dicts are NOT recursively flattened, so we must use fully explicit keys.
    moveit_controllers = {
        "moveit_simple_controller_manager.controller_names": [
            "manipulator_controller",
            "gripper_controller",
        ],
        "moveit_simple_controller_manager.manipulator_controller.type":      "FollowJointTrajectory",
        "moveit_simple_controller_manager.manipulator_controller.action_ns": "follow_joint_trajectory",
        "moveit_simple_controller_manager.manipulator_controller.default":   True,
        "moveit_simple_controller_manager.manipulator_controller.joints": [
            "joint_1_s", "joint_2_l", "joint_3_u",
            "joint_4_r", "joint_5_b", "joint_6_t",
        ],
        "moveit_simple_controller_manager.gripper_controller.type":      "FollowJointTrajectory",
        "moveit_simple_controller_manager.gripper_controller.action_ns": "follow_joint_trajectory",
        "moveit_simple_controller_manager.gripper_controller.default":   True,
        "moveit_simple_controller_manager.gripper_controller.joints": [
            "robotiq_85_left_knuckle_joint",
        ],
    }

    # ── Move Group Node ───────────────────────────────────────────────────────
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            joint_limits,
            trajectory_execution,
            planning_scene_monitor,
            moveit_controllers,
            {"use_sim_time": True},
            ompl_planning_yaml_path,   # last → overrides any default pipeline params
        ],
    )

    # ── Gazebo Resource Paths ─────────────────────────────────────────────────
    resource_paths = [
        os.path.join(pkg_hc20_prefix,    "share"),
        os.path.join(pkg_robotiq_prefix, "share"),
        os.environ.get("GZ_SIM_RESOURCE_PATH", ""),
    ]
    gz_resource_path = ":".join(filter(None, resource_paths))
    set_gz_path  = SetEnvironmentVariable("GZ_SIM_RESOURCE_PATH",    gz_resource_path)
    set_ign_path = SetEnvironmentVariable("IGN_GAZEBO_RESOURCE_PATH", gz_resource_path)

    # ── Gazebo Sim ────────────────────────────────────────────────────────────
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("ros_gz_sim"),
                "launch", "gz_sim.launch.py",
            )
        ),
        launch_arguments={"gz_args": "-r empty.sdf"}.items(),
    )

    # ── Spawn Robot ───────────────────────────────────────────────────────────
    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=["-topic", "robot_description", "-name", "hc20", "-z", "0.1"],
        output="screen",
    )

    # ── Robot State Publisher ─────────────────────────────────────────────────
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[robot_description, {"use_sim_time": True}, {"publish_frequency": 50.0}],
    )

    # ── RViz ─────────────────────────────────────────────────────────────────
    rviz_config_file = os.path.join(pkg_hc20_support, "launch", "display.rviz")
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        output="screen",
        arguments=["-d", rviz_config_file],
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            {"use_sim_time": True},
        ],
    )

    # ── Clock Bridge ─────────────────────────────────────────────────────────
    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
        output="screen",
    )

    # ── Controller Spawners (triggered after robot spawned in Gazebo) ─────────
    load_joint_state_broadcaster = ExecuteProcess(
        cmd=["ros2", "control", "load_controller",
             "--set-state", "active", "joint_state_broadcaster"],
        output="screen",
    )
    load_manipulator_controller = ExecuteProcess(
        cmd=["ros2", "control", "load_controller",
             "--set-state", "active", "manipulator_controller"],
        output="screen",
    )
    load_gripper_controller = ExecuteProcess(
        cmd=["ros2", "control", "load_controller",
             "--set-state", "active", "gripper_controller"],
        output="screen",
    )

    return LaunchDescription([
        set_gz_path,
        set_ign_path,
        gz_sim,
        spawn_robot,
        robot_state_publisher,
        move_group_node,
        rviz_node,
        bridge,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_robot,
                on_exit=[
                    load_joint_state_broadcaster,
                    load_manipulator_controller,
                    load_gripper_controller,
                ],
            )
        ),
    ])
