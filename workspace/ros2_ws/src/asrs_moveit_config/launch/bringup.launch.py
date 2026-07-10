from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import xacro
import yaml


def _load_yaml(path):
    with open(path, "r") as f:
        return yaml.safe_load(f)


def _load_moveit_yaml(path):
    data = _load_yaml(path)
    if not isinstance(data, dict):
        return {}
    if "move_group" in data and isinstance(data["move_group"], dict):
        mg = data["move_group"]
        if "ros__parameters" in mg and isinstance(mg["ros__parameters"], dict):
            return mg["ros__parameters"]
        return mg
    return data


def generate_launch_description():
    pkg_cfg = get_package_share_directory("asrs_moveit_config")
    pkg_desc = get_package_share_directory("asrs_description")

    xacro_path = os.path.join(pkg_desc, "urdf", "asrs_robot.urdf.xacro")
    srdf_path = os.path.join(pkg_cfg, "config", "asrs_robot.srdf")
    rviz_path = os.path.join(pkg_cfg, "config", "moveit.rviz")

    kinematics_yaml = _load_moveit_yaml(
        os.path.join(pkg_cfg, "config", "kinematics.yaml")
    )
    ompl_planning_yaml = _load_moveit_yaml(
        os.path.join(pkg_cfg, "config", "ompl_planning.yaml")
    )
    moveit_ctrls_yaml = _load_moveit_yaml(
        os.path.join(pkg_cfg, "config", "moveit_controllers.yaml")
    )
    traj_exec_yaml = _load_moveit_yaml(
        os.path.join(pkg_cfg, "config", "trajectory_execution.yaml")
    )
    joint_limits_yaml = _load_moveit_yaml(
        os.path.join(pkg_cfg, "config", "joint_limits.yaml")
    )

    robot_description_planning = {"robot_description_planning": joint_limits_yaml}

    ompl_cfg = ompl_planning_yaml.setdefault("ompl", {})
    ompl_cfg.setdefault("planning_plugin", "ompl_interface/OMPLPlanner")
    ompl_cfg.setdefault(
        "request_adapters",
        "default_planner_request_adapters/AddTimeOptimalParameterization "
        "default_planner_request_adapters/FixWorkspaceBounds "
        "default_planner_request_adapters/FixStartStateBounds "
        "default_planner_request_adapters/FixStartStateCollision "
        "default_planner_request_adapters/FixStartStatePathConstraints",
    )
    ompl_cfg.setdefault("start_state_max_bounds_error", 0.1)
    ompl_planning_yaml.setdefault("default_planning_pipeline", "ompl")
    ompl_planning_yaml.setdefault("planning_pipelines", ["ompl"])

    with open(srdf_path, "r") as f:
        robot_description_semantic = {"robot_description_semantic": f.read()}

    urdf_xml = xacro.process_file(xacro_path).toxml()
    robot_description = {"robot_description": urdf_xml}

    no_sim_time = {"use_sim_time": False}

    # Static TF: map -> base_link
    map_to_base_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="map_to_base_link_tf",
        arguments=["0", "0", "0", "0", "0", "0", "map", "base_link"],
        output="screen",
    )

    # Robot state publisher
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[robot_description, no_sim_time],
        output="screen",
    )

    # Teensy bridge
    bridge_node = Node(
        package="asrs_bridge",
        executable="teensy_bridge",
        name="teensy_bridge",
        output="screen",
    )

    # MoveIt move_group
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_planning,
            kinematics_yaml,
            ompl_planning_yaml,
            traj_exec_yaml,
            moveit_ctrls_yaml,
            no_sim_time,
        ],
    )

    # RViz
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz",
        arguments=["-d", rviz_path],
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics_yaml,
            no_sim_time,
        ],
        output="screen",
    )

    # Pick and place node
    pick_place_node = Node(
        package="asrs_pickplace",
        executable="pick_place_node",
        name="pick_place_node",
        output="screen",
    )

    return LaunchDescription(
        [
            map_to_base_tf,
            robot_state_publisher_node,
            bridge_node,
            move_group_node,
            rviz_node,
            pick_place_node,
        ]
    )
