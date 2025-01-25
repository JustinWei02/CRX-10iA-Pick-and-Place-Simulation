from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import xacro
import yaml

def generate_launch_description():
    # Paths to robot files
    description_package = get_package_share_directory("crx_10ia_description")
    moveit_package = get_package_share_directory("crx_10ia_moveit")

    # Generate the robot description (URDF)
    xacro_file = os.path.join(description_package, "urdf", "crx10ia.urdf.xacro")
    doc = xacro.process_file(xacro_file)
    robot_description = {"robot_description": doc.toxml()}

    # Load SRDF
    srdf_file = os.path.join(moveit_package, "config", "crx10ia.srdf")
    with open(srdf_file, "r") as file:
        robot_description_semantic = {"robot_description_semantic": file.read()}

    # Load kinematics.yaml and parse its contents
    kinematics_file = os.path.join(moveit_package, "config", "kinematics.yaml")
    with open(kinematics_file, "r") as file:
        kinematics_config = yaml.safe_load(file)
    robot_description_kinematics = {"robot_description_kinematics": kinematics_config}

    # Add planning_scene_monitor parameters
    planning_scene_monitor_parameters = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
        "publish_robot_description": True,
        "publish_robot_description_semantic": True,
    }

    # Node configuration
    block_pick_place_node = Node(
        package="crx_10ia_action",
        executable="block_pick_place_node",
        name="block_pick_place_node",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            planning_scene_monitor_parameters,
        ],
        arguments=["--ros-args", "--log-level", "info"]
    )

    return LaunchDescription([block_pick_place_node])