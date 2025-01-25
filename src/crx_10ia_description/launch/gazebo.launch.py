from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, IncludeLaunchDescription
from launch.substitutions import Command, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from os import pathsep
from ament_index_python.packages import get_package_share_directory, get_package_prefix

def generate_launch_description():
    
    crx_10ia_description = get_package_share_directory("crx_10ia_description")
    crx_10ia_description_prefix = get_package_prefix("crx_10ia_description")
    
    world_path = os.path.join(crx_10ia_description, "world")
    world_path += os.pathsep + os.path.join(crx_10ia_description_prefix, "share")
    
    gz_sim_resource_path = SetEnvironmentVariable("GZ_SIM_RESOURCE_PATH", world_path)
       
    model_arg = DeclareLaunchArgument(
        name="model", 
        default_value=os.path.join(get_package_share_directory("crx_10ia_description"), "urdf", "crx10ia.urdf.xacro"),
        description="absolute path to the robot URDF file"
    )
    
    world_arg = DeclareLaunchArgument(
        name="world", 
        default_value=os.path.join(get_package_share_directory("crx_10ia_description"), "world", "world.sdf"),
        description="simulation world"
    )
    
    robot_description = ParameterValue(Command(["xacro ", LaunchConfiguration("model")]), value_type=str)
    
    robot_state_publisher = Node(
        package="robot_state_publisher", 
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description}]
    )
    
    # Start Gazebo with the specified world file, using string formatting for the world argument
    start_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')),
        launch_arguments=[
            ('gz_args', [LaunchConfiguration("world"), " --physics-engine gz-physics-bullet-featherstone-plugin"])
        ]
    )

    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=["-entity", "crx_10ia", "-topic", "robot_description"]
    )

    # Bridge
    lidar_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['lidar@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
                   '/lidar/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked'],
        output='screen'
    )

    camera_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/camera/image_raw@sensor_msgs/msg/Image@gz.msgs.Image',
            '/camera/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo'],
        output='screen'
    )
    
    #contact_bridge = Node(
    #        package='ros_gz_bridge',
    #        executable='parameter_bridge',
    #        name='contact_sensor_bridge',
    #        arguments=['/vacuum_gripper/touched@ros_gz_interfaces/msg/Contacts@gz.msgs.Contacts'],
    #        output='screen'
    #    )


    
    return LaunchDescription([
        gz_sim_resource_path,
        model_arg, 
        world_arg,  # Add world argument to launch description
        robot_state_publisher, 
        start_gazebo, 
        spawn_robot,
        lidar_bridge,
        camera_bridge,
        #contact_bridge
    ])
