import os
import xacro
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, AppendEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_milker_vision = get_package_share_directory('milker_vision')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    share_parent_dir = os.path.dirname(pkg_milker_vision)

    # Ensure Gazebo Harmonic can locate meshes referenced with package://
    set_gz_resource_path = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        share_parent_dir
    )
    set_ign_resource_path = AppendEnvironmentVariable(
        'IGN_GAZEBO_RESOURCE_PATH',
        share_parent_dir
    )

    # Launch Arguments
    gz_args_arg = DeclareLaunchArgument(
        'gz_args',
        default_value='-r empty.sdf',
        description='Arguments for Gazebo Sim (e.g. -r empty.sdf or -r -s empty.sdf for headless)'
    )

    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='false',
        description='Whether to launch RViz 2'
    )

    # 1. Launch Gazebo Harmonic
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': LaunchConfiguration('gz_args')}.items(),
    )

    # 2. Parse XACRO and start robot_state_publisher
    xacro_file = os.path.join(pkg_milker_vision, 'urdf', 'vision_rig.xacro')
    robot_description_raw = xacro.process_file(xacro_file).toxml()

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_raw}],
    )

    # 3. Spawn entity into Gazebo
    spawn_vision_rig = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_vision_rig',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'vision_rig',
            '-allow_renaming', 'true'
        ],
        output='screen',
    )

    # 4. Bridge Gazebo PointClouds, JointStates, and Arm Joint Commands
    gz_ros_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        arguments=[
            '/overhead_camera/depth/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
            '/ee_camera/depth/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
            '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
            '/arm/base_to_part_2/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double',
            '/arm/part_2_to_part_3/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double',
            '/arm/part_3_to_part_4__3/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double',
            '/arm/part_4__3_to_part_4/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double',
            '/arm/part_4_to_part_4__2/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double',
            '/arm/part_4__2_to_part_5/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double',
            '/arm/part_5_to_part_2_3/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double',
        ],
        output='screen',
    )

    # 5. Launch Phase 1: Overhead Cow & Imaginary Udder Detector
    overhead_detector = Node(
        package='milker_vision',
        executable='overhead_detector.py',
        name='overhead_detector',
        output='screen',
    )

    # 6. Launch Phase 2: End-Effector Close-Range Teat Detector
    ee_teat_detector = Node(
        package='milker_vision',
        executable='ee_teat_detector.py',
        name='ee_teat_detector',
        output='screen',
    )

    # 7. Launch Arm Controller & Sequential Cup Docking Node
    arm_controller = Node(
        package='milker_vision',
        executable='arm_controller.py',
        name='arm_controller',
        output='screen',
    )

    # 8. Optional RViz 2 Node
    rviz_config_file = os.path.join(pkg_milker_vision, 'config', 'milker_tracking.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        condition=IfCondition(LaunchConfiguration('rviz')),
        output='screen',
    )

    return LaunchDescription([
        set_gz_resource_path,
        set_ign_resource_path,
        gz_args_arg,
        rviz_arg,
        gz_sim,
        robot_state_publisher,
        spawn_vision_rig,
        gz_ros_bridge,
        overhead_detector,
        ee_teat_detector,
        arm_controller,
        rviz_node,
    ])
