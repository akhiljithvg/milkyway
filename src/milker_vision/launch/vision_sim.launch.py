import os
import xacro
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_milker_vision = get_package_share_directory('milker_vision')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

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

    # 4. Bridge Gazebo PointCloud2 to ROS 2
    gz_ros_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        arguments=[
            '/camera/depth/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
        ],
        output='screen',
    )

    # 5. Launch custom 6-DOF Quad Teat Perception Node
    teat_detector = Node(
        package='milker_vision',
        executable='teat_detector.py',
        name='teat_detector',
        output='screen',
    )

    # 6. Optional RViz 2 Node
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
        gz_args_arg,
        rviz_arg,
        gz_sim,
        robot_state_publisher,
        spawn_vision_rig,
        gz_ros_bridge,
        teat_detector,
        rviz_node,
    ])
