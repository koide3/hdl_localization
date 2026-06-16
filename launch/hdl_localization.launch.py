from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.conditions import IfCondition
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir
from launch_ros.descriptions import ComposableNode, ParameterFile


def generate_launch_description():
    component_manager = LaunchConfiguration(
        "component_manager", default="hdl_localization_component_manager"
    )
    points_topic = LaunchConfiguration("points_topic", default="/velodyne_points")
    globalmap_pcd = LaunchConfiguration(
        "globalmap_pcd", default=os.path.join(os.path.expanduser("~"), "map.pcd")
    )
    imu_topic = LaunchConfiguration("imu_topic", default="/imu/data")
    use_global_localization = LaunchConfiguration(
        "use_global_localization", default=True
    )
    plot_estimation_errors = LaunchConfiguration(
        "plot_estimation_errors", default=False
    )

    params_file = LaunchConfiguration(
        "params_file",
        default=os.path.join(
            get_package_share_directory("hdl_localization"),
            "param",
            "hdl_localization.yaml",
        ),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "component_manager",
                default_value=component_manager,
                description="name of component manager",
            ),
            DeclareLaunchArgument("imu_topic", default_value=imu_topic),
            DeclareLaunchArgument(
                "points_topic",
                default_value=points_topic,
                description="topic of point cloud",
            ),
            DeclareLaunchArgument("globalmap_pcd", default_value=globalmap_pcd),
            DeclareLaunchArgument(
                "use_global_localization", default_value=use_global_localization
            ),
            DeclareLaunchArgument(
                "plot_estimation_errors", default_value=plot_estimation_errors
            ),
            DeclareLaunchArgument("params_file", default_value=params_file),
            Node(
                package="hdl_global_localization",
                executable="hdl_global_localization_node",
                name="hdl_global_localization_node",
                namespace="hdl_global_localization",
                condition=IfCondition(use_global_localization),
            ),
            ComposableNodeContainer(
                name=component_manager,
                namespace="",
                package="rclcpp_components",
                executable="component_container_mt",
                composable_node_descriptions=[
                    ComposableNode(
                        package="hdl_localization",
                        plugin="hdl_localization::GlobalmapServer",
                        name="global_map_server",
                        parameters=[params_file, {"globalmap_pcd": globalmap_pcd}],
                    ),
                    ComposableNode(
                        package="hdl_localization",
                        plugin="hdl_localization::HdlLocalization",
                        name="hdl_localization",
                        remappings=[
                            ("/velodyne_points", points_topic),
                            ("/gpsimu_driver/imu_data", imu_topic),
                        ],
                        parameters=[
                            params_file,
                            {"use_global_localization": use_global_localization},
                        ],
                    ),
                ],
            ),
            Node(
                package="hdl_localization",
                executable="plot_status.py",
                name="plot_estimation_errors",
                condition=IfCondition(plot_estimation_errors),
            ),
        ]
    )
