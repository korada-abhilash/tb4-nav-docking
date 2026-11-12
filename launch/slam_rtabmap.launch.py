"""RTAB-Map RGB-D SLAM on the simulated TurtleBot 4 (RGB-D camera, RPLIDAR scan, wheel odometry).

Wheel odometry is the motion prior, the 2D lidar builds the occupancy grid and refines loop closures,
and the RGB-D images give visual loop closure. Publishes /map and the map->odom transform.

ros2 launch /ws/launch/slam_rtabmap.launch.py [database_path:=/ws/outputs/rtabmap.db] [localization:=false]
"""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

RTABMAP_ARGS = " ".join([
    "--Reg/Strategy 1",            # ICP on the lidar scan to refine visual links
    "--Reg/Force3DoF true",        # planar robot
    "--RGBD/NeighborLinkRefining true",
    "--RGBD/ProximityBySpace true",
    "--Grid/Sensor 0",             # occupancy grid from the lidar
    "--Grid/RangeMax 8",
])


def generate_launch_description():
    rtabmap = PathJoinSubstitution([get_package_share_directory("rtabmap_launch"), "launch", "rtabmap.launch.py"])
    common = [
        ("use_sim_time", "true"),
        ("rgb_topic", "/rgbd_camera/image"),
        ("depth_topic", "/rgbd_camera/depth_image"),
        ("camera_info_topic", "/rgbd_camera/camera_info"),
        ("frame_id", "base_link"),
        ("odom_topic", "/odom"),
        ("visual_odometry", "false"),
        ("subscribe_scan", "true"),
        ("scan_topic", "/scan"),
        ("approx_sync", "true"),
        ("rtabmap_viz", "false"),
        ("rviz", "false"),
        ("map_topic", "/map"),  # default is /rtabmap/map; Nav2's costmaps listen on /map
        ("database_path", LaunchConfiguration("database_path")),
    ]
    mapping = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(rtabmap),
        launch_arguments=common + [("rtabmap_args", "--delete_db_on_start " + RTABMAP_ARGS)],
        condition=UnlessCondition(LaunchConfiguration("localization")),
    )
    localizing = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(rtabmap),
        launch_arguments=common + [("localization", "true"), ("rtabmap_args", RTABMAP_ARGS)],
        condition=IfCondition(LaunchConfiguration("localization")),
    )
    return LaunchDescription([
        DeclareLaunchArgument("database_path", default_value="/ws/outputs/rtabmap_warehouse.db"),
        DeclareLaunchArgument("localization", default_value="false"),
        mapping,
        localizing,
    ])
