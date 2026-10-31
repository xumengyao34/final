# 实车建图启动: livox_imu_level + FAST-LIO + live_gridmap -> RViz 实时 2D 栅格地图
# 直接覆盖保存图片+yaml
import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# 标定值来自 mbf_test 的 launch/lidar_extrinsic.py
sys.path.insert(0, os.path.join(
    get_package_share_directory('mbf_test'), 'launch'))
from lidar_extrinsic import ROLL, PITCH, YAW


def generate_launch_description():
    rviz = LaunchConfiguration('rviz')
    save_dir = LaunchConfiguration('save_dir')
    save_name = LaunchConfiguration('save_name')

    # /livox/imu 旋平 
    imu_level = Node(
        package='mbf_test',
        executable='livox_imu_level.py',
        name='livox_imu_level',
        output='screen',
        parameters=[{
            'level_roll': ROLL,
            'level_pitch': PITCH,
            'level_yaw': YAW,
        }],
    )

    # FAST-LIO 里程计 + 建图: 每 1s 把累积地图点云发到 /Laser_map (camera_init 系),
    # 发布 /Odometry + TF camera_init -> body。mid360.yaml 里 pcd_save_en:true /
    # interval:-1, ctrl-C 还会在 FAST_LIO/PCD/ 存一份 PCD 原始点云。
    # 注意: 不带 real_car 里的 /Odometry->/Odometry_raw 改名 (建图不用 odom_guard)。
    fast_lio = Node(
        package='fast_lio',
        executable='fastlio_mapping',
        name='fastlio_mapping',
        output='screen',
        parameters=[
            os.path.join(get_package_share_directory('fast_lio'), 'config', 'mid360.yaml'),
        ],
    )

    # 建图期间 map 系 = camera_init 系 (都在建图起点), 恒等 TF 让 RViz 固定 map 帧
    # 能看到车在动 (camera_init -> body 由 FAST-LIO 发)
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_map_camera_init',
        arguments=['--x', '0', '--y', '0', '--z', '0',
                   '--roll', '0', '--pitch', '0', '--yaw', '0',
                   '--frame-id', 'map', '--child-frame-id', 'camera_init'],
    )

    # /Laser_map -> 实时 2D 栅格 /map (RViz 可见), ctrl-C 自动保存
    gridmap = Node(
        package='t11_10',
        executable='live_gridmap_node',
        name='live_gridmap',
        output='screen',
        parameters=[{
            'save_dir': save_dir,
            'save_name': save_name,
        }],
    )

    rviz_cfg = os.path.join(
        get_package_share_directory('t11_10'), 'rviz', 'mapping.rviz')
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_cfg],
        condition=IfCondition(rviz),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'rviz', default_value='true',
            description='是否启动 RViz2'),
        DeclareLaunchArgument(
            'save_dir', default_value='/home/xmy/final/src/mbf_test/maps',
            description='地图保存目录 (与导航 launch 的 map 同一目录)'),
        DeclareLaunchArgument(
            'save_name', default_value='now_map',
            description='保存文件名 (直接覆盖, 不留备份)'),
        imu_level,
        fast_lio,
        static_tf,
        gridmap,
        rviz_node,
    ])
