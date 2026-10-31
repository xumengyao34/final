import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

sys.path.insert(0, os.path.join(
    get_package_share_directory('mbf_test'), 'launch'))
from lidar_extrinsic import ROLL, PITCH, YAW


def generate_launch_description():
    # 启动时也可以用参数覆盖:  ros2 launch mbf_test mbf_demo.launch.py map:=/path/to/your_map.yaml
    map_yaml_path = LaunchConfiguration('map')

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    rviz = LaunchConfiguration('rviz', default='true')

    # 统一参数文件 (MBF + Nav2)
    params_file = os.path.join(
        get_package_share_directory('mbf_test'), 'config', 'mbf_params.yaml')

    # AMCL 调参: 局部匹配更硬 + 定错位更快自愈 (粒子/光束数、recovery_alpha_fast)
    amcl_params_file = os.path.join(
        get_package_share_directory('mbf_test'), 'config', 'amcl.yaml')

    # RViz2 配置 (Map / Path / RobotModel / TF 显示)
    rviz_config = os.path.join(
        get_package_share_directory('mbf_test'), 'rviz', 'mbf_demo.rviz')

    # 占位机器人 URDF
    urdf_file = os.path.join(
        get_package_share_directory('mbf_test'), 'urdf', 'dummy_robot.urdf')
    with open(urdf_file, 'r') as f:
        robot_description = f.read()

    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    return LaunchDescription([
        DeclareLaunchArgument(
            'map', default_value='/home/xmy/final/src/mbf_test/maps/now_map.yaml',
            description='地图 yaml 的绝对路径'),
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='是否使用仿真时钟'),
        DeclareLaunchArgument(
            'rviz', default_value='true',
            description='是否启动 RViz2 (无显示环境用 rviz:=false)'),

        # 1. 启动地图服务器加载地图
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            parameters=[{'yaml_filename': map_yaml_path,
                         'use_sim_time': use_sim_time}]
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_map',
            parameters=[{'autostart': True},
                        {'node_names': ['map_server']}]
        ),

        
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_base_livox',
            arguments=['--x', '0.15', '--y', '0', '--z', '0.25',
                       '--roll', str(ROLL), '--pitch', str(PITCH),
                       '--yaw', str(YAW),
                       '--frame-id', 'base_link', '--child-frame-id', 'livox_frame']
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_base_scan',
            arguments=['--x', '0.15', '--y', '0', '--z', '0.25',
                       '--roll', '0', '--pitch', '0', '--yaw', '0',
                       '--frame-id', 'base_link', '--child-frame-id', 'base_scan']
        ),
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            parameters=[
                amcl_params_file,  # 粒子数/光束数/自愈强度, 见 config/amcl.yaml
                {'use_sim_time': use_sim_time},
                {'yaml_filename': map_yaml_path},   # 与 map_server 同一张地图
                {'base_frame_id': 'base_link'},
                # 里程计由 FAST-LIO 提供, odom_guard 合成后发布
                # /Odometry + TF odom -> base_link (topic /Odometry)。
                # map -> odom 由 AMCL 发布, 与标准 Nav2 结构一致
                {'odom_frame_id': 'odom'},
                {'global_frame_id': 'map'},
                {'scan_topic': '/scan'},
                {'laser_min_range': 0.15},
                {'laser_max_range': 15.0},
            ]
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            parameters=[{'autostart': True},
                        {'node_names': ['amcl']}]
        ),

        # ------------------------------------------------------------------
        # [已定方案] 里程计用 FAST-LIO 激光里程计 。FAST-LIO 由 real_car.launch.py 启动,
        #   odom_guard 检查后合成 /Odometry + TF odom -> base_link。
        #   [可选备选] 轮速+IMU EKF 融合: 启用后 odom -> base_link 由 EKF 发布,
        #   odom_guard 与 FAST-LIO 整体停掉即可 (odom 帧名保持不变)
        # ------------------------------------------------------------------
        # Node(
        #     package='robot_localization',
        #     executable='ekf_node',
        #     name='ekf_filter_node',
        #     parameters=[os.path.join(
        #         get_package_share_directory('mbf_test'), 'config', 'ekf.yaml')], 
        #     remappings=[('odometry/filtered', 'odom')]
        # ),

        # 发布 /robot_description，让 RViz 能显示机器人模型
        #  有真车完整 URDF 就替换文件, 根部同样用 base_link
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'robot_description': robot_description}]
        ),

        # 3. Nav2 导航栈 (代价地图 + 规划器 + 控制器 + 生命周期管理)
        #    由 Nav2 提供代价地图和规划/控制后端
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')),
            launch_arguments={'params_file': params_file,
                              'use_sim_time': use_sim_time}.items()
        ),

        # 4. MBF 接口服务器 (mbf_simple_nav + mbf_nav2_connector 插件)
        #    对外提供 MBF 的 get_path / exe_path / recovery / move_base action，
        #    实际规划与控制转发给 Nav2
        # 注意: 不做 cmd_vel remap —— 实际速度由 Nav2 的 velocity_smoother
        # 发布在 /cmd_vel (TwistStamped)，MBF 侧只是转发控制请求
        Node(
            package='mbf_simple_nav',
            executable='mbf_simple_nav_node',
            name='mbf_nav2',
            output='screen',
            parameters=[params_file]
        ),

        # 5. 启动 RViz2 (可用 rviz:=false 关闭)
        #    必须带 -d 配置: 里面预置了 Map(/map)、Path(/plan)、
        #    RobotModel、TF 等显示，否则打开是空白 RViz，看不到路径和机器人
        #    2026-10-25: 主机是 Intel Xe 核显, rviz2 每跑必报 GLSL 着色器
        #    链接失败 (indexed_8bit_image), 设初始位姿/拖动时卡死。
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            condition=IfCondition(rviz)
        )
    ])
