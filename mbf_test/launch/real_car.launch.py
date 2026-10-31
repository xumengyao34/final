import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

sys.path.insert(0, os.path.join(
    get_package_share_directory('mbf_test'), 'launch'))
from lidar_extrinsic import ROLL, PITCH, YAW


def generate_launch_description():
    """实车一键启动: Nav2 + MBF (mbf_demo) + FAST-LIO里程计 + MID360点云转scan + 串口桥。
    (/Odometry_raw + TF_raw, camera_init -> body 原始流), odom_guard 检查后
    合成 /Odometry + TF odom -> base_link
    """

    map_yaml = LaunchConfiguration('map')
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    rviz = LaunchConfiguration('rviz', default='true')
    serial_port = LaunchConfiguration('serial_port', default='/dev/ttyACM0')
    baudrate = LaunchConfiguration('baudrate', default='115200')
    chassis_angle = LaunchConfiguration('chassis_x_angle_deg', default='90.0')

    mbf_demo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('mbf_test'),
                         'launch', 'mbf_demo.launch.py')),
        launch_arguments={
            'map': map_yaml,
            'use_sim_time': use_sim_time,
            'rviz': rviz,
        }.items(),
    )

    # /livox/imu 旋平+偏航对齐 -> /imu_level 
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

    # FAST-LIO 激光里程计: /livox/lidar + /imu_level -> /Odometry_raw + TF_raw
    # 输出先走 odom_guard 速度门: 由 odom_guard 检查后合成 odom -> base_link
    fast_lio = Node(
        package='fast_lio',
        executable='fastlio_mapping',
        name='fastlio_mapping',
        output='screen',
        remappings=[
            ('/Odometry', '/Odometry_raw'),
            ('/tf', '/tf_raw'),
        ],
        parameters=[
            os.path.join(get_package_share_directory('fast_lio'),
                         'config', 'mid360.yaml'),
            {'use_sim_time': use_sim_time},
        ],
    )

    # odom 速度门: /Odometry_raw -> 超速检查 -> /Odometry + TF odom -> base_link
    # 超速跳变时冻结在上一帧好位姿并置 /odom_anomaly, 见 scripts/odom_guard.py;
    odom_guard = Node(
        package='mbf_test',
        executable='odom_guard.py',
        name='odom_guard',
        output='screen',
        parameters=[{
            'max_lin_vel': 3.0,   # 10Hz 下 = 0.3m/帧, 与 monitor_lio jump_th 一致
            'max_ang_vel': 6.0,  
            'level_roll': ROLL,
            'level_pitch': PITCH,
            'level_yaw': YAW,
        }],
    )

    # MID360 点云 -> /scan (LaserScan), 供 AMCL 和代价地图使用
    livox_pc_to_scan = Node(
        package='mbf_test',
        executable='livox_pc_to_scan.py',
        name='livox_pc_to_scan',
        output='screen',
    )

    # LIO 健康监测: /Odometry 跳变/发散检测 + /scan /imu_level /amcl_pose 心跳
    # 监测原始流: odom_guard 会滤掉跳变, 看 raw 才能还原发散现场
    monitor_lio = Node(
        package='mbf_test',
        executable='monitor_lio.py',
        name='monitor_lio',
        output='screen',
        remappings=[('/Odometry', '/Odometry_raw')],
    )

    # 单目标导航 (RViz "Nav2 Goal" 按钮 / bt_navigator) 的堵路自动重规划:
    # 障碍静止 5s -> 重发 /goal_pose 让 bt_navigator 重新规划绕行。
    blocked_replan = Node(
        package='mbf_test',
        executable='blocked_replan.py',
        name='blocked_replan',
        output='screen',
    )

    # 定位错误检测 -> 自动全局重定位: 把 /scan 投影到地图统计穿墙比例,
    # 连续失配 -> 调 AMCL 的 /reinitialize_global_localization (全局重定位)。
    # [2026-10-25 已禁用] 实车定位不准, 触发后每次都全局撒粒子重定位到
    # 错误的位置; 阈值未调好前先注释, 需要观测时看 /reloc_mismatch。
    # reloc_guard = Node(
    #     package='mbf_test',
    #     executable='reloc_guard.py',
    #     name='reloc_guard',
    #     output='screen',
    # )

    # AMCL 初始位姿助手 (scripts/amcl_auto_init.py): initialpose 中继放大
    # yaw 协方差 + 无初始位姿自动全局定位。
    # [2026-10-25 晚实车禁用] relay 的 create_timer 是周期定时器, 每次点
    # RViz 后每 0.5s 永久重撒初始位姿 (20:40 场 AMCL 收到 510 次
    # /set_initial_pose): TF 被钉在点击位姿不动 (车在走 TF 不动的元凶)、
    # σ45° 散开在稀疏地图里反复收敛到旧位姿 (TF 反复跳的元凶)、定位
    # 不准、全局规划不出新路径。
    # amcl_auto_init = Node(
    #     package='mbf_test',
    #     executable='amcl_auto_init.py',
    #     name='amcl_auto_init',
    #     output='screen',
    # )

    # /cmd_vel (Twist, Humble 的 velocity_smoother 平滑输出) -> 底盘串口协议帧
    serial_bridge = Node(
        package='serial_bridge_pkg',
        executable='serial_bridge',
        name='cmd_vel_to_serial',
        output='screen',
        parameters=[{
            'port': serial_port,
            'baudrate': baudrate,
            'use_stamped': False,
            # 参数定义: 底盘 vx 轴在 base_link 中的方向角 (CCW 为正)。
            # 底盘 vx 轴物理指向 = 车左 = base +y -> +90° 。c = Rz(-angle)·v:
            # 下地实测方向不对就翻 chassis_x_angle_deg
            'chassis_x_angle_deg': chassis_angle,
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'map', default_value='/home/xmy/final/src/mbf_test/maps/now_map.yaml',
            description='地图 yaml 的绝对路径'),
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='是否使用仿真时钟 (实车为 false)'),
        DeclareLaunchArgument(
            'rviz', default_value='true',
            description='是否启动 RViz2'),
        DeclareLaunchArgument(
            'serial_port', default_value='/dev/ttyACM0',
            description='底盘串口设备号'),
        DeclareLaunchArgument(
            'baudrate', default_value='115200',
            description='底盘串口波特率 (与电控一致)'),
        DeclareLaunchArgument(
            'chassis_x_angle_deg', default_value='90.0',
            description='底盘 vx 轴在 base_link 中的方向角 (度, CCW 为正)。'
                        '约定有雷达端=车头后, 底盘 vx 轴指向车左 = base +y -> +90°; '
                        '下地实测前进方向不对时翻成 -90.0 试'),

        mbf_demo,
        imu_level,
        fast_lio,
        odom_guard,
        livox_pc_to_scan,
        monitor_lio,
        blocked_replan,
        # reloc_guard,
        # amcl_auto_init,
        serial_bridge,
    ])
