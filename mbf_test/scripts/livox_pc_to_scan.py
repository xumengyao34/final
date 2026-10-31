#!/usr/bin/env python3
"""MID360 点云 -> 2D LaserScan, 供 AMCL 和代价地图使用。

雷达斜装 44.2° (roll=-0.772) 且
装在车头边缘中心上方 0.25m, 安装外参全部在静态 TF 里:
  1. 先用 TF 把点云从 livox_frame 转到 base_scan
     (base_scan = 雷达安装位置的"水平无倾角"虚拟坐标系,
      x 轴 = 车头方向 = 雷达端, 由 base_link -> base_scan 静态 TF 定义);
  2. 转完后点云已把安装倾角修正掉, 按地面射线滤波剔除 (每个点与地面模型
     z_floor(r) 比较, 离地不足 margin 的点是地面, 高于 max_z 的是天花板
     回波, 两者都剔; 等价于沿每条射线取第一个离地点);
  3. 按水平角度分桶投影成 2D scan, 同角度取最近点。

"""
import math

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSProfile, ReliabilityPolicy, DurabilityPolicy,
                       HistoryPolicy)
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener

try:
    from livox_ros_driver2.msg import CustomMsg
except ImportError:
    CustomMsg = None  # 运行时在 __init__ 里报错提示, 避免 import 阶段直接崩


def sensor_data_qos():
    """每次调用返回一份全新的 sensor_data QoS (best effort, keep last 5),
    与 livox_ros_driver2 的发布端和 nav2 AMCL 的订阅端保持一致。"""
    return QoSProfile(
        depth=5,
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
        history=HistoryPolicy.KEEP_LAST)


def quat_to_rot(q):
    """tf2 四元数 (x, y, z, w) -> 3x3 旋转矩阵 (手写, 不依赖 tf_transformations)。"""
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ], dtype=np.float64)


def parse_xyz(cloud):
    """从 livox CustomMsg 取 x/y/z, 返回 (N,3) float64。"""
    if len(cloud.points) == 0:
        return np.empty((0, 3), dtype=np.float64)
    return np.array([[p.x, p.y, p.z] for p in cloud.points], dtype=np.float64)


class LivoxToScan(Node):
    def __init__(self):
        super().__init__('livox_pc_to_scan')

        self.declare_parameter('cloud_topic', '/livox/lidar')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('target_frame', 'base_scan')
        # 地面射线滤波在 base_scan 系(水平)下做: 每个点与地面模型
        # z_floor(r) = ground_z + ground_slope*r 比较, 离地超过 ground_margin
        # 才保留 (等价于"沿每条射线取第一个离地点"), 高于 max_z 的剔除。
        #   理论计算, fov_blindspot_analysis.py] max_z 1.8 -> 2.5:
        #   车前墙/家具的回波 z = 0.759*r, max_z=1.8 时车前 2.37m 外全滤掉
        #   (代价地图车前 2.4m 外是空的), 2.5 时墙可见到 3.3m。大厅天花板
        #   3m: 37.2° 束打天花板 z=3.0 在 3.95m 处, 仍 >2.5 被滤, 不产生
        #   幻影障碍; 头顶有低于 2.6m 横梁/灯具的场地要相应调小。
        #   注意: 1.7m 的人在车前 1.9m 外仍不可见 —— 是物理极限
        #   (最低光束 37.2° 越过人头)
        self.declare_parameter('ground_z', -0.25)
        self.declare_parameter('ground_margin', 0.10)
        self.declare_parameter('ground_slope', 0.0)  # 坡道场地按实测填 (z 随 r 的坡度)
        self.declare_parameter('max_z', 2.5)
        self.declare_parameter('range_min', 0.15)
        self.declare_parameter('range_max', 15.0)
        self.declare_parameter('angle_resolution', 0.5)  # 度, 720 个 bin
        # 车体足迹滤波 (base_scan 系水平面 xy): 车体 0.3x0.3m 正方形,
        # base_scan 原点在车头边缘中点 -> 车体占 x∈[-0.3,0], y∈[-0.15,0.15],
        # 各加 0.05 余量。雷达装车头, 向后看必然扫到车体; 万向轮旋转/横移时
        # 车体反射点随车转, 是 AMCL 旋转失配的干扰源 
        self.declare_parameter('footprint_x_min', -0.35)
        self.declare_parameter('footprint_x_max', 0.05)
        self.declare_parameter('footprint_y_max', 0.2)

        self.target_frame = self.get_parameter('target_frame').value
        self.ground_z = self.get_parameter('ground_z').value
        self.ground_margin = self.get_parameter('ground_margin').value
        self.ground_slope = self.get_parameter('ground_slope').value
        self.max_z = self.get_parameter('max_z').value
        self.range_min = self.get_parameter('range_min').value
        self.range_max = self.get_parameter('range_max').value
        self.foot_x_min = self.get_parameter('footprint_x_min').value
        self.foot_x_max = self.get_parameter('footprint_x_max').value
        self.foot_y_max = self.get_parameter('footprint_y_max').value
        self.n_bins = int(360.0 / self.get_parameter('angle_resolution').value)
        self.angle_increment = 2.0 * math.pi / self.n_bins

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        if CustomMsg is None:
            self.get_logger().error(
                'livox_ros_driver2 未安装: 需要先 source ~/final/install/setup.bash '
                '(本节点订阅的是 CustomMsg 类型)')
            return
        # 驱动用 xfer_format=4 发 CustomMsg; QoS 用 sensor_data (best effort),
        # 与 AMCL 订阅 /scan 的口径一致 
        self.create_subscription(
            CustomMsg,
            self.get_parameter('cloud_topic').value,
            self.cloud_callback,
            sensor_data_qos())
        self.scan_pub = self.create_publisher(
            LaserScan, self.get_parameter('scan_topic').value, sensor_data_qos())
        self.get_logger().info(
            f'点云 -> LaserScan: {self.get_parameter("cloud_topic").value} '
            f'-> {self.get_parameter("scan_topic").value} (frame: {self.target_frame})')

        # 1Hz 心跳: 收/发计数, 上实车后靠它确认数据流是否正常
        self.recv_count = 0
        self.pub_count = 0
        self.create_timer(1.0, self.heartbeat)

    def heartbeat(self):
        self.get_logger().info(
            f'alive: recv={self.recv_count} pub={self.pub_count}')

    def cloud_callback(self, cloud):
        self.recv_count += 1
        try:
            t = self.tf_buffer.lookup_transform(
                self.target_frame, cloud.header.frame_id, cloud.header.stamp,
                timeout=rclpy.duration.Duration(seconds=0.1))
        except Exception as e:
            self.get_logger().warn(
                f'TF 查询失败 (livox_frame -> {self.target_frame}): {e}',
                throttle_duration_sec=5.0)
            return

        try:
            pts = parse_xyz(cloud)
        except Exception as e:
            self.get_logger().warn(f'点云解析失败: {e}', throttle_duration_sec=5.0)
            return
        if pts.shape[0] == 0:
            return

        # 变换到 target_frame: 安装位置与倾斜角都包含在 TF 里
        rot = quat_to_rot(t.transform.rotation)
        tr = np.array([t.transform.translation.x,
                       t.transform.translation.y,
                       t.transform.translation.z])
        p = rot @ pts.T + tr[:, None]  # (3, N)

        # 地面射线滤波 (水平系下): 每个点与地面模型 z_floor(r) 比较,
        # 离地超过 ground_margin 才保留 —— 等价于沿每条射线取第一个离地点
        # (贴地点全部跳过, 近处地面环不会淹没远处真障碍); 高于 max_z 的
        # 点 (天花板/横梁回波) 剔除。
        r = np.hypot(p[0], p[1])
        z_floor = self.ground_z + self.ground_slope * r
        mask = (p[2] > z_floor + self.ground_margin) & (p[2] < self.max_z)
        # 车体足迹过滤: 剔除落入车体矩形 (水平面) 的点 (车体反射, 见上)
        mask &= ~((p[0] > self.foot_x_min) & (p[0] < self.foot_x_max)
                  & (np.abs(p[1]) < self.foot_y_max))
        x, y, r = p[0, mask], p[1, mask], r[mask]
        if x.size == 0:
            return

        valid = (r > self.range_min) & (r < self.range_max)
        x, y, r = x[valid], y[valid], r[valid]
        if x.size == 0:
            return

        # 按角度分桶, 同角度取最近点
        ang = np.arctan2(y, x)
        idx = np.clip(((ang + math.pi) / (2.0 * math.pi) * self.n_bins).astype(int),
                      0, self.n_bins - 1)
        ranges = np.full(self.n_bins, np.inf, dtype=np.float32)
        np.minimum.at(ranges, idx, r.astype(np.float32))

        scan = LaserScan()
        # [2026-10-27] 不用雷达设备时间戳 (PTP 起不来时与主机时钟漂移 10s+,
        # AMCL/costmap 的 TF 全用主机时间 -> message filter 丢帧, 障碍层失效)。
        # 改主机当前时间, 与下游 TF 树统一
        scan.header.stamp = self.get_clock().now().to_msg()
        scan.header.frame_id = self.target_frame
        scan.angle_min = -math.pi
        scan.angle_max = math.pi
        scan.angle_increment = self.angle_increment
        scan.time_increment = 0.0
        scan.range_min = self.range_min
        scan.range_max = self.range_max
        scan.ranges = ranges.tolist()
        self.scan_pub.publish(scan)
        self.pub_count += 1


def main(args=None):
    rclpy.init(args=args)
    node = LivoxToScan()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
