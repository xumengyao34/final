#!/usr/bin/env python3
"""定位错误检测 + 自动重定位 (reloc guard)。

背景: AMCL 定错位后 (scan 与地图对不上), 默认只靠 recovery_alpha_fast
随机撒粒子慢慢自愈; 本节点显式检测"定错位", 调 AMCL 的
/reinitialize_global_localization 服务 (粒子撒满自由空间重新收敛),
把自愈变成显式闭环, 与 config/amcl.yaml 里调大的 recovery_alpha_fast 配合。

打分方法 (每 score_period 拍取最新一帧 /scan):
  用 tf2 查 map -> base_scan (amcl_pose 的 TF + 静态 base_link -> base_scan),
  把光束沿射线步进投影到地图:
    - 射线在到达终点前 endpoint_margin 米内穿过占据格 -> 穿墙 (cut)。
      正确位姿下光束终点本来就落在墙上, 所以终点附近不判;
    - 雷达起点所在格本身是占据 -> 机器人在墙里 (单独算, 直接记 bad);
    - 短光束 (< min_beam_len) 不参与, 图外/未知格不算穿墙。
  正确位姿 + 地图与现场一致时 cut 比例应远小于 10%; 定错位时大量穿墙。
  滑动窗口 window_size 拍中 bad 比例 >= trigger_ratio -> 触发重定位,
  触发后冷却 cooldown 秒 (重定位后粒子重新收敛期间失配仍高, 防连环触发)。

能力边界: 只抓"位姿明显错位" (穿墙/进墙)。对称环境里 AMCL 收敛到另一个
"看起来也合理"的错误位姿时, 光束都能正常落在墙上, 本方法抓不到 ——
那种情况需要全局扫描匹配, 不在本节点范围内。

接线 (real_car.launch.py 已接):
  订阅 /scan (best effort) 和 /map (latch); 调 /reinitialize_global_localization
  (nav2_amcl 提供, std_srvs/Empty)。

参数 (启动时可覆盖):
  score_period      2.0   打分周期 (秒)
  max_beams         180   采样光束数 (720 个 bin 里等间隔抽)
  min_beam_len      1.0   短于该长度的光束不参与打分 (米)
  endpoint_margin   0.5   终点附近不判穿墙的距离 (米)
  occupied_thresh   65    占据格判定阈值 (0-100)
  cut_threshold     0.3   单拍穿墙比例超过即记 bad
  window_size       5     滑动窗口拍数
  trigger_ratio     0.8   窗口内 bad 占比超过即触发重定位
  cooldown          60.0  两次触发最小间隔 (秒)
  mismatch_topic    /reloc_mismatch  每拍失配比例 (Float32, 实车观测调参用)

注意: 本节点只调 AMCL 重定位服务, 不改位姿、不停导航; 阈值是首版猜测,
实车先看 /reloc_mismatch 和日志 (正常跑应 <0.1, 定错位应 >0.5) 再收紧/放宽。
"""
import math
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSProfile, ReliabilityPolicy, DurabilityPolicy,
                       HistoryPolicy)
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32
from std_srvs.srv import Empty
from tf2_ros import Buffer, TransformListener


def sensor_data_qos():
    """/scan 的订阅 QoS (best effort keep last 5), 与 livox_pc_to_scan 一致。"""
    return QoSProfile(
        depth=5,
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
        history=HistoryPolicy.KEEP_LAST)


class RelocGuard(Node):

    def __init__(self):
        super().__init__('reloc_guard')
        self.declare_parameter('score_period', 2.0)
        self.declare_parameter('max_beams', 180)
        self.declare_parameter('min_beam_len', 1.0)
        self.declare_parameter('endpoint_margin', 0.5)
        self.declare_parameter('occupied_thresh', 65)
        self.declare_parameter('cut_threshold', 0.3)
        self.declare_parameter('window_size', 5)
        self.declare_parameter('trigger_ratio', 0.8)
        self.declare_parameter('cooldown', 60.0)
        self.declare_parameter('mismatch_topic', '/reloc_mismatch')
        self.period = float(self.get_parameter('score_period').value)
        self.max_beams = int(self.get_parameter('max_beams').value)
        self.min_len = float(self.get_parameter('min_beam_len').value)
        self.margin = float(self.get_parameter('endpoint_margin').value)
        self.occ_th = int(self.get_parameter('occupied_thresh').value)
        self.cut_th = float(self.get_parameter('cut_threshold').value)
        self.win_n = int(self.get_parameter('window_size').value)
        self.trig_ratio = float(self.get_parameter('trigger_ratio').value)
        self.cooldown = float(self.get_parameter('cooldown').value)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_subscription(
            LaserScan, '/scan', self.on_scan, sensor_data_qos())
        # /map 是 latch 的 (map_server transient_local), 迟订阅也能拿到
        map_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(OccupancyGrid, '/map', self.on_map, map_qos)
        self.mismatch_pub = self.create_publisher(
            Float32, self.get_parameter('mismatch_topic').value, 10)
        self.reinit_cli = self.create_client(
            Empty, '/reinitialize_global_localization')

        self.map = None
        self.last_scan = None
        self.scores = deque(maxlen=self.win_n)
        self.last_trigger = None
        self.create_timer(self.period, self.tick)
        self.get_logger().info(
            f'reloc guard 启动: 每{self.period}s打分 max_beams={self.max_beams} '
            f'cut_th={self.cut_th} window={self.win_n} '
            f'trigger={self.trig_ratio} cooldown={self.cooldown}s')

    def on_scan(self, msg):
        self.last_scan = msg

    def on_map(self, msg):
        self.map = msg

    def cell_value(self, wx, wy):
        """世界坐标 -> 占据格值; -2 表示图外。"""
        info = self.map.info
        cx = int((wx - info.origin.position.x) / info.resolution)
        cy = int((wy - info.origin.position.y) / info.resolution)
        if cx < 0 or cx >= info.width or cy < 0 or cy >= info.height:
            return -2
        return self.map.data[cy * info.width + cx]

    def ray_cuts(self, sx, sy, ex, ey):
        """光束在到达终点 margin 米之前是否穿过占据格。"""
        dist = math.hypot(ex - sx, ey - sy)
        if dist <= self.margin:
            return False
        t_lim = (dist - self.margin) / dist
        res = self.map.info.resolution
        steps = max(1, int(dist / (res * 0.5)))
        for k in range(1, steps):
            t = k / steps
            if t > t_lim:
                break
            if self.cell_value(sx + (ex - sx) * t,
                               sy + (ey - sy) * t) > self.occ_th:
                return True
        return False

    def score(self, scan):
        """返回 (cut_ratio, origin_in_wall); 有效光束不足时返回 (None, False)。"""
        t = self.tf_buffer.lookup_transform(
            'map', 'base_scan', scan.header.stamp,
            timeout=rclpy.duration.Duration(seconds=0.5))
        tx = t.transform.translation.x
        ty = t.transform.translation.y
        q = t.transform.rotation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))

        n = len(scan.ranges)
        step = max(1, n // self.max_beams)
        cut = 0
        valid = 0
        for i in range(0, n, step):
            r = scan.ranges[i]
            if not math.isfinite(r) or r < self.min_len or r > scan.range_max:
                continue
            ang = scan.angle_min + i * scan.angle_increment
            ex = tx + r * math.cos(yaw + ang)
            ey = ty + r * math.sin(yaw + ang)
            valid += 1
            if self.ray_cuts(tx, ty, ex, ey):
                cut += 1
        if valid < 10:
            return None, False
        origin_wall = self.cell_value(tx, ty) > self.occ_th
        return cut / valid, origin_wall

    def trigger(self):
        if not self.reinit_cli.service_is_ready():
            self.get_logger().warn(
                '重定位服务未就绪 (AMCL 没起?), 跳过本次调用')
            return
        self.last_trigger = self.get_clock().now()
        self.get_logger().error(
            f'[定位错误] 近 {self.win_n} 拍 {sum(self.scores)} 拍失配, '
            f'调 /reinitialize_global_localization 全局重定位 '
            f'(冷却 {self.cooldown}s, 粒子重新收敛前失配仍高属正常)')
        self.reinit_cli.call_async(Empty.Request())

    def tick(self):
        scan = self.last_scan
        if self.map is None or scan is None:
            return
        now = self.get_clock().now()
        stamp = rclpy.time.Time.from_msg(scan.header.stamp)
        if (now - stamp).nanoseconds * 1e-9 > 3.0 * self.period:
            self.get_logger().warn(
                '/scan 数据过期, 跳过打分', throttle_duration_sec=10.0)
            return
        try:
            ratio, origin_wall = self.score(scan)
        except Exception as e:
            self.get_logger().warn(
                f'打分失败 (TF 查询超时?): {e}', throttle_duration_sec=5.0)
            return
        if ratio is None:
            self.get_logger().warn(
                '有效光束不足, 跳过打分', throttle_duration_sec=10.0)
            return

        msg = Float32()
        msg.data = ratio
        self.mismatch_pub.publish(msg)

        bad = origin_wall or ratio >= self.cut_th
        self.scores.append(bad)
        self.get_logger().info(
            f'[打分] 穿墙比={ratio:.2f} (阈{self.cut_th}) '
            f'原点在墙内={origin_wall} 窗口bad={sum(self.scores)}/{len(self.scores)}')
        if self.last_trigger is not None and \
                (now - self.last_trigger).nanoseconds * 1e-9 < self.cooldown:
            return
        if len(self.scores) < self.win_n or \
                sum(self.scores) / len(self.scores) < self.trig_ratio:
            return
        self.trigger()


def main():
    rclpy.init()
    node = RelocGuard()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
