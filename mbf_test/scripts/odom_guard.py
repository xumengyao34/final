#!/usr/bin/env python3
"""FAST-LIO 里程计速度门 (odom guard): 物理可行性检查 + 冻结/告警。

接线 (real_car.launch.py 已配好):
  fastlio_mapping 的 /Odometry 和 /tf 被重映射为 /Odometry_raw /tf_raw,
  本节点检查后把"可信"的位姿重新发成 /Odometry + TF odom -> base_link,
  下游 (AMCL、代价地图、MBF、RViz) 只看得到过滤后的流。
  [2026-10-30 TF 简化] body 帧已从 TF 树删除: FAST-LIO 的 body 系
  (水平 IMU 系, 轴与 base_link 对齐, 见 livox_imu_level.py 的 level_yaw)
  位姿在本节点合成到 base_link —— 位置加固定 IMU 偏移、姿态四元数不变:
    pos_base = pos_body + q · offset, offset = lidar_pos - R·lidar_ext
  其中 R 与 livox_imu_level.py / mid360.yaml 的安装矩阵一致 (同一组 launch
  参数传入), offset ≈ (0.136, -0.011, 0.202) = IMU 在 base_link 系中的位置。

检查逻辑 (里程计 10Hz, 默认参数按此调):
  每帧与"上一帧已接受位姿"比较, 线速度 v=位移/dt、角速度 w=|yaw差|/dt,
  任一超阈值即违规:
    - 丢弃该帧, 用上一帧好位姿 + 当前时间戳重发 (下游看到"车没动"而不是
      "车瞬移"), TF 同步重发保持新鲜 (TF 断更会让下游查询超时);
    - 连续违规 trigger_count 帧 -> /odom_anomaly 置 True (latch);
    - 重新锚定 (下面两条路): 冻结期间 dt 相对最后接受帧一直在涨, 涨过
      max_dt 或冻结超过 unfreeze_time 都说明"一直没收到可信位姿", 此时
      强制接受当前帧 (防永久冻结) 并保持告警, 恢复稳定 clear_time 后解除。
      重新锚定后若 FAST-LIO 缓慢漂移 (每帧增量低于阈值), 速度门无法与真实
      运动区分 —— 这是速度门的能力边界, 缓慢漂移靠 AMCL 激光匹配压。
  断流重连: 未冻结时 dt<=0 或 dt>max_dt 视为流重启 (fast_lio 重启/时间戳
  回跳), 直接接受、重置基线, 告警状态不受影响。
  /odom_anomaly (std_msgs/Bool) 目前无下游消费, 留作导航恢复行为挂接点
  (检测到跳变 -> 暂停导航/重定位)。

参数 (启动时可覆盖, 见 real_car.launch.py):
  max_lin_vel     3.0   线速度上限 m/s (10Hz 下 = 0.3m/帧, 与 monitor_lio
                        的 jump_th 一致; 正常行驶 << 此值, 发散跳变远大于此值)
  max_ang_vel     6.0   角速度上限 rad/s (脱困原地旋转一般 <=3 rad/s)
  trigger_count   3     连续违规多少帧才报异常
  unfreeze_time   5.0   冻结多久后强制重新锚定 (秒)
  clear_time      10.0  连续稳定多久后解除告警 (秒)
  max_dt          1.0   两帧间隔超过此值视为断流重连, 直接接受 (秒)

注意: 本节点死掉会让 /Odometry 和 /tf 断流 (fail-closed), monitor_lio 的
心跳日志里能看到位置停更, 实车排查时留意。
"""
import math

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from rclpy.qos import QoSProfile, DurabilityPolicy, HistoryPolicy


def quat_to_yaw(q):
    """四元数 (x,y,z,w) -> yaw (绕 z, rad)"""
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap_pi(a):
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


def quat_rotate(q, v):
    """四元数 q (x,y,z,w) 旋转向量 v -> (vx, vy, vz)"""
    qx, qy, qz, qw = q
    # t = 2·q_xyz × v; v' = v + qw·t + q_xyz × t
    tx = 2.0 * (qy * v[2] - qz * v[1])
    ty = 2.0 * (qz * v[0] - qx * v[2])
    tz = 2.0 * (qx * v[1] - qy * v[0])
    return (v[0] + qw * tx + (qy * tz - qz * ty),
            v[1] + qw * ty + (qz * tx - qx * tz),
            v[2] + qw * tz + (qx * ty - qy * tx))


def rpy_to_matrix(roll, pitch, yaw):
    """R = Rz(yaw)·Ry(pitch)·Rx(roll): 雷达系 -> base 系 (与 livox_imu_level.py
    及 mid360.yaml 的安装矩阵完全一致, 改安装姿态三处必须同步)"""
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return [[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp,     cp * sr,                cp * cr]]


class OdomGuard(Node):

    def __init__(self):
        super().__init__('odom_guard')
        self.declare_parameter('max_lin_vel', 3.0)
        self.declare_parameter('max_ang_vel', 6.0)
        self.declare_parameter('trigger_count', 3)
        self.declare_parameter('unfreeze_time', 5.0)
        self.declare_parameter('clear_time', 10.0)
        self.declare_parameter('max_dt', 1.0)
        self.max_lin = float(self.get_parameter('max_lin_vel').value)
        self.max_ang = float(self.get_parameter('max_ang_vel').value)
        self.trigger_n = int(self.get_parameter('trigger_count').value)
        self.unfreeze_t = float(self.get_parameter('unfreeze_time').value)
        self.clear_t = float(self.get_parameter('clear_time').value)
        self.max_dt = float(self.get_parameter('max_dt').value)

        # [2026-10-30 TF 简化] 安装外参: level_* 由 launch 从 lidar_extrinsic.py 传入
        self.declare_parameter('level_roll')   # MID360 绕雷达 x 轴斜装角
        self.declare_parameter('level_pitch')
        self.declare_parameter('level_yaw')   # 雷达 +x 朝车右 -> -90°
        self.declare_parameter('lidar_pos', [0.15, 0.0, 0.25])       # 雷达在 base_link 系位置
        # 出厂外参 (Livox 语义 = lidar 在 IMU 系中的位置, 与 FAST-LIO 的
        # Lidar_T_wrt_IMU 一致; 旧注释说成 "IMU 在 lidar 系" 是反的)
        self.declare_parameter('lidar_ext', [-0.011, -0.02329, 0.04412])
        r = float(self.get_parameter('level_roll').value)
        p = float(self.get_parameter('level_pitch').value)
        y = float(self.get_parameter('level_yaw').value)
        t1 = [float(v) for v in self.get_parameter('lidar_pos').value]
        ext = [float(v) for v in self.get_parameter('lidar_ext').value]
        R = rpy_to_matrix(r, p, y)
        # offset = t1 - R·ext = IMU 在 base_link 系中的位置 (≈ (0.136, -0.011, 0.202))
        self.offset = [t1[i] - sum(R[i][j] * ext[j] for j in range(3))
                       for i in range(3)]
        self.get_logger().info(
            f'odom 速度门启动: v_max={self.max_lin}m/s w_max={self.max_ang}rad/s '
            f'trigger={self.trigger_n}帧 unfreeze={self.unfreeze_t}s '
            f'clear={self.clear_t}s | IMU 偏移=({self.offset[0]:.3f}, '
            f'{self.offset[1]:.3f}, {self.offset[2]:.3f}) '
            f'(发布 odom -> base_link)')

        # 默认 QoS (reliable): fast_lio 的 /Odometry 是默认 QoS 发布
        self.sub = self.create_subscription(
            Odometry, '/Odometry_raw', self.on_odom, 10)
        self.pub = self.create_publisher(Odometry, '/Odometry', 10)
        self.tf_br = TransformBroadcaster(self)
        # latch: 迟订阅者也能拿到当前告警状态
        latch_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                               history=HistoryPolicy.KEEP_LAST)
        self.anomaly_pub = self.create_publisher(Bool, '/odom_anomaly', latch_qos)

        # 上一帧已接受: (t, x, y, z, yaw, qx, qy, qz, qw, cov)
        self.last = None
        self.streak = 0           # 连续违规帧数
        self.frozen_at = None     # 本次冻结起始 (monotonic 秒)
        self.anomaly = False      # 当前告警状态
        self.last_bad_at = None   # 最近一次违规时刻 (monotonic 秒)
        self.get_logger().info(
            f'odom 速度门启动: v_max={self.max_lin}m/s w_max={self.max_ang}rad/s '
            f'trigger={self.trigger_n}帧 unfreeze={self.unfreeze_t}s '
            f'clear={self.clear_t}s')

    def _publish(self, msg, pos, quat, cov):
        """把给定位姿合成到 base_link 后重发 /Odometry + TF (冻结时用)。

        [2026-10-30 TF 简化] 入参 pos/quat 是 FAST-LIO 的 body 系位姿
        (odom -> body), 这里加固定 IMU 偏移合成 odom -> base_link
        (pos_base = pos_body + q·offset, 姿态不变)。

        [2026-10-27] stamp 统一用主机当前时间, 不再沿用 FAST-LIO 的雷达设备
        时间戳 (PTP 服务在 USB 网卡上起不来, 设备时间自由漂移 10s+, 与 AMCL
        的 now 混在同一棵 TF 树里 -> costmap 查 TF 全失败不出图 / 障碍层丢帧)。
        """
        ox, oy, oz = quat_rotate(quat, self.offset)
        px, py, pz = pos[0] + ox, pos[1] + oy, pos[2] + oz
        out = Odometry()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = 'odom'
        out.child_frame_id = 'base_link'
        out.pose.pose.position.x = px
        out.pose.pose.position.y = py
        out.pose.pose.position.z = pz
        out.pose.pose.orientation.x = quat[0]
        out.pose.pose.orientation.y = quat[1]
        out.pose.pose.orientation.z = quat[2]
        out.pose.pose.orientation.w = quat[3]
        out.pose.covariance = cov
        # 冻结期间不转发速度 (下游把"没动"当事实即可)
        self.pub.publish(out)

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()   # 同上, 统一主机时间
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = px
        t.transform.translation.y = py
        t.transform.translation.z = pz
        t.transform.rotation.x = quat[0]
        t.transform.rotation.y = quat[1]
        t.transform.rotation.z = quat[2]
        t.transform.rotation.w = quat[3]
        self.tf_br.sendTransform(t)

    def _set_anomaly(self, on):
        if on == self.anomaly:
            return
        self.anomaly = on
        msg = Bool()
        msg.data = on
        self.anomaly_pub.publish(msg)
        if on:
            self.get_logger().error(
                f'[odom 异常] 连续 {self.trigger_n} 帧超速, /odom_anomaly=True, '
                f'位姿已冻结 (挂接点: 暂停导航/重定位)')
        else:
            self.get_logger().info('[odom 恢复] 持续稳定, /odom_anomaly=False')

    def on_odom(self, msg):
        t = rclpy.time.Time.from_msg(msg.header.stamp).nanoseconds * 1e-9
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = quat_to_yaw(q)
        cov = list(msg.pose.covariance)
        now = self.get_clock().now().nanoseconds * 1e-9

        # 告警已挂起且持续稳定 -> 解除
        if self.anomaly and self.last_bad_at is not None and \
                now - self.last_bad_at > self.clear_t:
            self._set_anomaly(False)

        # 第一帧 / 断流重连: 无条件接受
        if self.last is None:
            self.last = (t, p.x, p.y, p.z, yaw,
                         q.x, q.y, q.z, q.w, cov)
            self._publish(msg, (p.x, p.y, p.z),
                          (q.x, q.y, q.z, q.w), cov)
            return
        t0, x0, y0, z0, yaw0, *_ = self.last
        dt = t - t0
        if dt <= 0.0 or dt > self.max_dt:
            # 断流重连(未冻结) 或 重新锚定(冻结中, dt 已涨过 max_dt)。
            # 两条路都必须接受(冻结期间继续逐帧比较会永远拒绝, 系统卡死),
            # 告警由稳定性定时器解除, 这里不动。
            if self.anomaly or self.frozen_at is not None:
                self.get_logger().error(
                    f'[强制重新锚定] dt={dt:.3f}s 超 max_dt, 接受当前位姿 '
                    f'({p.x:.2f},{p.y:.2f},{p.z:.2f}) 并保持告警')
            else:
                self.get_logger().warn(
                    f'[断流重连] dt={dt:.3f}s, 跳过检查直接接受 (t={t:.3f})')
            self.streak = 0
            self.frozen_at = None
            self.last = (t, p.x, p.y, p.z, yaw,
                         q.x, q.y, q.z, q.w, cov)
            self._publish(msg, (p.x, p.y, p.z),
                          (q.x, q.y, q.z, q.w), cov)
            return

        # 冻结超时 (时间戳持续更新但始终不合规时的兜底, 防永久冻结)
        if self.frozen_at is not None and now - self.frozen_at > self.unfreeze_t:
            self.get_logger().error(
                f'[强制重新锚定] 冻结超 {self.unfreeze_t}s, 接受当前位姿 '
                f'({p.x:.2f},{p.y:.2f},{p.z:.2f}) 并保持告警')
            self.streak = 0
            self.frozen_at = None
            self.last = (t, p.x, p.y, p.z, yaw,
                         q.x, q.y, q.z, q.w, cov)
            self._publish(msg, (p.x, p.y, p.z),
                          (q.x, q.y, q.z, q.w), cov)
            return

        d = math.sqrt((p.x - x0) ** 2 + (p.y - y0) ** 2 + (p.z - z0) ** 2)
        v = d / dt
        w = abs(wrap_pi(yaw - yaw0)) / dt

        if v <= self.max_lin and w <= self.max_ang:
            # 通过: 正常转发
            self.streak = 0
            self.frozen_at = None
            self.last = (t, p.x, p.y, p.z, yaw,
                         q.x, q.y, q.z, q.w, cov)
            self._publish(msg, (p.x, p.y, p.z),
                          (q.x, q.y, q.z, q.w), cov)
            return

        # 违规: 丢弃该帧, 用上一帧好位姿重发 (冻结)
        self.streak += 1
        self.last_bad_at = now
        if self.frozen_at is None:
            self.frozen_at = now
        self.get_logger().warn(
            f'[违规{self.streak}] dt={dt * 1000:.0f}ms v={v:.2f}m/s '
            f'(限{self.max_lin}) w={w:.2f}rad/s (限{self.max_ang}) '
            f'位移={d:.2f}m, 已冻结在 ({x0:.2f},{y0:.2f},{z0:.2f})',
            throttle_duration_sec=1.0)
        if self.streak >= self.trigger_n:
            self._set_anomaly(True)
        self._publish(msg, (x0, y0, z0), self.last[5:9], self.last[9])


def main():
    rclpy.init()
    node = OdomGuard()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
