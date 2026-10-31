#!/usr/bin/env python3
"""AMCL 初始位姿助手: 初始位姿给斜了自动回正 + 没给初始位姿自动全局定位。

背景 (2026-10-25 实车):
     旧 reloc_guard 教训: 全图撒粒子会收敛到"莫名其妙的位置" (空旷对称
     环境里粒子收敛到错位姿, 穿墙率也低, 校验不到)。所以本节点撒粒子后
     必须用协方差做收敛校验, 重试有上限, 不乱飞。

两个功能 (各自独立开关):
  A. initialpose 中继放大 yaw 协方差 (默认开):
     订阅 /initialpose, 延迟 relay_delay 秒后调 /set_initial_pose, 把 yaw
     协方差放大到 yaw_sigma (默认 45°), 位置协方差保持原值。粒子云覆盖
     ±90°, scan 匹配在有特征处把 yaw 拉正。AMCL 直接收 RViz 的原始
     initialpose 在先, 放大版在后覆盖它。
  B. 无初始位姿自动全局定位 (默认开):
     启动后等 init_wait 秒: 没收到 /amcl_pose 也没人给 initialpose
     (中继没转发过) -> 调 /reinitialize_global_localization 全图撒粒子。
     之后每 check_period 秒看 /amcl_pose 协方差 (粒子云整体协方差,
     nav2_amcl publishAmclPose 填的是 set->cov, 见源码注释 "Report the
     overall filter covariance"):
       连续 conv_checks 拍 cov[0] < conv_x_var 且 cov[35] < conv_yaw_var
       -> 收敛成功;
       check_timeout 秒内没收敛 -> 重撒, 最多 max_retries 次;
       全部失败 -> 打错误日志保持现状 (不无限撒, 避免 reloc_guard 教训)。
     协方差校验只能挡"没收敛", 挡不住"收敛到错误位姿"——空旷对称环境里
     撒粒子可能收敛错且校验不出 (与 reloc_guard 同一能力边界)。测试区
     地图特征稀疏, 建议空旷处放几个纸箱再依赖本功能。
     观测: /amcl_cov (Float32MultiArray, [x方差, yaw方差]) 用于调阈值。

  订阅 /initialpose (RViz 2D Pose Estimate) 和 /amcl_pose; 调
  /set_initial_pose (nav2_msgs/srv/SetInitialPose) 和
  /reinitialize_global_localization (std_srvs/srv/Empty), 均为
  nav2_amcl 原生服务。
"""
import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import Float32MultiArray
from std_srvs.srv import Empty
from nav2_msgs.srv import SetInitialPose


class AmclAutoInit(Node):

    def __init__(self):
        super().__init__('amcl_auto_init')
        self.declare_parameter('relay_enabled', True)
        self.declare_parameter('relay_delay', 0.5)
        self.declare_parameter('yaw_sigma', math.radians(45.0))
        self.declare_parameter('auto_global_reloc', True)
        self.declare_parameter('init_wait', 20.0)
        self.declare_parameter('check_period', 2.0)
        self.declare_parameter('conv_checks', 3)
        self.declare_parameter('conv_x_var', 0.25)      # x 方差阈值 (m^2), σ=0.5m
        self.declare_parameter('conv_yaw_var', 0.075)   # yaw 方差阈值 (rad^2), σ≈16°
        self.declare_parameter('check_timeout', 40.0)
        self.declare_parameter('max_retries', 2)
        self.declare_parameter('cov_topic', '/amcl_cov')

        self.relay_on = bool(self.get_parameter('relay_enabled').value)
        self.relay_delay = float(self.get_parameter('relay_delay').value)
        self.yaw_sigma = float(self.get_parameter('yaw_sigma').value)
        self.auto_on = bool(self.get_parameter('auto_global_reloc').value)
        self.init_wait = float(self.get_parameter('init_wait').value)
        self.check_period = float(self.get_parameter('check_period').value)
        self.conv_n = int(self.get_parameter('conv_checks').value)
        self.conv_x_var = float(self.get_parameter('conv_x_var').value)
        self.conv_yaw_var = float(self.get_parameter('conv_yaw_var').value)
        self.check_timeout = float(self.get_parameter('check_timeout').value)
        self.max_retries = int(self.get_parameter('max_retries').value)

        # /initialpose 和 /amcl_pose 的双方 (RViz / AMCL) 都用 default QoS
        self.create_subscription(
            PoseWithCovarianceStamped, '/initialpose', self.on_initialpose, 10)
        self.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose', self.on_amcl_pose, 10)
        self.cov_pub = self.create_publisher(
            Float32MultiArray, self.get_parameter('cov_topic').value, 10)
        self.set_pose_cli = self.create_client(SetInitialPose, '/set_initial_pose')
        self.reinit_cli = self.create_client(Empty, '/reinitialize_global_localization')

        # 状态机: wait_init -> check -> done
        self.state = 'wait_init'
        self.start = self.get_clock().now()
        self.got_initialpose = False
        self.got_amcl_pose = False
        self.last_pose = None      # 最新 amcl_pose
        self.conv_streak = 0       # 连续收敛拍数
        self.retries = 0
        self.check_start = None

        self.create_timer(1.0, self.tick)
        self.create_timer(5.0, self.heartbeat)
        self.get_logger().info(
            f'启动: relay={self.relay_on} (yaw σ={math.degrees(self.yaw_sigma):.0f}°) '
            f'auto_global={self.auto_on} init_wait={self.init_wait}s '
            f'conv(cov0<{self.conv_x_var}, cov35<{self.conv_yaw_var}) '
            f'x{self.conv_n} timeout={self.check_timeout}s retries={self.max_retries}')

    # ------------------------------------------------------------------
    # 功能 A: initialpose 中继, 放大 yaw 协方差
    # ------------------------------------------------------------------
    def on_initialpose(self, msg):
        """RViz 给了初始位姿: 稍后转发放大版; 同时标记"有人给过位姿",
        取消功能 B 的自动全局定位。
        [2026-10-25 晚修正] 之前只在 wait_init 状态才置 done: 若自动撒粒子
        已进入 check 收敛循环, 手动给 initialpose 不会退出循环, 40s 后还会
        重撒把手动位姿冲掉 -> "定位乱跑"。现在任何状态收到 initialpose
        都立即停止自动循环。"""
        self.got_initialpose = True
        if self.state != 'done':
            self.state = 'done'
            self.get_logger().info('收到 /initialpose, 停止自动全局定位循环')
        if not self.relay_on:
            self.get_logger().info('收到 /initialpose (relay 关闭, 不放大转发)')
            return
        self.get_logger().info(
            f'收到 /initialpose, {self.relay_delay}s 后转发放大版 '
            f'(yaw σ {math.degrees(self.yaw_sigma):.0f}°)')
        # 一次性 timer: 晚于 AMCL 直接收原始 initialpose, 放大版覆盖它。
        # [教训 2026-10-25 晚] rclpy 的 create_timer 是周期定时器! 之前没
        # cancel: 每点一次 RViz 就多一个每 0.5s 永久重撒初始位姿的定时器
        # (20:40 场 AMCL 收到 510 次 /set_initial_pose) -> TF 被钉死在点击
        # 位姿 (车在走 TF 不动的元凶) + σ45° 散开在稀疏地图反复收敛回旧
        # 位姿 (TF 在给定位姿和初始地间反复跳的元凶)。必须用完即 cancel。
        timer = self.create_timer(
            self.relay_delay, lambda: self._relay_once(timer, msg))

    def _relay_once(self, timer, msg):
        timer.cancel()
        self.relay_pose(msg)

    def relay_pose(self, msg):
        if not self.set_pose_cli.service_is_ready():
            self.get_logger().warn(
                '/set_initial_pose 服务未就绪 (AMCL 没起?), 放弃本次转发')
            return
        req = SetInitialPose.Request()
        req.pose = PoseWithCovarianceStamped()
        req.pose.header = msg.header
        req.pose.pose.pose = msg.pose.pose
        cov = list(msg.pose.covariance)   # RViz: (0.25, 0.25, 0.0685)
        # 只放大 yaw 方差 (index 35), 位置保持 RViz 原值 (σ=0.5m)
        cov[35] = self.yaw_sigma ** 2
        req.pose.pose.covariance = cov
        self.set_pose_cli.call_async(req)
        self.get_logger().info('已转发放大版 initialpose 到 /set_initial_pose')

    # ------------------------------------------------------------------
    # 功能 B: 无初始位姿 -> 自动全局定位 + 协方差收敛校验
    # ------------------------------------------------------------------
    def on_amcl_pose(self, msg):
        self.got_amcl_pose = True
        self.last_pose = msg

    def scatter(self):
        if not self.reinit_cli.service_is_ready():
            self.get_logger().warn(
                '重定位服务未就绪 (AMCL 没起?), 稍后再试', throttle_duration_sec=10.0)
            return False
        self.get_logger().error(
            f'[自动全局定位] 无初始位姿, 第 {self.retries + 1}/{self.max_retries + 1} 次 '
            '调 /reinitialize_global_localization 全图撒粒子')
        self.reinit_cli.call_async(Empty.Request())
        self.check_start = self.get_clock().now()
        self.conv_streak = 0
        return True

    def tick(self):
        now = self.get_clock().now()
        # 每拍重读参数: 车上随时 `ros2 param set /amcl_auto_init
        # auto_global_reloc false` 可立即叫停自动全局定位 (撒粒子循环退出,
        # AMCL 保持当前状态)
        if self.state != 'done' and not self.get_parameter('auto_global_reloc').value:
            self.state = 'done'
            self.get_logger().info(
                'auto_global_reloc 被置 false, 停止自动全局定位 (保持 AMCL 当前状态)')
            return
        if self.state == 'wait_init':
            if self.got_amcl_pose or self.got_initialpose:
                self.state = 'done'
                return
            if (now - self.start).nanoseconds * 1e-9 < self.init_wait:
                return
            if self.scatter():
                self.state = 'check'
            return

        if self.state == 'check':
            pose = self.last_pose
            if pose is not None:
                cov = pose.pose.covariance
                msg = Float32MultiArray()
                msg.data = [cov[0], cov[35]]
                self.cov_pub.publish(msg)
                converged = cov[0] < self.conv_x_var and cov[35] < self.conv_yaw_var
                if converged:
                    self.conv_streak += 1
                else:
                    self.conv_streak = 0
                self.get_logger().info(
                    f'[收敛检查] cov0={cov[0]:.3f} cov35={cov[35]:.3f} '
                    f'(阈 {self.conv_x_var}/{self.conv_yaw_var}) '
                    f'连续收敛 {self.conv_streak}/{self.conv_n}')
                if self.conv_streak >= self.conv_n:
                    self.state = 'done'
                    self.get_logger().info(
                        '✓ 自动全局定位收敛 (协方差连续达标), 定位链就绪')
                    return
            if (now - self.check_start).nanoseconds * 1e-9 > self.check_timeout:
                if self.retries < self.max_retries:
                    self.retries += 1
                    self.scatter()
                else:
                    self.state = 'done'
                    self.get_logger().error(
                        f'[自动全局定位] {self.max_retries + 1} 次撒粒子后协方差仍未收敛, '
                        '保持 AMCL 当前结果不再重试。空旷对称环境可能收敛到错误位姿, '
                        '建议放纸箱增加特征后手动给 /initialpose')

    def heartbeat(self):
        self.get_logger().info(
            f'[状态] state={self.state} 收过initialpose={self.got_initialpose} '
            f'收过amcl_pose={self.got_amcl_pose} 撒粒子次数={self.retries}')


def main(args=None):
    rclpy.init(args=args)
    node = AmclAutoInit()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
