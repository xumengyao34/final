#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
堵路自动重规划

导航中若被障碍物 (人或物) 挡住, DWB 会把速度压到 0; 连续 BLOCKED_TIME 秒既
没有动作指令、车也没有位移 -> 判定障碍静止 -> 把 /goal_pose 重新发布一遍
(等价于在 RViz 里把同一个目标再点一次), bt_navigator 收到后会先取消当前
目标再重新规划执行。

分工/避让:
    multi_goal_nav.py 跑多目标时会以 1Hz 发 /multi_goal_active=True,
    本节点收到后闭嘴, 互不干扰 (避免在 MBF 目标执行时误发 Nav2 目标)。

局限:
    只对"RViz Nav2 Goal 按钮"发的目标生效 (/goal_pose 有记录); 用
    ros2 action send_goal /navigate_to_pose 直接发的目标没有 /goal_pose
    记录, 本节点不介入 (bt_navigator 自己的恢复行为树仍会兜底)。
"""

import math
import sys
from collections import deque

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Path
from std_msgs.msg import Bool
from tf2_ros import Buffer, TransformListener

# 判定参数与 multi_goal_nav.py 的看门狗保持一致
ODOM_FRAME = 'odom'  # 里程计帧 (与 mbf_params.yaml 的 odom_frame 一致; 2026-10-30 TF 简化)
MAP_FRAME = 'map'           # 航点所在帧
BLOCKED_TIME = 5.0          # 障碍静止超过该时长才重规划 (秒)
BLOCKED_DIST = 0.15         # 该时长内物理位移小于该距离才算被堵 (米)
GOAL_GUARD_DIST = 0.6       # 距目标小于该距离不触发 (终点收尾交给控制器, 米)
REPLAN_MAX = 3              # 单个目标最多自动重规划次数, 用尽后放弃 (交给 bt_navigator 自己的恢复)
GOAL_TTL = 600.0            # /goal_pose 记录的有效期: 目标发了超过该时长不再介入 (秒)
PLAN_ACTIVE_GAP = 1.5       # /local_plan 超过该时长没更新视为"当前没有在执行路径" (秒)
SUPPRESS_STALE = 3.0        # /multi_goal_active 超过该时长没收到视为多目标脚本已退出 (秒)


def _dist(a, b):
    """两点的平面距离, 入参是 (x, y) 元组"""
    return math.hypot(a[0] - b[0], a[1] - b[1])


class BlockedReplan(Node):

    def __init__(self):
        super().__init__('blocked_replan')

        # RViz "Nav2 Goal" 按钮发 /goal_pose, bt_navigator 订阅它发起导航;
        # 本节点也订阅它记录当前目标, 触发时重新发布同一帧 -> 等价于再点一次
        self._goal_sub = self.create_subscription(PoseStamped, 'goal_pose', self._on_goal, 10)
        self._goal_pub = self.create_publisher(PoseStamped, 'goal_pose', 10)

        # /local_plan 由 DWB 在执行路径时持续发布 (被堵时也发): 导航活动信号
        self._plan_sub = self.create_subscription(Path, 'local_plan', self._on_plan, 10)

        # 多目标导航脚本的活动标志: True 时本节点闭嘴
        self._suppress_sub = self.create_subscription(Bool, 'multi_goal_active', self._on_suppress, 10)

        # 动作指令 + TF 位移, 判定"是不是真被堵住了"
        self._cmd_sub = self.create_subscription(Twist, 'cmd_vel', self._on_cmd, 10)
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self._goal = None            # 最近一次 /goal_pose (PoseStamped)
        self._goal_t = 0.0           # 该目标的接收时刻
        self._last_repub_t = 0.0     # 本节点最后一次重发 /goal_pose 的时刻 (区分自己的回声)
        self._last_plan_t = 0.0      # 最后一次收到 /local_plan 的时刻
        self._last_motion = None     # 最后一次检测到"在动"的时刻
        self._suppressed = False     # 多目标脚本是否在跑
        self._suppress_t = 0.0
        self._hist = deque()         # (t, odom_x, odom_y, map_x, map_y) 滑动窗口
        self._replan_count = 0       # 当前目标已自动重规划次数
        self._gave_up_logged = False

        self._timer = self.create_timer(0.1, self._tick)
        self.get_logger().info(
            '堵路自动重规划已启用: 障碍静止 %.0fs -> 重发 /goal_pose 重新规划 (单目标最多 %d 次)'
            % (BLOCKED_TIME, REPLAN_MAX))

    # ------------------------------------------------------------------ 订阅回调
    def _on_goal(self, msg):
        t = self.get_clock().now().nanoseconds / 1e9
        same_pose = (self._goal is not None
                     and _dist((msg.pose.position.x, msg.pose.position.y),
                               (self._goal.pose.position.x, self._goal.pose.position.y)) < 0.01)
        if same_pose and t - self._last_repub_t < 2.0:
            # 本节点自己重发的回声: 只续期, 不重置计数
            self._goal_t = t
            return
        # 新目标 (或用户重新点了同一个点): 重置全部状态
        self._goal = msg
        self._goal_t = t
        self._replan_count = 0
        self._gave_up_logged = False
        self._hist.clear()
        self._last_motion = t
        self.get_logger().info('记录单目标导航 (%.2f, %.2f)'
                               % (msg.pose.position.x, msg.pose.position.y))

    def _on_plan(self, msg):
        self._last_plan_t = self.get_clock().now().nanoseconds / 1e9

    def _on_cmd(self, msg):
        # 有实际动作指令 (线速度或角速度) 就认为"在动"—— 堵路时 DWB 会把
        # 指令压到 ~0; 原地转向 (wz 大) 也算在动, 不会误触发
        if abs(msg.linear.x) > 0.03 or abs(msg.angular.z) > 0.15:
            self._last_motion = self.get_clock().now().nanoseconds / 1e9

    def _on_suppress(self, msg):
        self._suppressed = msg.data
        self._suppress_t = self.get_clock().now().nanoseconds / 1e9

    # ------------------------------------------------------------------ 主循环
    def _lookup_xy(self, frame):
        """查 base_link 在 frame 下的 (x, y); 查不到返回 None"""
        try:
            t = self._tf_buffer.lookup_transform(frame, 'base_link', rclpy.time.Time())
        except Exception:
            return None
        return (t.transform.translation.x, t.transform.translation.y)

    def _tick(self):
        t = self.get_clock().now().nanoseconds / 1e9

        # 多目标脚本在跑: 闭嘴 (它的看门狗管它自己的目标)
        if self._suppressed and t - self._suppress_t < SUPPRESS_STALE:
            return

        # 没有有效的单目标记录 (没用 RViz Nav2 Goal 发过目标, 或太久以前)
        if self._goal is None or t - self._goal_t > GOAL_TTL:
            return

        # 当前没有在跟路径 (导航结束/失败/取消, DWB 不发布 /local_plan): 不管
        if t - self._last_plan_t > PLAN_ACTIVE_GAP:
            return

        odom = self._lookup_xy(ODOM_FRAME)   # 物理位移用里程计帧, 不受 AMCL 漂移影响
        mapp = self._lookup_xy(MAP_FRAME)    # 目标距离用地图帧
        if odom is None or mapp is None:
            return
        self._hist.append((t, odom, mapp))
        while self._hist and t - self._hist[0][0] > BLOCKED_TIME + 2.0:
            self._hist.popleft()

        # 5s 内只要还有速度指令就认为"在动" (起步、原地转向、缓慢蠕行都不触发)
        if self._last_motion is None:
            self._last_motion = t
            return
        if t - self._last_motion < BLOCKED_TIME:
            return

        t0, odom0, mapp0 = self._hist[0]
        if t - t0 < BLOCKED_TIME:
            return
        odom_moved = _dist(odom, odom0)
        mapp_moved = _dist(mapp, mapp0)
        if odom_moved < BLOCKED_DIST and mapp_moved > 0.3:
            # 车没动但 map 位姿在动: 疑似 AMCL 定位漂移, 不是被堵, 交给定位侧处理
            return
        if odom_moved >= BLOCKED_DIST or mapp_moved >= BLOCKED_DIST:
            return  # 在动, 没被堵

        # 距目标太近: 终点附近的原地转向收尾交给控制器, 不触发
        gp = self._goal.pose.position
        if _dist(mapp, (gp.x, gp.y)) < GOAL_GUARD_DIST:
            return

        # 被静止障碍堵住 >= BLOCKED_TIME 秒: 重发同一个目标, 让 bt_navigator 重新规划
        if self._replan_count >= REPLAN_MAX:
            if not self._gave_up_logged:
                self._gave_up_logged = True
                self.get_logger().error(
                    '✘ 障碍静止超过 %.0fs 且已重规划 %d 次仍被堵, 放弃自动重规划, '
                    '后续交给 bt_navigator 自己的恢复行为' % (BLOCKED_TIME, REPLAN_MAX))
            return

        self._replan_count += 1
        msg = PoseStamped()
        msg.header.frame_id = self._goal.header.frame_id
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose = self._goal.pose
        self._goal_pub.publish(msg)
        self._last_repub_t = t
        self.get_logger().warn(
            '⏳ 前方障碍静止超过 %.0fs, 重发目标重新规划绕行 (%d/%d)'
            % (BLOCKED_TIME, self._replan_count, REPLAN_MAX))
        # 重置判定窗口: 新一轮执行给足 5s 再判
        self._last_motion = t
        self._hist.clear()


def main(argv=None):
    rclpy.init(args=argv)
    node = BlockedReplan()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
