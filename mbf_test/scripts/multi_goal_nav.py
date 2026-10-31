
"""
多目标导航

两个后端:
    - mbf (默认): 调 mbf_simple_nav 的 /mbf_nav2/move_base, 走你们配的
      nav2_planner / nav2_controller 转发插件。mbf 节点名不同就用 --action 覆盖。
    - nav2: 调 bt_navigator 的 /navigate_to_pose, 每个点独立走
      "重规划 + 恢复" 行为树, 单点失败自动重试。
      (bt_navigator 已由 nav2_bringup 的 navigation_launch.py 启动)

"""

import argparse
import math
import queue
import sys
import threading
from collections import deque

import rclpy
from rclpy.action import ActionClient
from rclpy.action.client import GoalStatus
from rclpy.node import Node
from geometry_msgs.msg import PointStamped, PoseStamped, Twist
from std_msgs.msg import Bool, ColorRGBA
from tf2_ros import Buffer, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

# ----------------------------------------------------------------------
# 堵路检测 (障碍物静止 -> 自动重新全局规划绕行):
#   目标执行中, 车被障碍 (人或物) 挡住时 DWB 会把速度压到 0。若持续
#   BLOCKED_TIME 秒既没有速度指令也没有位移, 判定障碍物是静止的, 取消
#   当前目标并重发同一个点 —— MBF 会重新 GetPath, 此时全局代价地图里
#   已标记该障碍 (全局代价地图同样订阅 /scan), 新路径会绕开它。
#   人走开了车会自己继续走, 不会触发; 只挡路片刻也不触发。
# ----------------------------------------------------------------------
ODOM_FRAME = 'odom'  # 里程计帧 (与 mbf_params.yaml 的 odom_frame 一致; 2026-10-30 TF 简化), 判车是否物理移动
MAP_FRAME = 'map'           # 航点所在帧, 判与目标距离
BLOCKED_TIME = 5.0          # 障碍静止超过该时长才重规划 (秒)
BLOCKED_DIST = 0.15         # 该时长内物理位移小于该距离才算被堵 (米)
GOAL_GUARD_DIST = 0.6       # 距目标小于该距离不触发 (终点收尾交给控制器, 米)
REPLAN_MAX = 3              # 单个航点最多自动重规划次数, 用尽后放弃该点


def quat_from_yaw(yaw):
    """yaw -> 单位四元数 [x, y, z, w]"""
    return [0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)]


def yaw_between(a, b):
    return math.atan2(b.y - a.y, b.x - a.x)


def _dist(a, b):
    """两点的平面距离, 入参是 (x, y) 元组"""
    return math.hypot(a[0] - b[0], a[1] - b[1])


class MultiGoalNav(Node):

    def __init__(self, backend, action, frame, stop_on_failure, auto_yaw):
        super().__init__('multi_goal_nav')

        # 按后端懒加载 action 类型, 避免互相依赖
        if backend == 'nav2':
            from nav2_msgs.action import NavigateToPose
            action = action or 'navigate_to_pose'
            self._action_cls = NavigateToPose
        elif backend == 'mbf':
            from mbf_msgs.action import MoveBase
            action = action or '/mbf_nav2/move_base'
            self._action_cls = MoveBase
        else:
            raise SystemExit('--backend 只能是 nav2 或 mbf')
        self._client = ActionClient(self, self._action_cls, action)

        self._backend = backend
        self._action_name = action
        self._frame = frame
        self._stop_on_failure = stop_on_failure
        self._auto_yaw = auto_yaw

        # RViz "Publish Point" 工具点一下就发一帧 /clicked_point
        self._sub = self.create_subscription(
            PointStamped, 'clicked_point', self._on_click, 10)
        # 航点可视化: RViz 加 MarkerArray 显示, topic /waypoint_markers
        self._marker_pub = self.create_publisher(MarkerArray, 'waypoint_markers', 10)

        # 状态
        self._waypoints = []      # 航点队列 (走完的不删, 用 _idx 区分, 显示成灰色)
        self._idx = 0             # 下一个要执行的下标
        self._running = False     # 是否在跑导航
        self._goal_handle = None  # 当前 action goal (None = 没有在执行的)
        self._stop_reason = None  # 主动停车原因: 's' 仅停止 / 'c' 清空 / 'q' 退出
        self._quit = False
        self._server_warned = False

        # stdin 线程 -> 主线程命令队列 (所有 rclpy 调用都留在主线程)
        self._cmd_q = queue.Queue()
        self._timer = self.create_timer(0.1, self._tick)
        threading.Thread(target=self._stdin_loop, daemon=True).start()

        # 堵路检测: 订阅 /cmd_vel 判"是否还有动作指令", 查 TF 判"是否真在动"
        self._cmd_sub = self.create_subscription(Twist, 'cmd_vel', self._on_cmd, 10)
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._last_motion = None     # 最后一次检测到"在动"(有速度指令或位移) 的时刻
        self._hist = deque()         # (t, odom_x, odom_y, map_x, map_y) 滑动窗口
        self._watchdog_wp_idx = None # 看门狗状态对应的航点下标 (换点即重置)
        self._replan_count = 0       # 当前航点已自动重规划次数
        self._replanning = False     # 正在取消并重发 (障碍静止触发的自动重规划)
        self._replan_giveup = False  # 重规划次数用尽, 取消后放弃该点

        # 1Hz 播报本脚本是否在跑导航: blocked_replan.py (单目标看门狗) 收到
        # True 就闭嘴, 避免在 MBF 目标执行期间误发 Nav2 目标
        self._active_pub = self.create_publisher(Bool, 'multi_goal_active', 10)
        self._active_timer = self.create_timer(1.0, self._publish_active)

        self._print_help()

    # ------------------------------------------------------------------ 终端
    def _stdin_loop(self):
        while True:
            try:
                line = sys.stdin.readline()
            except Exception:
                break
            if not line:  # EOF (Ctrl-D) 等价于 q
                self._cmd_q.put('q')
                break
            self._cmd_q.put(line.strip().lower())

    def _print_help(self):
        print('\n' + '=' * 64)
        print('  RViz 多目标导航  (backend: %s, action: %s)' % (self._backend, self._action_name))
        print('=' * 64)
        print('  1. RViz 工具栏选 "Publish Point"')
        print('  2. 在地图上按顺序点目标点 (绿色球 + 编号)')
        print('  3. 终端按 Enter 开始, 机器人依次前往')
        print('-' * 64)
        print('  Enter 开始 | u 撤销上一点 | c 清空 | s 停止 | h 帮助 | q 退出')
        print('=' * 64 + '\n')

    # ------------------------------------------------------------ 主循环 (timer)
    def _tick(self):
        # 1. 处理终端命令
        while not self._cmd_q.empty():
            self._handle_cmd(self._cmd_q.get_nowait())
        if self._quit:
            return

        # 2. 有正在执行的目标: 监测是否被静止障碍堵住 (超时自动重规划绕行)
        if self._running and self._goal_handle is not None:
            self._check_blocked()
            return

        # 3. 没有在执行的目标就等结果回调推进
        if not self._running:
            return

        # 3. 全部走完: 收工
        if self._idx >= len(self._waypoints):
            self._running = False
            self.get_logger().info('✔ 全部目标点已到达, 队列清空')
            self._waypoints.clear()
            self._idx = 0
            self._publish_markers()
            return

        self._send_next()

    def _handle_cmd(self, cmd):
        if cmd in ('', 'enter', '\r'):
            if self._running:
                self.get_logger().info('已经在导航中 (可继续加点, 会追加到队尾)')
            elif not self._waypoints:
                self.get_logger().info('队列为空, 请先在 RViz 里点几个目标点')
            else:
                self._running = True
                self._reset_watchdog()
                self.get_logger().info('▶ 开始多目标导航, 共 %d 个点' % len(self._waypoints))
                self._publish_markers()
        elif cmd == 'u':
            self._undo_last()
        elif cmd == 'c':
            self._stop('c')
        elif cmd == 's':
            self._stop('s')
        elif cmd == 'h':
            self._print_help()
        elif cmd == 'q':
            self._stop('q')

    def _undo_last(self):
        if not self._waypoints:
            self.get_logger().info('队列为空, 没有可撤销的点')
            return
        # 正在执行的点不能撤; 其余队尾点都可以撤
        if self._goal_handle is not None and len(self._waypoints) - 1 <= self._idx:
            self.get_logger().info('当前目标执行中, 没有其他未走的点可撤销')
            return
        wp = self._waypoints.pop()
        self._recompute_yaws()
        self.get_logger().info(
            '已撤销最后一个点 (%.2f, %.2f), 剩余 %d 个'
            % (wp.pose.position.x, wp.pose.position.y, len(self._waypoints)))
        self._publish_markers()

    def _stop(self, mode):
        """s=仅停止 / c=停止并清空 / q=停止并退出"""
        if mode == 'c':
            self._waypoints.clear()
            self._idx = 0
            self.get_logger().info('已清空所有目标点')
            self._publish_markers()
        elif mode == 's' and not self._running:
            self.get_logger().info('当前没有在导航')
            return

        if self._goal_handle is not None:
            # 取消当前目标; 结果回调看到 _stop_reason 就不继续走
            self._stop_reason = mode
            self._goal_handle.cancel_goal_async()
            self.get_logger().info('正在取消当前目标...')
        else:
            self._running = False
            if mode == 'q':
                self._quit = True
            elif mode == 's':
                self.get_logger().info('已停止 (剩余点保留, Enter 继续)')

    # ------------------------------------------------------------ 航点收/发
    def _on_click(self, msg):
        wp = PoseStamped()
        wp.header.frame_id = self._frame
        wp.header.stamp = self.get_clock().now().to_msg()
        wp.pose.position = msg.point
        wp.pose.orientation.w = 1.0  # 朝向由 _recompute_yaws 统一算
        self._waypoints.append(wp)
        self._recompute_yaws()
        self.get_logger().info(
            '+ 添加 %d 号点 (%.2f, %.2f)' % (len(self._waypoints), msg.point.x, msg.point.y))
        self._publish_markers()

    def _recompute_yaws(self):
        """中间点对准下一点方向, 最后点保持来向, 单点则 yaw=0"""
        n = len(self._waypoints)
        for i, wp in enumerate(self._waypoints):
            if self._auto_yaw and n >= 2:
                if i < n - 1:
                    yaw = yaw_between(wp.pose.position, self._waypoints[i + 1].pose.position)
                else:
                    yaw = yaw_between(self._waypoints[i - 1].pose.position, wp.pose.position)
            else:
                yaw = 0.0
            q = quat_from_yaw(yaw)
            wp.pose.orientation.x, wp.pose.orientation.y = q[0], q[1]
            wp.pose.orientation.z, wp.pose.orientation.w = q[2], q[3]

    def _send_next(self):
        if not self._client.server_is_ready():
            if not self._server_warned:
                self.get_logger().warn(
                    '等待 action server: %s ... '
                    '(导航栈没起来? 先 ros2 launch mbf_test real_car.launch.py)' % self._action_name)
                self._server_warned = True
            return
        self._server_warned = False

        wp = self._waypoints[self._idx]
        wp.header.stamp = self.get_clock().now().to_msg()

        if self._backend == 'nav2':
            goal = self._action_cls.Goal()
            goal.pose = wp
        else:
            goal = self._action_cls.Goal()
            goal.target_pose = wp
            # controller / planner / recovery_behaviors 留空 => 用 MBF 配置的默认插件

        self.get_logger().info(
            '→ 前往 %d/%d 号点 (%.2f, %.2f)'
            % (self._idx + 1, len(self._waypoints), wp.pose.position.x, wp.pose.position.y))
        self._client.send_goal_async(goal).add_done_callback(self._goal_response_cb)

    def _goal_response_cb(self, future):
        handle = future.result()
        if handle is None or not handle.accepted:
            self.get_logger().warn('%d 号点被服务器拒绝' % (self._idx + 1))
            if self._stop_reason is not None:
                self._stop_reason = None
                self._running = False
            else:
                self._on_goal_finished(GoalStatus.STATUS_ABORTED)
            return
        self._goal_handle = handle
        # 新航点: 重置堵路检测状态; 自动重规划重发的是同一个点 (下标不变),
        # 保留计数和静默计时, 保证下一次仍是 5s 判定而不是重新等一轮
        if self._idx != self._watchdog_wp_idx:
            self._watchdog_wp_idx = self._idx
            self._replan_count = 0
            self._hist.clear()
            self._last_motion = self.get_clock().now().nanoseconds / 1e9
        handle.get_result_async().add_done_callback(self._result_cb)
        if self._stop_reason is not None:
            handle.cancel_goal_async()

    def _result_cb(self, future):
        self._goal_handle = None
        if self._stop_reason is not None:
            reason = self._stop_reason
            self._stop_reason = None
            self._running = False
            if reason == 'c':
                self.get_logger().info('已停止并清空队列')
            else:
                self.get_logger().info('已停止 (剩余 %d 个点保留, Enter 继续)'
                                       % max(0, len(self._waypoints) - self._idx))
            self._publish_markers()
            return
        if self._replanning:
            # 障碍静止触发的取消已完成: 重发同一点, 让 MBF 重新 GetPath 绕行
            self._replanning = False
            if self._replan_giveup:
                self._replan_giveup = False
                self._on_goal_finished(GoalStatus.STATUS_ABORTED)
            else:
                self.get_logger().info('🔄 重新规划绕行 (%d/%d)'
                                       % (self._replan_count, REPLAN_MAX))
                self._send_next()
            return
        res = future.result()
        self._on_goal_finished(res.status, res.result)

    def _on_goal_finished(self, status, result=None):
        # mbf_simple_nav 失败时 action 状态也可能是 SUCCEEDED, 要看 result.outcome
        if self._backend == 'mbf' and result is not None:
            ok = (status == GoalStatus.STATUS_SUCCEEDED
                  and result.outcome == self._action_cls.Result.SUCCESS)
        else:
            ok = (status == GoalStatus.STATUS_SUCCEEDED)
        if ok:
            self._idx += 1
            if self._idx < len(self._waypoints):
                self.get_logger().info('✔ 到达 %d/%d 号点' % (self._idx, len(self._waypoints)))
            else:
                self.get_logger().info('✔ 最后一个点到达!')
        elif self._stop_on_failure:
            self.get_logger().error(
                '✘ %d 号点失败 (status=%d), 已停车, Enter 可重试该点' % (self._idx + 1, status))
            self._running = False
        else:
            self.get_logger().warn(
                '✘ %d 号点失败 (status=%d), 跳过继续' % (self._idx + 1, status))
            self._idx += 1
        self._publish_markers()

    # ------------------------------------------------------------ 堵路检测 (障碍静止 5s -> 自动重规划)
    def _publish_active(self):
        msg = Bool()
        msg.data = self._running
        self._active_pub.publish(msg)

    def _reset_watchdog(self):
        self._watchdog_wp_idx = None
        self._replan_count = 0
        self._replanning = False
        self._replan_giveup = False
        self._hist.clear()
        self._last_motion = None

    def _on_cmd(self, msg):
        # 有实际动作指令 (线速度或角速度) 就认为"在动"—— 堵路时 DWB 会把
        # 指令压到 ~0; 原地转向 (wz 大) 也算在动, 不会误触发
        if abs(msg.linear.x) > 0.03 or abs(msg.angular.z) > 0.15:
            self._last_motion = self.get_clock().now().nanoseconds / 1e9

    def _lookup_xy(self, frame):
        """查 base_link 在 frame 下的 (x, y); 查不到返回 None"""
        try:
            t = self._tf_buffer.lookup_transform(frame, 'base_link', rclpy.time.Time())
        except Exception:
            return None
        return (t.transform.translation.x, t.transform.translation.y)

    def _check_blocked(self):
        """目标执行中且 5s 既无速度指令又无位移 -> 判定被静止障碍堵住, 取消重发"""
        if self._replanning or self._stop_reason is not None:
            return
        odom = self._lookup_xy(ODOM_FRAME)   # 物理位移用里程计帧, 不受 AMCL 漂移影响
        mapp = self._lookup_xy(MAP_FRAME)    # 目标距离用地图帧 (航点所在帧)
        if odom is None or mapp is None:
            return
        t = self.get_clock().now().nanoseconds / 1e9
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
        wp = self._waypoints[self._idx].pose.position
        if _dist(mapp, (wp.x, wp.y)) < GOAL_GUARD_DIST:
            return

        # 被静止障碍堵住 >= BLOCKED_TIME 秒: 取消并重发同一点 -> MBF 重新全局规划绕行
        self._replanning = True
        if self._replan_count >= REPLAN_MAX:
            self._replan_giveup = True
            self.get_logger().error(
                '✘ %d 号点: 障碍静止超过 %.0fs 且已重规划 %d 次仍被堵, 放弃该点'
                % (self._idx + 1, BLOCKED_TIME, REPLAN_MAX))
        else:
            self._replan_count += 1
            self.get_logger().warn(
                '⏳ %d 号点: 前方障碍静止超过 %.0fs, 取消目标重新规划绕行 (%d/%d)'
                % (self._idx + 1, BLOCKED_TIME, self._replan_count, REPLAN_MAX))
        self._goal_handle.cancel_goal_async()

    # ------------------------------------------------------------ 可视化
    def _publish_markers(self):
        ma = MarkerArray()

        # 先删掉上一批 (同 namespace), 避免撤销/清空后残留
        clear = Marker()
        clear.header.frame_id = self._frame
        clear.header.stamp = self.get_clock().now().to_msg()
        clear.ns = 'multi_goal'
        clear.action = Marker.DELETEALL
        ma.markers.append(clear)

        pts = [w.pose.position for w in self._waypoints]
        if not pts:
            self._marker_pub.publish(ma)
            return

        # 连线
        if len(pts) >= 2:
            line = Marker()
            line.header.frame_id = self._frame
            line.header.stamp = clear.header.stamp
            line.ns = 'multi_goal'
            line.id = 1
            line.type = Marker.LINE_STRIP
            line.scale.x = 0.04
            line.color = ColorRGBA(r=0.2, g=0.8, b=0.2, a=0.8)
            line.points = pts
            ma.markers.append(line)

        # 球 + 编号
        for i, p in enumerate(pts):
            s = Marker()
            s.header.frame_id = self._frame
            s.header.stamp = clear.header.stamp
            s.ns = 'multi_goal'
            s.id = 10 + 2 * i
            s.type = Marker.SPHERE
            s.pose.position = p
            s.pose.orientation.w = 1.0
            s.scale.x = s.scale.y = s.scale.z = 0.22
            if self._running and i == self._idx:
                s.color = ColorRGBA(r=1.0, g=0.55, b=0.0, a=0.9)    # 当前点: 橙
            elif self._running and i < self._idx:
                s.color = ColorRGBA(r=0.55, g=0.55, b=0.55, a=0.45)  # 已走过: 灰
            else:
                s.color = ColorRGBA(r=0.2, g=0.8, b=0.2, a=0.8)     # 待走: 绿
            ma.markers.append(s)

            t = Marker()
            t.header.frame_id = self._frame
            t.header.stamp = clear.header.stamp
            t.ns = 'multi_goal'
            t.id = 11 + 2 * i
            t.type = Marker.TEXT_VIEW_FACING
            t.pose.position.x = p.x
            t.pose.position.y = p.y
            t.pose.position.z = p.z + 0.35
            t.pose.orientation.w = 1.0
            t.scale.z = 0.4
            t.text = str(i + 1)
            t.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
            ma.markers.append(t)

        self._marker_pub.publish(ma)


def main(argv=None):
    parser = argparse.ArgumentParser(description='RViz 多目标顺序导航')
    parser.add_argument('--backend', choices=['nav2', 'mbf'], default='mbf',
                        help='导航后端: mbf=/mbf_nav2/move_base (默认); '
                             'nav2=bt_navigator /navigate_to_pose')
    parser.add_argument('--action', default=None,
                        help='覆盖 action 名 (默认: mbf=/mbf_nav2/move_base, '
                             'nav2=/navigate_to_pose)')
    parser.add_argument('--frame', default='map', help='航点坐标系 (默认 map)')
    parser.add_argument('--stop-on-failure', action='store_true',
                        help='某个点失败就停车不跳过 (默认失败跳过继续)')
    parser.add_argument('--no-auto-yaw', action='store_true',
                        help='目标朝向固定 yaw=0 (默认自动对准下一点方向)')
    args, _ = parser.parse_known_args(argv)

    rclpy.init(args=argv)
    node = MultiGoalNav(args.backend, args.action, args.frame,
                        args.stop_on_failure, not args.no_auto_yaw)
    try:
        while rclpy.ok() and not node._quit:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        if node._goal_handle is not None:
            node._goal_handle.cancel_goal_async()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
