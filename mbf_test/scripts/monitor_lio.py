#监测 FAST-LIO 里程计是否突然飘走/发散 (实车排查用, 只打日志不发布任何东西)。

import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, LaserScan


class LioMonitor(Node):
    def __init__(self):
        super().__init__('monitor_lio')
        self.declare_parameter('jump_th', 0.3)
        self.declare_parameter('z_max', 2.0)
        self.jump_th = float(self.get_parameter('jump_th').value)
        self.z_max = float(self.get_parameter('z_max').value)

        self.last_odom = None  # (t, x, y, z)
        self.scan_count = 0
        self.imu_count = 0
        self.amcl_count = 0

        # 默认 QoS (reliable) 订阅可以连上任何发布端, 包括 best_effort 的 /scan
        self.create_subscription(Odometry, '/Odometry', self.on_odom, 10)
        self.create_subscription(LaserScan, '/scan', self.on_scan, 10)
        self.create_subscription(Imu, '/imu_level', self.on_imu, 10)
        self.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose', self.on_amcl, 10)
        self.create_timer(2.0, self.tick)
        self.get_logger().info(
            f'LIO 监测启动: jump_th={self.jump_th}m z_max={self.z_max}m')

    def on_odom(self, msg):
        t = rclpy.time.Time.from_msg(msg.header.stamp).nanoseconds * 1e-9
        p = msg.pose.pose.position
        if abs(p.z) > self.z_max:
            self.get_logger().error(
                f'[发散] z={p.z:.2f}m 超阈值! pos=({p.x:.2f},{p.y:.2f},{p.z:.2f}) t={t:.3f}')
        if self.last_odom is not None:
            t0, x0, y0, z0 = self.last_odom
            dt = t - t0
            d = math.sqrt((p.x - x0) ** 2 + (p.y - y0) ** 2 + (p.z - z0) ** 2)
            if 0.0 < dt < 0.5 and d > self.jump_th:
                self.get_logger().error(
                    f'[跳变] {dt * 1000:.0f}ms 内位移 {d:.2f}m: '
                    f'({x0:.2f},{y0:.2f},{z0:.2f}) -> ({p.x:.2f},{p.y:.2f},{p.z:.2f}) '
                    f't={t:.3f}')
        self.last_odom = (t, p.x, p.y, p.z)

    def on_scan(self, msg):
        self.scan_count += 1

    def on_imu(self, msg):
        self.imu_count += 1

    def on_amcl(self, msg):
        self.amcl_count += 1

    def tick(self):
        odom = self.last_odom
        pos = f'({odom[1]:.3f},{odom[2]:.3f},{odom[3]:.3f})' if odom else '无数据'
        self.get_logger().info(
            f'心跳: odom={pos} scan={self.scan_count}Hz imu={self.imu_count}Hz '
            f'amcl={self.amcl_count}Hz')
        if self.scan_count == 0:
            self.get_logger().warn('/scan 断流! AMCL/代价地图会跟着失效')
        if self.imu_count == 0:
            self.get_logger().warn('/imu_level 断流! FAST-LIO 会停住')
        self.scan_count = 0
        self.imu_count = 0
        self.amcl_count = 0


def main():
    rclpy.init()
    node = LioMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
