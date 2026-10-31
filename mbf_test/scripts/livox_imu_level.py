#把 /livox/imu 旋转到水平系, 发布为 /imu_level, 供 FAST-LIO 订阅。

import math

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu


def rot_zyx(roll, pitch, yaw):
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp,     cp * sr,                cp * cr],
    ])


def r_to_quat(R):
    """旋转矩阵 -> (x,y,z,w) 四元数 (与 tf2 同向)"""
    t = R.trace()
    if t > 0.0:
        s = math.sqrt(t + 1.0) * 2.0
        return ((R[2][1] - R[1][2]) / s, (R[0][2] - R[2][0]) / s,
                (R[1][0] - R[0][1]) / s, s / 4.0)
    i = int(np.argmax(np.diag(R)))
    if i == 0:
        s = math.sqrt(1.0 + R[0][0] - R[1][1] - R[2][2]) * 2.0
        return (s / 4.0, (R[0][1] + R[1][0]) / s, (R[0][2] + R[2][0]) / s,
                (R[2][1] - R[1][2]) / s)
    if i == 1:
        s = math.sqrt(1.0 + R[1][1] - R[0][0] - R[2][2]) * 2.0
        return ((R[0][1] + R[1][0]) / s, s / 4.0, (R[1][2] + R[2][1]) / s,
                (R[0][2] - R[2][0]) / s)
    s = math.sqrt(1.0 + R[2][2] - R[0][0] - R[1][1]) * 2.0
    return ((R[0][2] + R[2][0]) / s, (R[1][2] + R[2][1]) / s, s / 4.0,
            (R[1][0] - R[0][1]) / s)


def quat_mul(q1, q2):
    """四元数乘法 q1⊗q2 (x,y,z,w)"""
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    return (w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
            w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2)


class LivoxImuLevel(Node):

    def __init__(self):
        super().__init__('livox_imu_level')
        self.declare_parameter('level_roll')   # 由 launch 从 lidar_extrinsic.py 传入
        self.declare_parameter('level_pitch')
        self.declare_parameter('level_yaw')
        roll = float(self.get_parameter('level_roll').value)
        pitch = float(self.get_parameter('level_pitch').value)
        yaw = float(self.get_parameter('level_yaw').value)
        self.R = rot_zyx(roll, pitch, yaw)
        self.qR = r_to_quat(self.R)
        # livox_ros_driver2 的 /livox/imu 是默认 QoS (reliable)。
        # 订阅端必须用默认(reliable): best_effort 订阅连不上 reliable 发布,
        self.sub = self.create_subscription(Imu, '/livox/imu', self.on_imu, 10)
        self.pub = self.create_publisher(Imu, '/imu_level', 10)
        self.get_logger().info(f'livox_imu_level: R = {self.R.tolist()}')

    def on_imu(self, msg):
        a = np.array([msg.linear_acceleration.x,
                      msg.linear_acceleration.y,
                      msg.linear_acceleration.z])
        w = np.array([msg.angular_velocity.x,
                      msg.angular_velocity.y,
                      msg.angular_velocity.z])
        a2 = self.R @ a
        w2 = self.R @ w
        out = Imu()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = 'imu_level'
        out.linear_acceleration.x = float(a2[0])
        out.linear_acceleration.y = float(a2[1])
        out.linear_acceleration.z = float(a2[2])
        out.angular_velocity.x = float(w2[0])
        out.angular_velocity.y = float(w2[1])
        out.angular_velocity.z = float(w2[2])
        # 姿态四元数也乘上 R (FAST-LIO 不用 orientation, 只求数据自洽)
        q = (msg.orientation.x, msg.orientation.y,
             msg.orientation.z, msg.orientation.w)
        q2 = quat_mul(self.qR, q)
        out.orientation.x, out.orientation.y = q2[0], q2[1]
        out.orientation.z, out.orientation.w = q2[2], q2[3]
        # 协方差矩阵也旋一下 (acc / gyro 的 3x3 块)
        out.orientation_covariance = msg.orientation_covariance
        out.linear_acceleration_covariance = self.rotate_cov(
            msg.linear_acceleration_covariance)
        out.angular_velocity_covariance = self.rotate_cov(
            msg.angular_velocity_covariance)
        self.pub.publish(out)

    def rotate_cov(self, cov):
        C = np.array(cov, dtype=float).reshape(3, 3)
        if np.any(C) and np.isfinite(C).all():
            C2 = self.R @ C @ self.R.T
            return C2.flatten().tolist()
        return cov  # 全 0 或全 -1 (未知) 原样保留


def main():
    rclpy.init()
    node = LivoxImuLevel()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
