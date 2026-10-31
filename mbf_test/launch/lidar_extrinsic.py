"""MID360 安装外参标定值。

三个 launch 从这里读, 传给 livox_imu_level / odom_guard 与静态 TF。
改安装姿态时改这里, 并同步 FAST_LIO/config/mid360.yaml 的 extrinsic_R/T。
"""
ROLL = -0.772
PITCH = 0.0
YAW = -1.5708
