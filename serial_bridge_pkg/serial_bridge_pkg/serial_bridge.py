import math
import signal
import struct
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped

try:
    import serial
except ImportError:
    serial = None  # 运行时会报错提示, 避免 import 阶段直接崩


def stop_frame():
    """零速帧 (全零, 与轴系映射无关), 用于超时/退出时强制停车。"""
    return struct.pack('<BBffff?B', 0x55, 0x04, 0.0, 0.0, 0.0, 0.0, False, 0xFF)


class CmdVelToSerial(Node):
    """把 Nav2 velocity_smoother 发出的 /cmd_vel 打包成底盘协议帧, 发串口。

    协议 (已与电控确认, 小端字节序, 共 20 字节):
        SOF(0x55) | ID(0x04) | vx | vy | wz | w | scan_mod_type | EOF(0xFF)
        其中 vx/vy/vz/w 为 float32, 单位 m/s 和 rad/s;
        w 为滚筒/拨弹轮速度, 导航不用恒为 0; scan_mod_type 恒为 False。

    注意: Humble 的 velocity_smoother 按 use_stamped 参数决定输出类型
    (true -> TwistStamped, 默认; false -> Twist)。订阅类型不匹配会静默
    收不到消息 (车不动)。本仓库 mbf_params.yaml 里 use_stamped: true,
    所以启动时用 use_stamped:=true 与之对齐 (real_car.launch.py 已改)。
    只改封装类型, 不影响速度分量与前进方向 (方向由 chassis_x_angle_deg /
    flip_wz 决定)。

    轴系补偿 (2026-10-23 实车实测, 麦克纳姆轮底盘): 底盘系 =
    Rz(chassis_x_angle_deg)·base_link, 默认 angle = -90°, 即底盘 vx 轴
    指向车右 (base_link 的 -y)。发送前把 cmd_vel 转到底盘系
    (c = Rz(-angle)·v), 否则发 vx 车会横着走。
    2026-10-23 下地实测: 当初 angle=90 (vx 轴=车左) 时发"前进"车倒着走——
    架起标定时以雷达朝向为左右基准 (雷达反装朝车尾, 雷达左=车右), 判断反了。
    改 angle=-90 后: 前进 (vx=0.2) -> 底盘 (vx=0, vy=+0.2)。方向再不对就改
    chassis_x_angle_deg (0 = 底盘 vx 指向车头, 90 = 车左, 180 = 车尾)。
    另外底盘 wz 反号 (+wz = 顺时针/车头右转, 与 ROS 逆时针为正相反),
    flip_wz=True 取反。2026-10-23 下地时 wz 修正能收敛, 反号大概率成立,
    但架起看麦轮空转容易误判, 换车/改电控后必须下地单独复验一次。

    安全看门狗 (2026-10-25 实车教训: 杀掉整个导航栈后, 底盘固件仍保持
    最后一帧"前进"指令一直走, 因为固件没有收不到指令就停的超时):
      本节点按 cmd_rate Hz 周期重发最后一帧 (保持指令流);
      超过 cmd_timeout 秒没收到新 /cmd_vel -> 改发零速帧强制停车;
      退出 (Ctrl+C / SIGTERM) 时也先连发零速帧再关串口。
      这样上游导航栈无论怎么死, 车都会在 cmd_timeout 内自己停住。
      [待办] 根因在固件: 建议电控侧加"收不到指令超时停车", 双保险。
    """

    def __init__(self):
        super().__init__('cmd_vel_to_serial')

        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('use_stamped', False)
        # 安全限幅: 防止规划器异常输出时车冲出去, 按实车调试情况调整
        self.declare_parameter('max_lin_x', 0.5)
        self.declare_parameter('max_lin_y', 0.5)
        self.declare_parameter('max_ang_z', 2.0)
        # 底盘 vx 轴在 base_link 中的方向角 (度, CCW 为正), 见类注释
        self.declare_parameter('chassis_x_angle_deg', -90.0)
        # 底盘 wz 与 ROS 约定相反 (实测 +wz = 顺时针), 见类注释
        self.declare_parameter('flip_wz', True)
        # 看门狗: 重发频率 / 断流超时 (秒), 见类注释
        self.declare_parameter('cmd_rate', 20.0)
        self.declare_parameter('cmd_timeout', 0.5)

        self.port = self.get_parameter('port').value
        self.baudrate = self.get_parameter('baudrate').value
        self.max_lin_x = self.get_parameter('max_lin_x').value
        self.max_lin_y = self.get_parameter('max_lin_y').value
        self.max_ang_z = self.get_parameter('max_ang_z').value
        angle = math.radians(self.get_parameter('chassis_x_angle_deg').value)
        self.cos_a = math.cos(angle)
        self.sin_a = math.sin(angle)
        self.flip_wz = self.get_parameter('flip_wz').value
        self.cmd_rate = float(self.get_parameter('cmd_rate').value)
        self.cmd_timeout = float(self.get_parameter('cmd_timeout').value)

        self.ser = None
        self.last_open_attempt = 0.0
        self.last_log_time = 0.0
        self.last_cmd_time = None   # 最近一次收到 /cmd_vel 的 monotonic 秒
        self.last_frame = stop_frame()  # 最后一帧 (默认零速)
        self.timeout_logged = False  # 超时状态只在进入时打一次日志
        self.open_serial()

        topic = self.get_parameter('cmd_vel_topic').value
        if self.get_parameter('use_stamped').value:
            self.create_subscription(
                TwistStamped, topic, self.stamped_callback, 10)
            self.get_logger().info(f'订阅 {topic} (TwistStamped), 串口 {self.port}@{self.baudrate}')
        else:
            self.create_subscription(
                Twist, topic, self.twist_callback, 10)
            self.get_logger().info(f'订阅 {topic} (Twist), 串口 {self.port}@{self.baudrate}')

        # 看门狗定时器: 周期重发最后一帧; /cmd_vel 断流超时 -> 发零速帧
        self.create_timer(1.0 / self.cmd_rate, self.send_tick)
        self.get_logger().info(
            f'看门狗: {self.cmd_rate}Hz 重发, /cmd_vel 断流 {self.cmd_timeout}s 发零速帧')

    def open_serial(self):
        """打开串口; 失败置 None, 由写失败路径按间隔重试 (USB 重插后能自恢复)。"""
        if self.ser is not None:
            try:
                self.ser.close()
            except Exception:
                pass
            self.ser = None
        if serial is None:
            self.get_logger().error('pyserial 未安装: pip3 install pyserial')
            return
        try:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=0.1)
            self.get_logger().info(f'串口已打开: {self.port} @ {self.baudrate}')
        except Exception as e:
            self.get_logger().error(f'串口打开失败 ({e}), 将自动重试')

    def _write(self, data):
        """发一帧; 串口未打开时按 2s 间隔重试, 不刷屏。"""
        if self.ser is None:
            now = time.monotonic()
            if now - self.last_open_attempt > 2.0:
                self.last_open_attempt = now
                self.open_serial()
            return
        try:
            self.ser.write(data)
        except Exception as e:
            self.get_logger().error(f'串口发送失败: {e}, 尝试重连')
            self.ser = None

    def stamped_callback(self, msg):
        self.twist_callback(msg.twist)

    def twist_callback(self, twist):
        # 限幅 (削顶而不是整体缩放, 保持小指令原样)
        vx = min(max(twist.linear.x, -self.max_lin_x), self.max_lin_x)
        vy = min(max(twist.linear.y, -self.max_lin_y), self.max_lin_y)
        wz = min(max(twist.angular.z, -self.max_ang_z), self.max_ang_z)
        w_speed = 0.0  # 滚筒/拨弹轮, 导航不用

        # 转到底盘系: 底盘系 = Rz(angle)·base_link, 所以 c = Rz(-angle)·v
        # (wz 绕 z 轴旋转不变, 不用动)
        chassis_vx = self.cos_a * vx + self.sin_a * vy
        chassis_vy = -self.sin_a * vx + self.cos_a * vy
        # wz 反号: 底盘 +wz = 顺时针, ROS 是逆时针为正
        if self.flip_wz:
            wz = -wz

        # 按协议打包: SOF(0x55), ID(0x04), 4×float, bool, EOF(0xFF)
        self.last_frame = struct.pack(
            '<BBffff?B', 0x55, 0x04, chassis_vx, chassis_vy, wz, w_speed, False, 0xFF)
        self.last_cmd_time = time.monotonic()
        self.timeout_logged = False
        self._write(self.last_frame)

        # 20Hz 指令流下只按 1Hz 打日志, 避免刷屏
        now = time.monotonic()
        if now - self.last_log_time > 1.0:
            self.last_log_time = now
            self.get_logger().info(
                f'发送(底盘系): vx={chassis_vx:.2f} vy={chassis_vy:.2f} wz={wz:.2f}')

    def send_tick(self):
        """看门狗: 周期重发最后一帧; /cmd_vel 断流超时 -> 发零速帧停车。"""
        now = time.monotonic()
        if self.last_cmd_time is None or now - self.last_cmd_time > self.cmd_timeout:
            if not self.timeout_logged:
                self.timeout_logged = True
                self.get_logger().error(
                    f'[看门狗] /cmd_vel 断流超 {self.cmd_timeout}s, 发零速帧停车!')
            self._write(stop_frame())
        else:
            self._write(self.last_frame)


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelToSerial()

    # SIGTERM (systemd 停服务等) 默认直接杀进程, finally 不执行;
    # 注册处理器先连发零速帧再退出, 保证"停掉一切 = 车停"
    def on_term(sig, frm):
        for _ in range(5):
            if node.ser is not None:
                try:
                    node.ser.write(stop_frame())
                except Exception:
                    pass
            time.sleep(0.05)
        raise SystemExit(0)
    signal.signal(signal.SIGTERM, on_term)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        for _ in range(5):
            if node.ser is not None:
                try:
                    node.ser.write(stop_frame())
                except Exception:
                    pass
                time.sleep(0.05)
        if node.ser is not None:
            node.ser.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
