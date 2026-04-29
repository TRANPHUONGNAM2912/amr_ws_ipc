#!/usr/bin/env python3
"""
ROS2 base driver node: differential drive encoder odometry over RS485.

Protocol  (CRC-16/Modbus, 13 bytes each direction):
  TX  ROS2 → STM32 : SOF[0x52,0x53] + LEN[0x08] + vl(f32LE) + vr(f32LE) + CRC16
  RX  STM32 → ROS2 : SOF[0x4F,0x44] + LEN[0x08] + vx(f32LE) + vyaw(f32LE) + CRC16

Subscriptions:
  /cmd_vel  [geometry_msgs/TwistStamped]

Publications:
  /wheel/odom  [nav_msgs/Odometry]   — twist only: vx + vyaw, for robot_localization EKF
  /diagnostics [diagnostic_msgs/DiagnosticArray]

Timer: 50 Hz (matches half the IMU rate of 98 Hz; EKF handles async inputs).

Non-blocking RS485 cycle each tick:
  1. reconnect_if_needed()
  2. read_available() → accumulate into rx buffer
  3. parse all complete frames → publish /wheel/odom
  4. send TX frame with current vl, vr
"""

from __future__ import annotations

import struct
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

from .protocol import (
    build_tx_frame,
    parse_rx_frame,
    RX_SOF,
    RX_FRAME_LEN,
    crc16,
)
from .serial_driver import SerialDriver

# ── Constants ──────────────────────────────────────────────────────────────────
_RX_BUF_MAX = 512   # hard cap on persistent RX buffer (bytes)

# Twist covariance indices (row-major 6×6):
#   [vx, vy, vz, vroll, vpitch, vyaw]
_COV_VX   = 0    # [0,0]
_COV_VYAW = 35   # [5,5]


class BaseDriverNode(Node):

    def __init__(self) -> None:
        super().__init__('base_driver_node')

        # ── Parameters ─────────────────────────────────────────────────────────
        self.declare_parameter('wheel_separation',  0.371)
        self.declare_parameter('wheel_radius',      0.075)
        self.declare_parameter('use_rad_s',         False)
        self.declare_parameter('publish_rate',      50.0)
        self.declare_parameter('cmd_vel_timeout',   0.5)
        self.declare_parameter('log_interval',      2.0)
        self.declare_parameter('enable_rs485',      True)
        self.declare_parameter('serial_port',       '/dev/ttyUSB1')
        self.declare_parameter('baudrate',          115200)
        self.declare_parameter('reconnect_timeout', 3.0)
        self.declare_parameter('odom_frame_id',     'odom')
        self.declare_parameter('base_frame_id',     'base_link')

        # Twist noise covariance: tunable via YAML
        self.declare_parameter('cov_vx',   0.01)
        self.declare_parameter('cov_vyaw', 0.05)

        self._L         = self.get_parameter('wheel_separation').value
        self._radius    = self.get_parameter('wheel_radius').value
        self._use_rad_s = self.get_parameter('use_rad_s').value
        self._rate_hz   = self.get_parameter('publish_rate').value
        self._timeout   = self.get_parameter('cmd_vel_timeout').value
        self._log_iv    = self.get_parameter('log_interval').value
        self._odom_fid  = self.get_parameter('odom_frame_id').value
        self._base_fid  = self.get_parameter('base_frame_id').value
        self._cov_vx    = self.get_parameter('cov_vx').value
        self._cov_vyaw  = self.get_parameter('cov_vyaw').value

        enable_rs485      = self.get_parameter('enable_rs485').value
        serial_port       = self.get_parameter('serial_port').value
        baudrate          = self.get_parameter('baudrate').value
        reconnect_timeout = self.get_parameter('reconnect_timeout').value

        # ── Serial driver ──────────────────────────────────────────────────────
        self._serial: SerialDriver | None = None
        if enable_rs485:
            self._serial = SerialDriver(serial_port, baudrate, reconnect_timeout)
            if not self._serial.open():
                self.get_logger().error(
                    f'Could not open RS485 port {serial_port} — will retry automatically'
                )

        # ── cmd_vel state ──────────────────────────────────────────────────────
        self._v: float = 0.0
        self._w: float = 0.0
        self._last_cmd_time = self.get_clock().now()
        self._cmd_count: int = 0

        # ── RX buffer ─────────────────────────────────────────────────────────
        self._rx_buf = bytearray()

        # ── Statistics ────────────────────────────────────────────────────────
        self._rx_ok:   int = 0
        self._rx_fail: int = 0
        self._rx_ok_at_last_log: int = 0

        # ── Publishers / subscribers ───────────────────────────────────────────
        self._sub_cmd = self.create_subscription(
            TwistStamped, 'cmd_vel', self._cb_cmd_vel, 10
        )
        self._pub_odom = self.create_publisher(Odometry, 'wheel/odom', 10)
        self._pub_diag = self.create_publisher(DiagnosticArray, '/diagnostics', 10)

        # ── Timers ─────────────────────────────────────────────────────────────
        self.create_timer(1.0 / self._rate_hz, self._timer_cb)
        if self._log_iv > 0:
            self.create_timer(self._log_iv, self._diag_cb)

        self.get_logger().info(
            f'base_driver_node ready | '
            f'L={self._L} m  R={self._radius} m  rate={self._rate_hz} Hz'
        )

    # ── cmd_vel callback ───────────────────────────────────────────────────────

    def _cb_cmd_vel(self, msg: TwistStamped) -> None:
        self._v = msg.twist.linear.x
        self._w = msg.twist.angular.z
        self._last_cmd_time = self.get_clock().now()
        self._cmd_count += 1
        if self._cmd_count == 1:
            self.get_logger().info('First cmd_vel received — motion control active.')

    # ── Main 50 Hz timer ───────────────────────────────────────────────────────

    def _timer_cb(self) -> None:
        now = self.get_clock().now()

        # Zero velocity on cmd_vel timeout
        if (now - self._last_cmd_time).nanoseconds * 1e-9 > self._timeout:
            self._v = 0.0
            self._w = 0.0

        # Differential drive kinematics: vl, vr in m/s (or rad/s)
        half_L = self._L * 0.5
        vl = self._v - self._w * half_L
        vr = self._v + self._w * half_L
        if self._use_rad_s:
            vl /= self._radius
            vr /= self._radius

        if self._serial is not None:
            self._rs485_cycle(vl, vr)

    # ── RS485 non-blocking cycle ───────────────────────────────────────────────

    def _rs485_cycle(self, vl: float, vr: float) -> None:
        """One RS485 I/O cycle — never blocks the timer callback.

        Sequence each tick (20 ms @ 50 Hz):
          1. Reconnect if port is dead or silent.
          2. Drain OS RX buffer into persistent rx_buf.
          3. Parse all complete frames → publish /wheel/odom.
          4. Send TX frame.

        STM32 responds within ~2 ms; the response is read on the next tick.
        """
        # 1. Reconnect guard
        if not self._serial.reconnect_if_needed():
            return

        # 2. Drain RX buffer
        chunk = self._serial.read_available()
        if chunk:
            self._rx_buf.extend(chunk)

        # Prevent buffer from growing unbounded (stale data scenario)
        if len(self._rx_buf) > _RX_BUF_MAX:
            self._rx_buf = self._rx_buf[-_RX_BUF_MAX:]

        # 3. Parse frames
        self._parse_rx_frames()

        # 4. Send TX
        self._serial.write(build_tx_frame(vl, vr))

    # ── Frame parser ───────────────────────────────────────────────────────────

    def _parse_rx_frames(self) -> None:
        """Extract and dispatch all complete frames from rx_buf."""
        while True:
            idx = self._rx_buf.find(RX_SOF)

            if idx < 0:
                # No SOF found — keep last byte if it is the first SOF byte
                # (partial header arriving across tick boundary).
                if self._rx_buf and self._rx_buf[-1] == RX_SOF[0]:
                    self._rx_buf = self._rx_buf[-1:]
                else:
                    self._rx_buf.clear()
                break

            if idx > 0:
                # Discard junk before SOF
                self._rx_buf = self._rx_buf[idx:]

            if len(self._rx_buf) < RX_FRAME_LEN:
                break   # Frame not yet complete — wait for next tick

            frame = bytes(self._rx_buf[:RX_FRAME_LEN])
            result = parse_rx_frame(frame)

            if result is not None:
                vx, vyaw = result
                self._rx_ok += 1
                self._serial.notify_valid_frame()
                self._publish_wheel_odom(vx, vyaw)
                self._rx_buf = self._rx_buf[RX_FRAME_LEN:]
            else:
                # CRC mismatch — log with details and skip one byte
                self._rx_fail += 1
                payload_len = frame[2] if len(frame) > 2 else 0
                crc_body = frame[:3 + payload_len]
                crc_calc = crc16(crc_body)
                crc_got  = frame[-2] | (frame[-1] << 8)
                try:
                    vx_raw, vyaw_raw = struct.unpack_from('<ff', frame, 3)
                    raw_str = f' vx={vx_raw:.3f} vyaw={vyaw_raw:.3f}'
                except Exception:
                    raw_str = ''
                self.get_logger().warning(
                    f'RS485 CRC mismatch | frame={frame.hex()} '
                    f'| crc_calc=0x{crc_calc:04x} crc_rx=0x{crc_got:04x}{raw_str}'
                )
                self._rx_buf = self._rx_buf[1:]

    # ── Odometry publisher ─────────────────────────────────────────────────────

    def _publish_wheel_odom(self, vx: float, vyaw: float) -> None:
        """Publish twist-only odometry for robot_localization EKF (odom0).

        Pose fields are left at zero; pose covariance is set to very large
        values so the EKF ignores them and tracks pose itself.

        Twist covariance reflects encoder noise only for vx [0,0] and
        vyaw [5,5]; all other axes are mechanically constrained (set large).
        """
        msg = Odometry()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = self._odom_fid
        msg.child_frame_id  = self._base_fid

        msg.twist.twist.linear.x  = float(vx)
        msg.twist.twist.angular.z = float(vyaw)

        # Twist covariance (6×6 row-major)
        tc = [0.0] * 36
        tc[_COV_VX]   = self._cov_vx    # σ²(vx)
        tc[_COV_VYAW] = self._cov_vyaw  # σ²(vyaw)
        # Constrained axes — set large so EKF de-weights them
        tc[7]  = 1e6   # vy
        tc[14] = 1e6   # vz
        tc[21] = 1e6   # vroll
        tc[28] = 1e6   # vpitch
        msg.twist.covariance = tc

        # Pose covariance — not tracked here, EKF integrates pose
        pc = [0.0] * 36
        for i in (0, 7, 14, 21, 28, 35):
            pc[i] = 1e6
        msg.pose.covariance = pc

        self._pub_odom.publish(msg)

    # ── Diagnostics callback ───────────────────────────────────────────────────

    def _diag_cb(self) -> None:
        connected = self._serial is not None and self._serial.is_connected

        # Frame rate over the last log_interval
        ok_delta   = self._rx_ok - self._rx_ok_at_last_log
        self._rx_ok_at_last_log = self._rx_ok
        expected   = int(self._rate_hz * self._log_iv)
        rate_pct   = ok_delta * 100.0 / expected if expected > 0 else 0.0

        if rate_pct >= 80.0:
            level = DiagnosticStatus.OK
            summary = f'OK  {rate_pct:.0f}% frames  ok={self._rx_ok} fail={self._rx_fail}'
        elif rate_pct >= 40.0:
            level = DiagnosticStatus.WARN
            summary = f'WARN  {rate_pct:.0f}% frames — check cable/baud'
        else:
            level = DiagnosticStatus.ERROR
            summary = f'ERROR  {rate_pct:.0f}% frames — RS485 link degraded'

        status = DiagnosticStatus()
        status.name    = 'RS485 Base Driver'
        status.hardware_id = self.get_parameter('serial_port').value
        status.level   = level
        status.message = summary
        status.values  = [
            KeyValue(key='connected',  value=str(connected)),
            KeyValue(key='rate_hz',    value=f'{ok_delta / self._log_iv:.1f}'),
            KeyValue(key='rate_pct',   value=f'{rate_pct:.0f}%'),
            KeyValue(key='crc_ok',     value=str(self._rx_ok)),
            KeyValue(key='crc_fail',   value=str(self._rx_fail)),
        ]

        arr = DiagnosticArray()
        arr.header.stamp = self.get_clock().now().to_msg()
        arr.status = [status]
        self._pub_diag.publish(arr)

        if level != DiagnosticStatus.OK:
            self.get_logger().warning(f'[diagnostics] {summary}')

    # ── Cleanup ────────────────────────────────────────────────────────────────

    def destroy_node(self, *args, **kwargs) -> None:
        if self._serial is not None:
            self._serial.close()
        super().destroy_node(*args, **kwargs)


# ── Entry point ────────────────────────────────────────────────────────────────

def main(args=None) -> None:
    rclpy.init(args=args)
    node = BaseDriverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
