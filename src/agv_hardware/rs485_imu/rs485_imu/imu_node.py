"""
ROS2 node for the WitMotion HWT905-485 IMU sensor.

Published topics:
    /imu/data          sensor_msgs/Imu          – orientation (quaternion only),
                                                  angular vel, linear accel
    /imu/rpy           geometry_msgs/Vector3Stamped – roll, pitch, yaw
                                                  (degrees by default; see rpy_in_degrees)
                                                  Does not modify /imu/data.
    /imu/mag           sensor_msgs/MagneticField – magnetic field (µT)
    /imu/temperature   sensor_msgs/Temperature   – board temperature

Parameters (see config/hwt905_params.yaml for defaults):
    port          (str)   Serial port, e.g. /dev/ttyUSB0
    baudrate      (int)   Baud rate (default 9600)
    device_addr   (int)   Modbus slave address (default 0x50 = 80)
    frame_id      (str)   TF frame for IMU (default 'imu_link')
    publish_rate  (float) Publishing frequency in Hz (default 98.0)
    read_timeout  (float) Serial read timeout in seconds (default 0.02)
    publish_euler   (bool)  If true, publish /imu/rpy (default true)
    rpy_in_degrees  (bool)  If true, /imu/rpy vector is °; if false, rad (default true)
    use_angles_for_orientation (bool) If true, build /imu/data.orientation from
                                      angle registers (roll/pitch/yaw) instead
                                      of the sensor quaternion (default true)
    linear_acceleration_covariance  (list[float]) 9 values, row-major
    angular_velocity_covariance     (list[float]) 9 values, row-major
    orientation_covariance          (list[float]) 9 values, row-major
"""

from __future__ import annotations

import math
from typing import Optional

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Vector3Stamped
from sensor_msgs.msg import Imu, MagneticField, Temperature
from std_msgs.msg import Header

from .hwt905_driver import HWT905Driver, ImuData

_DEG2RAD = math.pi / 180.0


def _quat_from_rpy(roll_rad: float, pitch_rad: float, yaw_rad: float) -> tuple[float, float, float, float]:
    """
    Convert roll/pitch/yaw (ZYX intrinsic, a.k.a. yaw-pitch-roll) to quaternion.
    Returns (w, x, y, z).
    """
    cy = math.cos(yaw_rad * 0.5)
    sy = math.sin(yaw_rad * 0.5)
    cp = math.cos(pitch_rad * 0.5)
    sp = math.sin(pitch_rad * 0.5)
    cr = math.cos(roll_rad * 0.5)
    sr = math.sin(roll_rad * 0.5)

    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    return (w, x, y, z)


class ImuNode(Node):
    def __init__(self) -> None:
        super().__init__('hwt905_imu_node')

        # ── Declare parameters ──────────────────────────────────────────
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 230400)
        self.declare_parameter('device_addr', 80)        # 0x50
        self.declare_parameter('frame_id', 'imu_link')
        self.declare_parameter('publish_rate', 98.0)
        self.declare_parameter('read_timeout', 0.02)
        self.declare_parameter('linear_acceleration_covariance',
                               [0.0] * 9)
        self.declare_parameter('angular_velocity_covariance',
                               [0.0] * 9)
        self.declare_parameter('orientation_covariance',
                               [0.0] * 9)
        self.declare_parameter('publish_euler', True)
        self.declare_parameter('rpy_in_degrees', True)
        self.declare_parameter('invert_yaw', True)
        self.declare_parameter('use_angles_for_orientation', False)

        # ── Read parameters ─────────────────────────────────────────────
        port         = self.get_parameter('port').value
        baudrate     = self.get_parameter('baudrate').value
        device_addr  = self.get_parameter('device_addr').value
        self._frame_id    = self.get_parameter('frame_id').value
        publish_rate = self.get_parameter('publish_rate').value
        read_timeout = self.get_parameter('read_timeout').value

        self._accel_cov = list(
            self.get_parameter('linear_acceleration_covariance').value)
        self._gyro_cov  = list(
            self.get_parameter('angular_velocity_covariance').value)
        self._orient_cov = list(
            self.get_parameter('orientation_covariance').value)
        self._publish_euler = self.get_parameter('publish_euler').value
        self._rpy_in_degrees = self.get_parameter('rpy_in_degrees').value
        self._yaw_sign = -1.0 if self.get_parameter('invert_yaw').value else 1.0
        self._use_angles_for_orientation = self.get_parameter('use_angles_for_orientation').value

        # ── Publishers ──────────────────────────────────────────────────
        self._imu_pub  = self.create_publisher(Imu,          '/imu/data',        10)
        self._mag_pub  = self.create_publisher(MagneticField, '/imu/mag',         10)
        self._temp_pub = self.create_publisher(Temperature,   '/imu/temperature', 10)
        self._rpy_pub: Optional[object] = None
        if self._publish_euler:
            self._rpy_pub = self.create_publisher(
                Vector3Stamped, '/imu/rpy', 10)

        # ── Driver ──────────────────────────────────────────────────────
        self._driver = HWT905Driver(
            port=port,
            baudrate=baudrate,
            device_addr=device_addr,
            read_timeout=read_timeout,
        )

        self._connect()

        # ── Timer ───────────────────────────────────────────────────────
        period = 1.0 / publish_rate
        self._timer = self.create_timer(period, self._timer_cb)

        rpy_unit = '°' if self._rpy_in_degrees else 'rad'
        self.get_logger().info(
            f'HWT905 IMU node started | port={port} | '
            f'baudrate={baudrate} | device_addr=0x{device_addr:02X} | '
            f'rate={publish_rate} Hz'
            + (f' | /imu/rpy in {rpy_unit}' if self._publish_euler else '')
        )

    # ── Lifecycle helpers ────────────────────────────────────────────────

    def _connect(self) -> None:
        try:
            self._driver.connect()
            self.get_logger().info(
                f'Serial port {self._driver.port} opened successfully.')
        except Exception as exc:
            self.get_logger().error(f'Cannot open serial port: {exc}')

    def destroy_node(self) -> None:
        self._driver.disconnect()
        super().destroy_node()

    # ── Timer callback ───────────────────────────────────────────────────

    def _timer_cb(self) -> None:
        if not self._driver.is_connected:
            self.get_logger().warn(
                'IMU not connected, retrying…', throttle_duration_sec=5.0)
            self._connect()
            return

        data = self._driver.read()
        if data is None:
            self.get_logger().warn(
                'IMU read failed (timeout or CRC error).',
                throttle_duration_sec=2.0)
            return

        stamp = self.get_clock().now().to_msg()
        self._publish_imu(data, stamp)
        if self._rpy_pub is not None:
            self._publish_rpy(data, stamp)
        self._publish_mag(data, stamp)
        self._publish_temperature(data, stamp)

    # ── Message builders ─────────────────────────────────────────────────

    def _make_header(self, stamp) -> Header:
        h = Header()
        h.stamp    = stamp
        h.frame_id = self._frame_id
        return h

    def _publish_imu(self, data: ImuData, stamp) -> None:
        msg = Imu()
        msg.header = self._make_header(stamp)

        # Orientation
        #
        # IMPORTANT:
        # Some HWT905 firmware configurations yield a quaternion convention that does
        # not match the Euler angle registers (0x3D–0x3F). For consistency with EKF/RViz,
        # we optionally build the quaternion from roll/pitch/yaw registers.
        if self._use_angles_for_orientation:
            roll = data.roll * _DEG2RAD
            pitch = data.pitch * _DEG2RAD
            yaw = data.yaw * _DEG2RAD * self._yaw_sign
            (w, x, y, z) = _quat_from_rpy(roll, pitch, yaw)
            msg.orientation.w = w
            msg.orientation.x = x
            msg.orientation.y = y
            msg.orientation.z = z
        else:
            # Sensor quaternion (Q0=w, Q1=x, Q2=y, Q3=z). Optionally flip yaw sign.
            msg.orientation.w = data.quat_w
            msg.orientation.x = data.quat_x
            msg.orientation.y = data.quat_y
            msg.orientation.z = data.quat_z * self._yaw_sign

        # Angular velocity: convert °/s → rad/s (flip gz to match yaw sign)
        msg.angular_velocity.x = data.gyro_x * _DEG2RAD
        msg.angular_velocity.y = data.gyro_y * _DEG2RAD
        msg.angular_velocity.z = data.gyro_z * _DEG2RAD * self._yaw_sign

        # Linear acceleration: already in m/s²
        msg.linear_acceleration.x = data.accel_x
        msg.linear_acceleration.y = data.accel_y
        msg.linear_acceleration.z = data.accel_z

        msg.orientation_covariance         = self._orient_cov
        msg.angular_velocity_covariance    = self._gyro_cov
        msg.linear_acceleration_covariance = self._accel_cov

        self._imu_pub.publish(msg)

    def _publish_rpy(self, data: ImuData, stamp) -> None:
        """Publish roll/pitch/yaw from sensor registers.

        Units depend on ``rpy_in_degrees``: degrees (default) or radians.
        This is a separate topic from ``sensor_msgs/Imu``; it does not alter
        ``/imu/data`` (which still uses quaternion + rad/s per ROS convention).
        """
        msg = Vector3Stamped()
        msg.header = self._make_header(stamp)
        if self._rpy_in_degrees:
            msg.vector.x = float(data.roll)
            msg.vector.y = float(data.pitch)
            msg.vector.z = float(data.yaw) * self._yaw_sign
        else:
            msg.vector.x = data.roll * _DEG2RAD
            msg.vector.y = data.pitch * _DEG2RAD
            msg.vector.z = data.yaw * _DEG2RAD * self._yaw_sign
        self._rpy_pub.publish(msg)

    def _publish_mag(self, data: ImuData, stamp) -> None:
        msg = MagneticField()
        msg.header = self._make_header(stamp)
        msg.magnetic_field.x = data.mag_x
        msg.magnetic_field.y = data.mag_y
        msg.magnetic_field.z = data.mag_z
        # Covariance unknown → leave as zeros
        self._mag_pub.publish(msg)

    def _publish_temperature(self, data: ImuData, stamp) -> None:
        msg = Temperature()
        msg.header      = self._make_header(stamp)
        msg.temperature = data.temperature
        msg.variance    = 0.0
        self._temp_pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ImuNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
