"""
Low-level driver for the WitMotion HWT905-485 9-axis IMU sensor.

The device communicates over RS485 using Modbus RTU protocol.
When connected via a USB-to-RS485 adapter the port appears as
/dev/ttyUSB0 (or similar).

────────────────────────────────────────────────────────────────────────
 HWT905 Register Map (from HWT905.pdf v20-0707 datasheet)
────────────────────────────────────────────────────────────────────────
 Addr  Symbol   Meaning                   Formula
 0x34  AX       X-axis Acceleration       (int16) / 32768 × 16 g → m/s²
 0x35  AY       Y-axis Acceleration
 0x36  AZ       Z-axis Acceleration
 0x37  GX       X-axis Angular velocity   (int16) / 32768 × 2000 → °/s
 0x38  GY       Y-axis Angular velocity
 0x39  GZ       Z-axis Angular velocity
 0x3A  HX       X-axis Magnetic field     (int16) raw LSB  (~0.15 µT/LSB)
 0x3B  HY       Y-axis Magnetic field
 0x3C  HZ       Z-axis Magnetic field
 0x3D  Roll     X-axis Angle              (int16) / 32768 × 180 → °
 0x3E  Pitch    Y-axis Angle                range: ±180° (Pitch ±90°)
 0x3F  Yaw      Z-axis Angle
 0x40  TEMP     Temperature               (int16) / 100 → °C
 0x41–0x50      Reserved
 0x51  Q0       Quaternion w              (int16) / 32768
 0x52  Q1       Quaternion x
 0x53  Q2       Quaternion y
 0x54  Q3       Quaternion z
────────────────────────────────────────────────────────────────────────

NOTE: A separate "High-precision sensor Modbus protocol" uses 32-bit
combined angle registers (0x3D-0x42 for Roll/Pitch/Yaw) — that layout
applies to higher-precision models (e.g. HWT9073), NOT the HWT905.
────────────────────────────────────────────────────────────────────────

Modbus RTU frame — Read Holding Registers (FC 0x03):

  REQUEST (8 bytes):
  +--------+--------+--------+--------+--------+--------+--------+--------+
  |  ADDR  |  0x03  | REG_H  | REG_L  | CNT_H  | CNT_L  | CRC_L  | CRC_H  |
  +--------+--------+--------+--------+--------+--------+--------+--------+

  RESPONSE (5 + 2×N bytes):
  +--------+--------+----------+--- 2×N data bytes ---+--------+--------+
  |  ADDR  |  0x03  | BYTE_CNT |   REG[0] ... REG[N]  | CRC_L  | CRC_H  |
  +--------+--------+----------+----------------------+--------+--------+

  CRC: CRC-16/Modbus, polynomial 0xA001, initial value 0xFFFF.
  Registers: 16-bit big-endian, interpreted as signed int16.

Example — read 3-axis acceleration (address 0x50, registers 0x34..0x36):
  Send:   50 03 00 34 00 03 49 84
  Return: 50 03 06 AxH AxL AyH AyL AzH AzL CRC_H CRC_L
"""

from __future__ import annotations

import time
from dataclasses import dataclass
from typing import Optional

import serial

from .modbus_frame import build_read_request, parse_read_response

# ─────────────────────────────────────────────────────────────────────────────
# Physical scaling constants  (from HWT905 datasheet §5.3)
# ─────────────────────────────────────────────────────────────────────────────
_G = 9.8                                    # m/s²  (datasheet uses 9.8)
ACCEL_SCALE = 16.0 * _G / 32768.0          # LSB → m/s²
GYRO_SCALE  = 2000.0 / 32768.0             # LSB → °/s
ANGLE_SCALE = 180.0 / 32768.0             # LSB → °  (range ±180° / ±90°)
MAG_SCALE   = 0.15                         # LSB → µT  (PNI RM3100 typical)
QUAT_SCALE  = 1.0 / 32768.0               # LSB → dimensionless

# ─────────────────────────────────────────────────────────────────────────────
# Modbus register block — read 0x34 to 0x54 in a single request (33 regs)
# ─────────────────────────────────────────────────────────────────────────────
_REG_START = 0x34
_REG_COUNT = 0x21          # 0x54 - 0x34 + 1 = 33 registers
_RESPONSE_LEN = 3 + _REG_COUNT * 2 + 2    # = 71 bytes

# Index within the block  (index = register_address − _REG_START)
_I_AX    = 0x34 - _REG_START   # 0
_I_AY    = 0x35 - _REG_START   # 1
_I_AZ    = 0x36 - _REG_START   # 2
_I_GX    = 0x37 - _REG_START   # 3
_I_GY    = 0x38 - _REG_START   # 4
_I_GZ    = 0x39 - _REG_START   # 5
_I_HX    = 0x3A - _REG_START   # 6
_I_HY    = 0x3B - _REG_START   # 7
_I_HZ    = 0x3C - _REG_START   # 8
_I_ROLL  = 0x3D - _REG_START   # 9
_I_PITCH = 0x3E - _REG_START   # 10
_I_YAW   = 0x3F - _REG_START   # 11
_I_TEMP  = 0x40 - _REG_START   # 12
_I_Q0    = 0x51 - _REG_START   # 29  (w)
_I_Q1    = 0x52 - _REG_START   # 30  (x)
_I_Q2    = 0x53 - _REG_START   # 31  (y)
_I_Q3    = 0x54 - _REG_START   # 32  (z)


# ─────────────────────────────────────────────────────────────────────────────
# Data container
# ─────────────────────────────────────────────────────────────────────────────

@dataclass
class ImuData:
    """Fully scaled IMU measurement.

    Units:
        accel_*   m/s²
        gyro_*    °/s     (convert to rad/s before publishing if needed)
        mag_*     µT
        roll/pitch/yaw  °  (roll ±180°, pitch ±90°, yaw ±180°)
        quat_*    dimensionless  (w, x, y, z)
        temperature  °C
    """
    accel_x: float = 0.0
    accel_y: float = 0.0
    accel_z: float = 0.0

    gyro_x: float = 0.0
    gyro_y: float = 0.0
    gyro_z: float = 0.0

    mag_x: float = 0.0
    mag_y: float = 0.0
    mag_z: float = 0.0

    roll:  float = 0.0   # X-axis, °
    pitch: float = 0.0   # Y-axis, °
    yaw:   float = 0.0   # Z-axis, °

    quat_w: float = 1.0
    quat_x: float = 0.0
    quat_y: float = 0.0
    quat_z: float = 0.0

    temperature: float = 0.0


# ─────────────────────────────────────────────────────────────────────────────
# Driver
# ─────────────────────────────────────────────────────────────────────────────

class HWT905Driver:
    """Serial Modbus RTU driver for the WitMotion HWT905-485 IMU.

    Reads registers 0x34–0x54 in a single Modbus FC03 transaction per call.

    Typical usage::

        driver = HWT905Driver(port='/dev/ttyUSB0', baudrate=9600)
        driver.connect()
        data = driver.read()   # ImuData | None
        driver.disconnect()
    """

    def __init__(
        self,
        port: str = '/dev/ttyUSB0',
        baudrate: int = 230400,
        device_addr: int = 0x50,
        read_timeout: float = 0.02,
    ) -> None:
        self.port         = port
        self.baudrate     = baudrate
        self.device_addr  = device_addr
        self.read_timeout = read_timeout

        self._serial: Optional[serial.Serial] = None
        # Pre-built request frame (constant for a given config)
        self._req: bytes = build_read_request(
            device_addr, _REG_START, _REG_COUNT
        )

    # ── Connection ────────────────────────────────────────────────────────

    def connect(self) -> None:
        """Open the RS485/serial port."""
        self._serial = serial.Serial(
            port=self.port,
            baudrate=self.baudrate,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=self.read_timeout,
        )

    def disconnect(self) -> None:
        """Close the serial port."""
        if self._serial and self._serial.is_open:
            self._serial.close()
            self._serial = None

    @property
    def is_connected(self) -> bool:
        return self._serial is not None and self._serial.is_open

    # ── Public API ────────────────────────────────────────────────────────

    def read(self) -> Optional[ImuData]:
        """Perform one Modbus transaction and return decoded IMU data.

        Returns:
            ImuData on success, None on communication / CRC error.
        """
        regs = self._transact()
        if regs is None or len(regs) < _REG_COUNT:
            return None
        return _decode(regs)

    # ── Internal I/O ──────────────────────────────────────────────────────

    def _transact(self) -> Optional[list[int]]:
        """Send FC03 request, read and parse the response."""
        if not self.is_connected:
            return None
        try:
            self._serial.reset_input_buffer()
            self._serial.write(self._req)
            raw = self._serial.read(_RESPONSE_LEN)
        except serial.SerialException:
            return None
        return parse_read_response(self.device_addr, raw)

    # ── Configuration (write single register — FC06) ──────────────────────

    def _write_reg(self, reg: int, value: int) -> None:
        """Modbus FC06 – Write Single Register (16-bit)."""
        if not self.is_connected:
            return
        from .modbus_frame import crc16
        payload = bytes([
            self.device_addr & 0xFF,
            0x06,
            (reg >> 8) & 0xFF,
            reg & 0xFF,
            (value >> 8) & 0xFF,
            value & 0xFF,
        ])
        crc = crc16(payload)
        self._serial.reset_input_buffer()
        self._serial.write(payload + bytes([crc & 0xFF, (crc >> 8) & 0xFF]))
        self._serial.read(8)   # wait for echo
        time.sleep(0.05)

    def _unlock(self) -> None:
        """Unlock configuration registers (required before any write)."""
        self._write_reg(0x69, 0xB588)

    def _save(self) -> None:
        """Persist current configuration to flash."""
        self._write_reg(0x00, 0x0000)

    def set_baudrate(self, baudrate: int) -> None:
        """Change the device baud rate (takes effect after power cycle).

        Supported: 4800, 9600, 19200, 38400, 57600, 115200,
                   230400, 460800, 921600
        """
        _MAP = {
            4800: 1, 9600: 2, 19200: 3, 38400: 4,
            57600: 5, 115200: 6, 230400: 7,
            460800: 8, 921600: 9,
        }
        idx = _MAP.get(baudrate)
        if idx is None:
            raise ValueError(f'Unsupported baudrate: {baudrate}')
        self._unlock()
        self._write_reg(0x04, idx)
        self._save()

    def set_modbus_address(self, addr: int) -> None:
        """Change the Modbus slave address (0x01–0x7F).

        The new address takes effect immediately after save.
        Remember to update ``self.device_addr`` and reconnect
        if you plan to keep communicating with the device.
        """
        if not (0x01 <= addr <= 0x7F):
            raise ValueError(f'Address must be 0x01–0x7F, got 0x{addr:02X}')
        self._unlock()
        self._write_reg(0x1A, addr)
        self._save()

    def calibrate_acceleration(self) -> None:
        """Trigger automatic accelerometer zero-bias calibration.

        Place the sensor on a flat, horizontal surface before calling.
        The calibration takes ~3–5 seconds to complete internally.
        """
        self._unlock()
        self._write_reg(0x01, 0x0001)   # enter accel calibration mode
        time.sleep(4.0)
        self._write_reg(0x01, 0x0000)   # exit calibration mode
        self._save()

    def calibrate_magnetic(self) -> None:
        """Enter magnetic field calibration mode.

        After calling, slowly rotate the sensor 360° around each of the
        X, Y, Z axes.  Then call ``finish_magnetic_calibration()``.
        """
        self._unlock()
        self._write_reg(0x01, 0x0007)   # magnetic calibration mode

    def finish_magnetic_calibration(self) -> None:
        """Exit magnetic calibration mode and save the result."""
        self._write_reg(0x01, 0x0000)
        self._save()

    def reboot(self) -> None:
        """Reboot the sensor."""
        self._unlock()
        self._write_reg(0x00, 0x00FF)


# ─────────────────────────────────────────────────────────────────────────────
# Register decoding  (module-level pure function, easy to unit-test)
# ─────────────────────────────────────────────────────────────────────────────

def _decode(regs: list[int]) -> ImuData:
    """Convert a raw register list (from address 0x34) to physical units.

    Uses the HWT905 datasheet formulas:
        Acceleration:  raw / 32768 × 16 × 9.8   [m/s²]
        Angular vel.:  raw / 32768 × 2000        [°/s]
        Angle:         raw / 32768 × 180         [°]
        Magnetic:      raw × 0.15                [µT]
        Quaternion:    raw / 32768               [dimensionless]
        Temperature:   raw / 100                 [°C]
    """
    return ImuData(
        accel_x=regs[_I_AX] * ACCEL_SCALE,
        accel_y=regs[_I_AY] * ACCEL_SCALE,
        accel_z=regs[_I_AZ] * ACCEL_SCALE,

        gyro_x=regs[_I_GX] * GYRO_SCALE,
        gyro_y=regs[_I_GY] * GYRO_SCALE,
        gyro_z=regs[_I_GZ] * GYRO_SCALE,

        mag_x=regs[_I_HX] * MAG_SCALE,
        mag_y=regs[_I_HY] * MAG_SCALE,
        mag_z=regs[_I_HZ] * MAG_SCALE,

        roll=regs[_I_ROLL]  * ANGLE_SCALE,
        pitch=regs[_I_PITCH] * ANGLE_SCALE,
        yaw=regs[_I_YAW]   * ANGLE_SCALE,

        quat_w=regs[_I_Q0] * QUAT_SCALE,
        quat_x=regs[_I_Q1] * QUAT_SCALE,
        quat_y=regs[_I_Q2] * QUAT_SCALE,
        quat_z=regs[_I_Q3] * QUAT_SCALE,

        temperature=regs[_I_TEMP] / 100.0,
    )
