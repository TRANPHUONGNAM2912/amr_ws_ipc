"""
RS485 frame protocol: ROS2 (Master) <-> STM32 (Slave)

Frame layout (both directions, 12 bytes):

  TX  ROS2 → STM32:
  ┌───────┬───────┬───────┬──────────────────┬──────────────────┬───────┬───────┐
  │ 0x52  │ 0x53  │ 0x08  │   vl  float32 LE │   vr  float32 LE │ CRC_L │ CRC_H │
  │ SOF1  │ SOF2  │  LEN  │     (4 bytes)    │     (4 bytes)    │       │       │
  └───────┴───────┴───────┴──────────────────┴──────────────────┴───────┴───────┘

  RX  STM32 → ROS2:
  ┌───────┬───────┬───────┬──────────────────┬──────────────────┬───────┬───────┐
  │ 0x4F  │ 0x44  │ 0x08  │   vx  float32 LE │ vyaw float32 LE  │ CRC_L │ CRC_H │
  │ SOF1  │ SOF2  │  LEN  │     (4 bytes)    │     (4 bytes)    │       │       │
  └───────┴───────┴───────┴──────────────────┴──────────────────┴───────┴───────┘

CRC-16/Modbus (polynomial 0xA001, init 0xFFFF) computed over:
  SOF1 + SOF2 + LEN + payload  (= first 10 bytes)
CRC appended little-endian: byte[10] = CRC & 0xFF, byte[11] = CRC >> 8

LEN field = payload byte count (8). Including LEN allows the parser
to handle variable-length frames in future firmware versions without
changing the framing logic.
"""

from __future__ import annotations
import struct

# ── SOF markers ────────────────────────────────────────────────────────────────
TX_SOF = bytes([0x52, 0x53])   # "RS" – ROS2 → STM32
RX_SOF = bytes([0x4F, 0x44])   # "OD" – STM32 → ROS2

# ── Frame sizes ────────────────────────────────────────────────────────────────
_HEADER_SIZE = 3    # SOF1 + SOF2 + LEN
_CRC_SIZE    = 2
_TX_PAYLOAD  = 8    # vl(4) + vr(4)
_RX_PAYLOAD  = 8    # vx(4) + vyaw(4)

TX_FRAME_LEN = _HEADER_SIZE + _TX_PAYLOAD + _CRC_SIZE   # 13 bytes
RX_FRAME_LEN = _HEADER_SIZE + _RX_PAYLOAD + _CRC_SIZE   # 13 bytes

# ── CRC-16/Modbus ───────────────────────────────────────────────────────────────
_CRC_POLY = 0xA001


def crc16(data: bytes) -> int:
    """CRC-16/Modbus: polynomial 0xA001, init 0xFFFF (IEC 61158)."""
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x0001:
                crc = (crc >> 1) ^ _CRC_POLY
            else:
                crc >>= 1
    return crc & 0xFFFF


# ── Frame builder ───────────────────────────────────────────────────────────────
def build_tx_frame(vl: float, vr: float) -> bytes:
    """Build a TX frame carrying left/right wheel velocity setpoints.

    Args:
        vl: Left wheel velocity in m/s (or rad/s if use_rad_s is set).
        vr: Right wheel velocity in m/s (or rad/s if use_rad_s is set).

    Returns:
        13-byte frame ready to write to the serial port.
    """
    payload = struct.pack('<ff', vl, vr)
    header  = TX_SOF + bytes([len(payload)])
    crc     = crc16(header + payload)
    return header + payload + bytes([crc & 0xFF, crc >> 8])


# ── Frame parser ────────────────────────────────────────────────────────────────
def parse_rx_frame(data: bytes) -> tuple[float, float] | None:
    """Parse a complete RX frame from the STM32.

    Args:
        data: Exactly RX_FRAME_LEN bytes starting at SOF.

    Returns:
        (vx, vyaw) on success, None on any validation failure.
    """
    if len(data) != RX_FRAME_LEN:
        return None

    if data[:2] != RX_SOF:
        return None

    payload_len = data[2]
    if payload_len != _RX_PAYLOAD:
        return None

    crc_body     = data[:_HEADER_SIZE + payload_len]
    crc_received = data[-2] | (data[-1] << 8)
    if crc16(crc_body) != crc_received:
        return None

    vx, vyaw = struct.unpack_from('<ff', data, _HEADER_SIZE)
    return vx, vyaw
