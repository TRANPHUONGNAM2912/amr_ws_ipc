"""
Modbus RTU frame builder and parser for RS485 communication.

Frame format - Read Holding Registers (Function Code 0x03):

  REQUEST (8 bytes):
  +--------+--------+--------+--------+--------+--------+--------+--------+
  |  ADDR  |  0x03  | REG_H  | REG_L  | CNT_H  | CNT_L  | CRC_L  | CRC_H  |
  +--------+--------+--------+--------+--------+--------+--------+--------+

  RESPONSE (5 + 2*N bytes):
  +--------+--------+----------+--- 2*N data bytes ---+--------+--------+
  |  ADDR  |  0x03  | BYTE_CNT |   REG[0]  ... REG[N] | CRC_L  | CRC_H  |
  +--------+--------+----------+----------------------+--------+--------+

CRC-16/Modbus: polynomial 0xA001 (reflected), initial value 0xFFFF.
Each register is 16-bit big-endian, returned as signed int16.
"""

from __future__ import annotations

MODBUS_FC_READ_HOLDING = 0x03
_CRC_POLY = 0xA001


def crc16(data: bytes) -> int:
    """Compute CRC-16/Modbus checksum over *data*."""
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x0001:
                crc = (crc >> 1) ^ _CRC_POLY
            else:
                crc >>= 1
    return crc & 0xFFFF


def build_read_request(device_addr: int, start_reg: int, count: int) -> bytes:
    """Build a Modbus RTU FC03 read-holding-registers request frame.

    Args:
        device_addr: Slave device address (e.g. 0x50).
        start_reg:   First register address to read (e.g. 0x34).
        count:       Number of 16-bit registers to read.

    Returns:
        8-byte frame ready to be written to the serial port.
    """
    payload = bytes([
        device_addr & 0xFF,
        MODBUS_FC_READ_HOLDING,
        (start_reg >> 8) & 0xFF,
        start_reg & 0xFF,
        (count >> 8) & 0xFF,
        count & 0xFF,
    ])
    crc = crc16(payload)
    return payload + bytes([crc & 0xFF, (crc >> 8) & 0xFF])


def parse_read_response(
    device_addr: int,
    data: bytes,
) -> list[int] | None:
    """Parse a Modbus RTU FC03 response frame.

    Validates device address, function code, byte count, and CRC.
    Each register value is returned as a **signed** 16-bit integer.

    Args:
        device_addr: Expected slave address.
        data:        Raw bytes received from the serial port.

    Returns:
        List of signed int16 register values, or None if the frame is
        invalid (wrong address, bad CRC, truncated data, error response).
    """
    # Minimum valid frame: addr + fc + byte_count + 2 data bytes + 2 CRC
    if len(data) < 7:
        return None

    if data[0] != (device_addr & 0xFF):
        return None

    # Error response: function code has high bit set (e.g. 0x83)
    if data[1] & 0x80:
        return None

    if data[1] != MODBUS_FC_READ_HOLDING:
        return None

    byte_count = data[2]
    expected_len = 3 + byte_count + 2
    if len(data) < expected_len:
        return None

    # CRC covers everything except the trailing CRC bytes
    payload = data[:3 + byte_count]
    received_crc = data[3 + byte_count] | (data[3 + byte_count + 1] << 8)
    if crc16(payload) != received_crc:
        return None

    registers: list[int] = []
    for i in range(0, byte_count, 2):
        raw = (data[3 + i] << 8) | data[3 + i + 1]
        # Sign-extend to Python int
        registers.append(raw if raw < 0x8000 else raw - 0x10000)

    return registers
