"""
Non-blocking RS485 serial driver with automatic reconnection.

Design:
  - read_available()  drains the OS receive buffer instantly (no blocking).
  - write()           attempts a write; returns False on error.
  - reconnect_if_needed() closes and re-opens the port when the link has been
    silent for longer than *reconnect_timeout* seconds. Call once per timer
    cycle, before read/write, so the node never stays stuck with a dead port.
"""

from __future__ import annotations
import time
import logging

try:
    import serial
    _SERIAL_OK = True
except ImportError:
    serial = None       # type: ignore[assignment]
    _SERIAL_OK = False

log = logging.getLogger(__name__)


class SerialDriver:
    """Thread-unsafe, non-blocking wrapper around pyserial for a single RS485 port."""

    def __init__(
        self,
        port: str,
        baudrate: int,
        reconnect_timeout: float = 3.0,
    ) -> None:
        self._port              = port
        self._baudrate          = baudrate
        self._reconnect_timeout = reconnect_timeout

        self._ser: 'serial.Serial | None' = None
        self._last_rx_time: float = time.monotonic()

        if not _SERIAL_OK:
            log.error('pyserial not installed. Run: pip install pyserial')

    # ── Public interface ────────────────────────────────────────────────────────

    def open(self) -> bool:
        """Open the serial port. Returns True on success."""
        if not _SERIAL_OK:
            return False
        try:
            # timeout=0 → non-blocking; read() returns immediately with
            # whatever bytes are already in the OS buffer.
            self._ser = serial.Serial(self._port, self._baudrate, timeout=0)
            self._last_rx_time = time.monotonic()
            log.info(f'RS485 opened: {self._port} @ {self._baudrate} bps')
            return True
        except Exception as exc:
            log.error(f'RS485 open failed ({self._port}): {exc}')
            self._ser = None
            return False

    def close(self) -> None:
        if self._ser is not None and self._ser.is_open:
            try:
                self._ser.close()
            except Exception:
                pass
        self._ser = None

    @property
    def is_connected(self) -> bool:
        return self._ser is not None and self._ser.is_open

    def read_available(self) -> bytes:
        """Drain all bytes currently in the OS receive buffer (non-blocking).

        Returns an empty bytes object if nothing is available or on error.
        """
        if not self.is_connected:
            return b''
        try:
            avail = self._ser.in_waiting      # type: ignore[union-attr]
            if avail > 0:
                data = self._ser.read(avail)  # type: ignore[union-attr]
                self._last_rx_time = time.monotonic()
                return data
        except Exception as exc:
            log.error(f'RS485 read error: {exc}')
            self._ser = None
        return b''

    def write(self, data: bytes) -> bool:
        """Write *data* to the port. Returns False on error."""
        if not self.is_connected:
            return False
        try:
            self._ser.write(data)             # type: ignore[union-attr]
            return True
        except Exception as exc:
            log.error(f'RS485 write error: {exc}')
            self._ser = None
            return False

    def reconnect_if_needed(self) -> bool:
        """Re-open the port if it is closed or has been silent too long.

        Returns True if the port is connected (possibly just reconnected).
        """
        if not _SERIAL_OK:
            return False

        silent_s = time.monotonic() - self._last_rx_time
        needs_reconnect = (
            not self.is_connected
            or silent_s > self._reconnect_timeout
        )

        if not needs_reconnect:
            return True

        if self.is_connected:
            log.warning(
                f'RS485 silent for {silent_s:.1f}s '
                f'(timeout={self._reconnect_timeout}s) — reconnecting'
            )
        else:
            log.info(f'RS485 port closed — attempting to open {self._port}')

        self.close()
        return self.open()

    def notify_valid_frame(self) -> None:
        """Call after a successfully parsed RX frame to reset the idle timer."""
        self._last_rx_time = time.monotonic()
