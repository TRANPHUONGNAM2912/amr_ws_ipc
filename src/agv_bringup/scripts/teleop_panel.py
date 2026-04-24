#!/usr/bin/env python3

import threading
import tkinter as tk

import rclpy
from rclpy.executors import SingleThreadedExecutor
from geometry_msgs.msg import Twist
from rclpy.node import Node


class TeleopPanel(Node):
    def __init__(self):
        super().__init__("teleop_panel")

        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("publish_hz", 100.0)
        self.declare_parameter("max_speed_mps", 1.0)
        self.declare_parameter("wheel_separation", 0.38)

        self._cmd_vel_topic = (
            self.get_parameter("cmd_vel_topic").get_parameter_value().string_value
        )
        self._publish_hz = (
            self.get_parameter("publish_hz").get_parameter_value().double_value
        )
        self._max_speed_mps = (
            self.get_parameter("max_speed_mps").get_parameter_value().double_value
        )
        self._wheel_separation = (
            self.get_parameter("wheel_separation").get_parameter_value().double_value
        )

        self._pub = self.create_publisher(Twist, self._cmd_vel_topic, 10)
        self._timer = self.create_timer(
            1.0 / max(self._publish_hz, 1.0), self._on_timer
        )

        self._lock = threading.Lock()
        self._active = False
        self._vx = 0.0
        self._wz = 0.0

        self._speed_var = None

    def _on_timer(self):
        with self._lock:
            active = self._active
            vx = self._vx
            wz = self._wz

        if not active:
            return

        msg = Twist()
        msg.linear.x = float(vx)
        msg.angular.z = float(wz)
        self._pub.publish(msg)

    def _publish_zero(self):
        self._pub.publish(Twist())

    def set_command(self, vx: float, wz: float):
        with self._lock:
            self._active = True
            self._vx = float(vx)
            self._wz = float(wz)

    def stop(self):
        with self._lock:
            self._active = False
            self._vx = 0.0
            self._wz = 0.0
        self._publish_zero()

    def _speed(self) -> float:
        if self._speed_var is None:
            return 0.0
        try:
            v = float(self._speed_var.get())
        except Exception:
            return 0.0
        return max(0.0, min(float(self._max_speed_mps), v))

    def _wz_for_inplace_turn(self, wheel_speed_mps: float) -> float:
        l = float(self._wheel_separation)
        if l <= 1e-6:
            return 0.0
        return 2.0 * float(wheel_speed_mps) / l

    def build_gui(self):
        root = tk.Tk()
        root.title("AGV Teleop Panel")

        self._speed_var = tk.DoubleVar(value=min(0.3, float(self._max_speed_mps)))

        main = tk.Frame(root, padx=12, pady=12)
        main.pack(fill="both", expand=True)

        speed_row = tk.Frame(main)
        speed_row.pack(fill="x", pady=(0, 10))

        tk.Label(speed_row, text="Vận tốc (m/s)").pack(anchor="w")
        tk.Scale(
            speed_row,
            from_=0.0,
            to=float(self._max_speed_mps),
            resolution=0.01,
            orient="horizontal",
            variable=self._speed_var,
            length=260,
        ).pack(fill="x")

        grid = tk.Frame(main)
        grid.pack()

        btn_up = tk.Button(grid, text="↑", width=6, height=2)
        btn_left = tk.Button(grid, text="←", width=6, height=2)
        btn_right = tk.Button(grid, text="→", width=6, height=2)
        btn_down = tk.Button(grid, text="↓", width=6, height=2)

        btn_up.grid(row=0, column=1, padx=6, pady=6)
        btn_left.grid(row=1, column=0, padx=6, pady=6)
        btn_right.grid(row=1, column=2, padx=6, pady=6)
        btn_down.grid(row=2, column=1, padx=6, pady=6)

        def bind_hold(button: tk.Button, on_press, on_release):
            button.bind("<ButtonPress-1>", lambda _evt: on_press())
            button.bind("<ButtonRelease-1>", lambda _evt: on_release())
            button.bind("<Leave>", lambda _evt: on_release())

        bind_hold(
            btn_up,
            on_press=lambda: self.set_command(vx=self._speed(), wz=0.0),
            on_release=self.stop,
        )
        bind_hold(
            btn_down,
            on_press=lambda: self.set_command(vx=-self._speed(), wz=0.0),
            on_release=self.stop,
        )
        bind_hold(
            btn_left,
            on_press=lambda: self.set_command(
                vx=0.0, wz=+self._wz_for_inplace_turn(self._speed())
            ),
            on_release=self.stop,
        )
        bind_hold(
            btn_right,
            on_press=lambda: self.set_command(
                vx=0.0, wz=-self._wz_for_inplace_turn(self._speed())
            ),
            on_release=self.stop,
        )

        tk.Label(
            main,
            text="Giữ nút để chạy, nhả nút để dừng (gửi 0 lên /cmd_vel).",
        ).pack(pady=(10, 0))

        def on_close():
            try:
                self.stop()
            finally:
                root.destroy()

        root.protocol("WM_DELETE_WINDOW", on_close)
        return root


def main():
    rclpy.init()
    node = TeleopPanel()
    gui = node.build_gui()

    executor = SingleThreadedExecutor()
    executor.add_node(node)

    def spin():
        try:
            executor.spin()
        except Exception:
            # When shutting down, rclpy can raise if context becomes invalid mid-wait.
            pass

    spin_thread = threading.Thread(target=spin, daemon=True)
    spin_thread.start()

    try:
        gui.mainloop()
    except KeyboardInterrupt:
        # Ctrl+C should close the GUI cleanly.
        try:
            gui.quit()
            gui.destroy()
        except Exception:
            pass
    finally:
        try:
            node.stop()
        except Exception:
            pass

        try:
            executor.shutdown()
        except Exception:
            pass

        try:
            node.destroy_node()
        except Exception:
            pass

        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass

        try:
            spin_thread.join(timeout=1.0)
        except Exception:
            pass


if __name__ == "__main__":
    main()

