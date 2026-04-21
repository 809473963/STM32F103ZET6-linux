import queue
import threading
import tkinter as tk
from tkinter import ttk
from typing import Dict, List, Tuple

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rcl_interfaces.msg import Parameter as RosParameter
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.msg import ParameterValue
from rcl_interfaces.srv import GetParameters
from rcl_interfaces.srv import SetParameters

from motor_control_msgs.msg import MotorControlState


PARAM_SPECS: List[Tuple[str, float, float, float]] = [
    ("tuning.group_a.p_des", -12.5, 12.5, 0.01),
    ("tuning.group_a.v_des", -30.0, 30.0, 0.05),
    ("tuning.group_a.kp", 0.0, 500.0, 0.1),
    # DM4310 path clamps kd <= 5. Keep slider range aligned with actual effect.
    ("tuning.group_a.kd", 0.0, 5.0, 0.01),
    ("tuning.group_a.t_ff", -18.0, 18.0, 0.05),
]


class TuningSliderNode(Node):
    def __init__(self, target_node: str) -> None:
        super().__init__("tuning_slider_gui")
        self._target_node = target_node
        self._get_client = self.create_client(GetParameters, f"{target_node}/get_parameters")
        self._set_client = self.create_client(SetParameters, f"{target_node}/set_parameters")
        self._state_queue: "queue.Queue[MotorControlState]" = queue.Queue(maxsize=10)
        self._fetch_queue: "queue.Queue[Tuple[bool, Dict[str, object], str]]" = queue.Queue(maxsize=10)
        self._set_queue: "queue.Queue[Tuple[bool, str]]" = queue.Queue(maxsize=10)

        self.create_subscription(MotorControlState, "motor_control/state", self._on_state, 20)

    def _on_state(self, msg: MotorControlState) -> None:
        try:
            self._state_queue.put_nowait(msg)
        except queue.Full:
            try:
                _ = self._state_queue.get_nowait()
            except queue.Empty:
                pass
            try:
                self._state_queue.put_nowait(msg)
            except queue.Full:
                pass

    def request_fetch_parameters(self) -> None:
        if not self._get_client.wait_for_service(timeout_sec=0.2):
            self._fetch_queue.put((False, {}, "参数服务不可用: get_parameters"))
            return

        names = ["tuning.group_a.enable", "tuning.group_a.motor_id"] + [p[0] for p in PARAM_SPECS]
        req = GetParameters.Request()
        req.names = names
        future = self._get_client.call_async(req)
        future.add_done_callback(self._on_fetch_done)

    def _on_fetch_done(self, future) -> None:
        try:
            result = future.result()
            values: Dict[str, object] = {}
            names = ["tuning.group_a.enable", "tuning.group_a.motor_id"] + [p[0] for p in PARAM_SPECS]
            for idx, p in enumerate(result.values):
                name = names[idx]
                if p.type == ParameterType.PARAMETER_BOOL:
                    values[name] = p.bool_value
                elif p.type == ParameterType.PARAMETER_INTEGER:
                    values[name] = int(p.integer_value)
                elif p.type == ParameterType.PARAMETER_DOUBLE:
                    values[name] = float(p.double_value)
            self._fetch_queue.put((True, values, ""))
        except Exception as exc:
            self._fetch_queue.put((False, {}, str(exc)))

    def request_set_parameters(self, values: Dict[str, object]) -> None:
        if not self._set_client.wait_for_service(timeout_sec=0.2):
            self._set_queue.put((False, "参数服务不可用: set_parameters"))
            return

        req = SetParameters.Request()
        for name, value in values.items():
            param = RosParameter()
            param.name = name

            pval = ParameterValue()
            if name == "tuning.group_a.enable":
                pval.type = ParameterType.PARAMETER_BOOL
                pval.bool_value = bool(value)
            elif name == "tuning.group_a.motor_id":
                pval.type = ParameterType.PARAMETER_INTEGER
                pval.integer_value = int(value)
            else:
                pval.type = ParameterType.PARAMETER_DOUBLE
                pval.double_value = float(value)

            param.value = pval
            req.parameters.append(param)

        future = self._set_client.call_async(req)
        future.add_done_callback(self._on_set_done)

    def _on_set_done(self, future) -> None:
        try:
            result = future.result()
            ok = all(r.successful for r in result.results)
            msg = "; ".join(r.reason for r in result.results if r.reason) if not ok else ""
            self._set_queue.put((ok, msg))
        except Exception as exc:
            self._set_queue.put((False, str(exc)))


class TuningSliderApp:
    def __init__(self, node: TuningSliderNode) -> None:
        self._node = node
        self._root = tk.Tk()
        self._root.title("STM32 DM4310 Realtime Tuning")
        self._root.geometry("760x560")

        self._enable_var = tk.BooleanVar(value=True)
        self._motor_id_var = tk.IntVar(value=1)
        self._slider_vars: Dict[str, tk.DoubleVar] = {}
        self._value_labels: Dict[str, ttk.Label] = {}
        self._dirty: Dict[str, object] = {}
        self._debounce_id = None

        self._state_labels: Dict[str, ttk.Label] = {}
        self._status_text = tk.StringVar(value="等待连接 stm32_serial_bridge ...")

        self._build_ui()
        self._root.after(200, self._tick)
        self._node.request_fetch_parameters()

    def _build_ui(self) -> None:
        main = ttk.Frame(self._root, padding=12)
        main.pack(fill=tk.BOTH, expand=True)

        top = ttk.LabelFrame(main, text="控制参数", padding=10)
        top.pack(fill=tk.X)

        row0 = ttk.Frame(top)
        row0.pack(fill=tk.X, pady=4)
        ttk.Checkbutton(row0, text="Enable", variable=self._enable_var, command=self._on_enable_changed).pack(
            side=tk.LEFT
        )
        ttk.Label(row0, text="Motor ID").pack(side=tk.LEFT, padx=(16, 6))
        motor_spin = tk.Spinbox(row0, from_=1, to=32, width=5, textvariable=self._motor_id_var, command=self._on_motor_id_changed)
        motor_spin.pack(side=tk.LEFT)
        motor_spin.bind("<KeyRelease>", lambda _e: self._on_motor_id_changed())

        for idx, (name, vmin, vmax, step) in enumerate(PARAM_SPECS):
            row = ttk.Frame(top)
            row.pack(fill=tk.X, pady=4)

            pretty = name.split(".")[-1]
            ttk.Label(row, text=f"{pretty}", width=10).pack(side=tk.LEFT)

            var = tk.DoubleVar(value=0.0)
            self._slider_vars[name] = var

            scale = ttk.Scale(
                row,
                from_=vmin,
                to=vmax,
                orient=tk.HORIZONTAL,
                variable=var,
                command=lambda _val, n=name, s=step: self._on_slider_changed(n, s),
            )
            scale.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=8)

            value_label = ttk.Label(row, text="0.000", width=9, anchor="e")
            value_label.pack(side=tk.LEFT)
            self._value_labels[name] = value_label

        action_row = ttk.Frame(top)
        action_row.pack(fill=tk.X, pady=(8, 0))
        ttk.Button(action_row, text="刷新参数", command=self._node.request_fetch_parameters).pack(side=tk.LEFT)
        ttk.Button(action_row, text="立即下发", command=self._flush_dirty_now).pack(side=tk.LEFT, padx=8)

        bottom = ttk.LabelFrame(main, text="电机状态反馈", padding=10)
        bottom.pack(fill=tk.BOTH, expand=True, pady=(10, 0))

        for label in ["enabled", "position(rad)", "velocity(rad/s)", "effort(Nm)", "status"]:
            row = ttk.Frame(bottom)
            row.pack(fill=tk.X, pady=3)
            ttk.Label(row, text=label, width=16).pack(side=tk.LEFT)
            val = ttk.Label(row, text="-", anchor="w")
            val.pack(side=tk.LEFT)
            self._state_labels[label] = val

        ttk.Label(main, textvariable=self._status_text, foreground="#0b6b3a").pack(anchor="w", pady=(8, 0))

    def _on_enable_changed(self) -> None:
        self._dirty["tuning.group_a.enable"] = bool(self._enable_var.get())
        self._schedule_send()

    def _on_motor_id_changed(self) -> None:
        try:
            motor_id = int(self._motor_id_var.get())
        except Exception:
            return
        self._dirty["tuning.group_a.motor_id"] = max(1, min(32, motor_id))
        self._schedule_send()

    def _on_slider_changed(self, name: str, step: float) -> None:
        raw = float(self._slider_vars[name].get())
        if step > 0:
            quantized = round(raw / step) * step
        else:
            quantized = raw
        self._slider_vars[name].set(quantized)
        self._value_labels[name].configure(text=f"{quantized:.3f}")
        self._dirty[name] = quantized
        self._schedule_send()

    def _schedule_send(self) -> None:
        if self._debounce_id is not None:
            self._root.after_cancel(self._debounce_id)
        self._debounce_id = self._root.after(80, self._flush_dirty_now)

    def _flush_dirty_now(self) -> None:
        if self._debounce_id is not None:
            self._root.after_cancel(self._debounce_id)
            self._debounce_id = None
        if not self._dirty:
            return
        payload = dict(self._dirty)
        self._dirty.clear()
        self._node.request_set_parameters(payload)

    def _tick(self) -> None:
        self._drain_fetch_results()
        self._drain_set_results()
        self._drain_state_updates()
        self._root.after(100, self._tick)

    def _drain_fetch_results(self) -> None:
        while True:
            try:
                ok, values, err = self._node._fetch_queue.get_nowait()
            except queue.Empty:
                return

            if not ok:
                self._status_text.set(f"读取参数失败: {err}")
                continue

            if "tuning.group_a.enable" in values:
                self._enable_var.set(bool(values["tuning.group_a.enable"]))
            if "tuning.group_a.motor_id" in values:
                self._motor_id_var.set(int(values["tuning.group_a.motor_id"]))

            for name, _vmin, _vmax, _step in PARAM_SPECS:
                if name in values:
                    val = float(values[name])
                    self._slider_vars[name].set(val)
                    self._value_labels[name].configure(text=f"{val:.3f}")

            self._status_text.set("参数已同步")

    def _drain_set_results(self) -> None:
        while True:
            try:
                ok, msg = self._node._set_queue.get_nowait()
            except queue.Empty:
                return
            if ok:
                self._status_text.set("参数下发成功")
            else:
                self._status_text.set(f"参数下发失败: {msg}")

    def _drain_state_updates(self) -> None:
        latest = None
        while True:
            try:
                latest = self._node._state_queue.get_nowait()
            except queue.Empty:
                break
        if latest is None:
            return

        self._state_labels["enabled"].configure(text=str(bool(latest.enabled)))
        self._state_labels["position(rad)"].configure(text=f"{latest.position:.4f}")
        self._state_labels["velocity(rad/s)"].configure(text=f"{latest.velocity:.4f}")
        self._state_labels["effort(Nm)"].configure(text=f"{latest.effort:.4f}")
        self._state_labels["status"].configure(text=f"{latest.status} ({latest.status_text})")

    def run(self) -> None:
        self._root.mainloop()


def main() -> None:
    rclpy.init()
    target_node = "/stm32_serial_bridge"
    node = TuningSliderNode(target_node=target_node)

    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    try:
        app = TuningSliderApp(node)
        app.run()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
