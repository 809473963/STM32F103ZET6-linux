#!/usr/bin/env python3
"""
arm_car_control.py — 树莓派端统一控制
======================================
底盘 WHEELTEC + 机械臂 DM4310/DMH3510 + 撑脚 DMH3510×2 + 摄像头直播

用法 (在树莓派上运行):
  python3 arm_car_control.py
  python3 arm_car_control.py --arm-port /dev/ttyUSB0 --car-port /dev/ttyACM0
  python3 arm_car_control.py --no-car    # 只用机械臂，不连底盘

按键速查:
  底盘: W/S前后  A/D左右转  Q/E速度档+/-
  机械臂: J抬(松手锁位)  K落  1/2选电机  0全部  F紧急失能
  撑脚: T伸出(抬车)  V收回  (松开停)
  调参: [/] KP  ;/' KD  ,/. KG  N/M bias  G重力开关  B零点  C校准  H浮动  L锁存
  共用: Space全停  R臂状态  P底盘反馈  X扫描CAN  ESC退出
"""

import argparse
import glob
import logging
import math
import os
import select
import signal
import struct
import subprocess
import sys
import termios
import threading
import time
import tty

try:
    import serial
except ImportError:
    print("缺少 pyserial: pip3 install pyserial")
    sys.exit(1)

# ═══════════════════════════════════════════════════════════════════════
#  一、STM32 机械臂 / 撑脚协议
# ═══════════════════════════════════════════════════════════════════════

HEADER = 0xAA
CMD_SET_MOTOR_SPEED    = 0x01
CMD_STOP_ALL           = 0x02
CMD_GET_STATUS         = 0x03
CMD_SET_ENABLE         = 0x04
CMD_SET_MOTOR_VEL_MRAD = 0x05
CMD_DM_SCAN_IDS        = 0x21
CMD_DM_SET_ID          = 0x22
CMD_SET_GRAVITY_COMP   = 0x24
CMD_REZERO_GRAVITY     = 0x25
CMD_DM_REG_READ        = 0x26
CMD_DM_REG_WRITE       = 0x27
CMD_DM_REG_SAVE        = 0x28
CMD_STATUS_REPLY       = 0x83
CMD_DM_SCAN_IDS_REPLY  = 0xA1
CMD_DM_SET_ID_REPLY    = 0xA2
CMD_DM_REG_READ_REPLY  = 0xA6
CMD_DM_REG_WRITE_REPLY = 0xA7

DIR_FORWARD  = 1
DIR_BACKWARD = 0
DIR_STOP     = 2
VEL_MAX_RAD_S = 14.0

# 电机配置 (须匹配固件 protocol.c)
MOTOR_MAP = {
    1: {"name": "arm_base",   "type": "DM4310",  "desc": "机械臂底部关节"},
}
ARM_IDS = [1]
ALL_IDS = [1]


# ── 协议帧 ───────────────────────────────────────────────────────────

def _checksum(length, cmd, payload):
    return (length + cmd + sum(payload)) & 0xFF

def make_frame(cmd, payload):
    length = len(payload)
    cs = _checksum(length, cmd, payload)
    return bytes([HEADER, length, cmd, *payload, cs])

def send_set_motor_vel_mrad(ser, motor_id, vel_mrad_s):
    v = max(-32768, min(32767, int(round(vel_mrad_s))))
    vb = struct.pack("<h", v)
    ser.write(make_frame(CMD_SET_MOTOR_VEL_MRAD, [motor_id, vb[0], vb[1]]))

def send_enable(ser, motor_id, enable):
    ser.write(make_frame(CMD_SET_ENABLE, [motor_id, 1 if enable else 0]))

def send_get_status(ser):
    ser.write(make_frame(CMD_GET_STATUS, []))

def send_stop_all(ser):
    ser.write(make_frame(CMD_STOP_ALL, []))

def send_dm_scan_ids(ser):
    ser.write(make_frame(CMD_DM_SCAN_IDS, []))

def send_set_gravity_comp(ser, motor_id, enable, kg, bias_rad, viscous, coulomb):
    p = bytearray([motor_id & 0xFF, 1 if enable else 0])
    p.extend(struct.pack("<f", float(kg)))
    p.extend(struct.pack("<f", float(bias_rad)))
    p.extend(struct.pack("<f", float(viscous)))
    p.extend(struct.pack("<f", float(coulomb)))
    ser.write(make_frame(CMD_SET_GRAVITY_COMP, p))

def send_rezero_gravity(ser, motor_id):
    ser.write(make_frame(CMD_REZERO_GRAVITY, [motor_id & 0xFF]))

def send_gateway_control(ser, motor_id, enable, p_des, v_des, kp, kd, t_ff,
                         target_position=0.0, target_velocity=0.0,
                         control_mode=1, motor_group=1):
    frame = bytearray(39)
    frame[0] = 0xAA; frame[1] = 0x55; frame[2] = 0x00; frame[3] = 0x20
    frame[4] = control_mode & 0xFF
    frame[5] = motor_group & 0xFF
    frame[6] = motor_id & 0xFF
    frame[7] = 1 if enable else 0
    frame[8:12]  = struct.pack("<f", float(p_des))
    frame[12:16] = struct.pack("<f", float(v_des))
    frame[16:20] = struct.pack("<f", float(kp))
    frame[20:24] = struct.pack("<f", float(kd))
    frame[24:28] = struct.pack("<f", float(t_ff))
    frame[28:32] = struct.pack("<f", float(target_position))
    frame[32:36] = struct.pack("<f", float(target_velocity))
    frame[36] = 0; frame[37] = 0
    frame[38] = sum(frame[2:38]) & 0xFF
    ser.write(frame)




# ── 应急停止 ──────────────────────────────────────────────────────────

def force_stop_all_motors(ser):
    for _ in range(2):
        for mid in ALL_IDS + [0xFF]:
            try:
                send_set_motor_vel_mrad(ser, mid, 0)
                send_enable(ser, mid, 0)
            except Exception:
                pass
        try:
            send_stop_all(ser)
        except Exception:
            pass
        time.sleep(0.02)


# ── 帧解析器 ─────────────────────────────────────────────────────────

class FrameParser:
    def __init__(self):
        self.reset()
    def reset(self):
        self.state = "header"; self.length = 0; self.cmd = 0
        self.payload = bytearray(); self.calc = 0
    def feed_byte(self, b):
        if self.state == "header":
            if b == HEADER: self.state = "length"
            return None
        if self.state == "length":
            self.length = b
            if b > 64: self.reset(); return None
            self.calc = b; self.state = "cmd"; return None
        if self.state == "cmd":
            self.cmd = b; self.calc = (self.calc + b) & 0xFF
            self.payload = bytearray()
            self.state = "checksum" if self.length == 0 else "data"
            return None
        if self.state == "data":
            self.payload.append(b); self.calc = (self.calc + b) & 0xFF
            if len(self.payload) >= self.length: self.state = "checksum"
            return None
        if self.state == "checksum":
            out = (self.cmd, bytes(self.payload)) if b == self.calc else None
            self.reset(); return out
        self.reset(); return None

def decode_status_reply(payload):
    if len(payload) < 16: return None
    status = payload[16] if len(payload) >= 17 else payload[15]
    return {
        "group": payload[0], "mode": payload[1], "id": payload[2],
        "enabled": payload[3],
        "pos": struct.unpack("<f", payload[4:8])[0],
        "vel": struct.unpack("<f", payload[8:12])[0],
        "tau": struct.unpack("<f", payload[12:16])[0],
        "status": status,
    }

def poll_stm32(ser, parser, budget_s=0.01, feedback_cb=None):
    end_t = time.time() + budget_s
    status_frame = None
    idle_count = 0
    while time.time() < end_t:
        try:
            waiting = getattr(ser, "in_waiting", 0)
            if waiting <= 0:
                idle_count += 1
                if idle_count > 2 and budget_s <= 0.02:
                    break  # 短预算时快速退出
                time.sleep(0.005)
                continue
            idle_count = 0
            raw = ser.read(waiting)
        except serial.SerialException:
            break
        if not raw: break
        for b in raw:
            frame = parser.feed_byte(b)
            if frame is None: continue
            cmd, payload = frame
            if cmd == CMD_STATUS_REPLY:
                status_frame = decode_status_reply(payload)
                if status_frame and feedback_cb:
                    feedback_cb(status_frame)
            elif cmd == CMD_DM_SCAN_IDS_REPLY and payload:
                raw_ids = list(payload[1:1+payload[0]])
                # master_id = can_id + 0x10, 去重并转换
                can_ids = sorted(set(
                    (rid - 0x10 if rid >= 0x10 else rid)
                    for rid in raw_ids if rid > 0
                ))
                print(f"\r\033[K[SCAN] CAN电机={can_ids} (原始帧id={raw_ids})")
    return status_frame


# ═══════════════════════════════════════════════════════════════════════
#  二、WHEELTEC 底盘
# ═══════════════════════════════════════════════════════════════════════

CAR_HEAD = 0x7B
CAR_TAIL = 0x7D
LEVELS_VX = [0, 100, 200, 300, 400, 500]
LEVELS_VZ = [0, 300, 500, 700, 900, 1200]

def _car_bcc(data: bytes) -> int:
    r = 0
    for b in data: r ^= b
    return r

def _car_make_frame(vx: int, vz: int) -> bytes:
    payload = (bytes([CAR_HEAD, 0x00, 0x00])
               + struct.pack('>h', max(-32768, min(32767, vx)))
               + struct.pack('>h', 0)
               + struct.pack('>h', max(-32768, min(32767, vz))))
    return payload + bytes([_car_bcc(payload), CAR_TAIL])

def _car_parse_fb(data: bytes):
    if len(data) != 24: return None
    if data[0] != CAR_HEAD or data[23] != CAR_TAIL: return None
    if _car_bcc(data[:22]) != data[22]: return None
    def s16(h, l):
        v = (h << 8) | l
        return v - 65536 if v > 32767 else v
    return {
        'flag_stop': data[1],
        'vx':   s16(data[2],  data[3])  / 1000.0,
        'vy':   s16(data[4],  data[5])  / 1000.0,
        'vz':   s16(data[6],  data[7])  / 1000.0,
        'ax':   s16(data[8],  data[9])  / 1672.0,
        'ay':   s16(data[10], data[11]) / 1672.0,
        'az':   s16(data[12], data[13]) / 1672.0,
        'gx':   s16(data[14], data[15]) / 3755.0,
        'gy':   s16(data[16], data[17]) / 3755.0,
        'gz':   s16(data[18], data[19]) / 3755.0,
        'volt': s16(data[20], data[21]) / 1000.0,
    }

class WheeltecCtrl:
    def __init__(self, port, baud=115200):
        self.ser = serial.Serial(port, baud, timeout=0.05)
        self.vx = 0; self.vz = 0
        self.feedback = None; self.running = True
        self._lock = threading.Lock()
        threading.Thread(target=self._send_loop, daemon=True).start()
        threading.Thread(target=self._recv_loop, daemon=True).start()
        print(f"\033[32m[OK] 底盘: {port}\033[0m")

    def set_vel(self, vx, vz):
        with self._lock:
            self.vx = max(-500, min(500, vx))
            self.vz = max(-1500, min(1500, vz))

    def stop(self):
        with self._lock: self.vx = self.vz = 0

    def _send_loop(self):
        while self.running:
            with self._lock: f = _car_make_frame(self.vx, self.vz)
            try: self.ser.write(f)
            except Exception: pass
            time.sleep(0.05)

    def _recv_loop(self):
        buf = bytearray()
        while self.running:
            try: buf.extend(self.ser.read(64))
            except Exception: break
            while len(buf) >= 24:
                idx = buf.find(CAR_HEAD)
                if idx < 0: buf.clear(); break
                if idx: del buf[:idx]
                if len(buf) < 24: break
                r = _car_parse_fb(bytes(buf[:24]))
                if r: self.feedback = r; del buf[:24]
                else: del buf[:1]

    def close(self):
        self.running = False; self.stop()
        time.sleep(0.12); self.ser.close()


# ═══════════════════════════════════════════════════════════════════════
#  三、摄像头 + Flask
# ═══════════════════════════════════════════════════════════════════════

try:
    import cv2
    from flask import Flask, Response, render_template_string, jsonify
except ImportError:
    cv2 = None
    Flask = Response = render_template_string = jsonify = None

try:
    from picamera2 import Picamera2
except ImportError:
    Picamera2 = None


class UsbCamera:
    """USB 摄像头类，支持颜色通道修正."""
    def __init__(self, dev_id: int, w: int, h: int, fps: int, color_order: str = "auto"):
        self.label = f"/dev/video{dev_id} (USB/UVC)"
        self.dev_id = dev_id
        self.color_order = color_order
        self.frame = None
        self.running = False
        self._lock = threading.Lock()
        cap = cv2.VideoCapture(dev_id)
        if not cap.isOpened():
            raise RuntimeError(f"/dev/video{dev_id} 打开失败")
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, w)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, h)
        cap.set(cv2.CAP_PROP_FPS, fps)
        self._cap = cap
        self.running = True
        threading.Thread(target=self._loop, daemon=True).start()
        print(f"\033[1;32m[OK] USB 摄像头: /dev/video{dev_id} (color={color_order})\033[0m")

    def _loop(self):
        while self.running:
            ret, frame = self._cap.read()
            if not ret:
                time.sleep(0.05); continue
            if len(frame.shape) == 3 and frame.shape[2] == 3:
                if self.color_order == "rgb":
                    frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            frame = cv2.rotate(frame, cv2.ROTATE_180)
            _, jpg = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 70])
            with self._lock:
                self.frame = jpg.tobytes()

    def get_frame(self):
        with self._lock:
            return self.frame

    def stop(self):
        self.running = False
        self._cap.release()


class CsiCamera:
    """与 wheeltec_all_in_one.py 完全一致的 CSI 摄像头类."""
    def __init__(self, w: int, h: int, fps: int, color_order: str = "rgb"):
        if Picamera2 is None:
            raise RuntimeError("未安装 python3-picamera2")
        self.label = "CSI Camera (Picamera2)"
        self.dev_id = "CSI"
        self.color_order = color_order
        self.frame = None
        self.running = False
        self._lock = threading.Lock()
        self._cam = Picamera2(0)
        config = self._cam.create_video_configuration(
            main={"size": (w, h), "format": "BGR888"},
            controls={"FrameRate": float(fps)}
        )
        self._cam.configure(config)
        self._cam.start()
        self.running = True
        threading.Thread(target=self._loop, daemon=True).start()
        print("\033[1;32m[OK] CSI 摄像头: Picamera2\033[0m")

    def _loop(self):
        while self.running:
            try:
                frame = self._cam.capture_array()
            except Exception:
                time.sleep(0.05); continue
            if len(frame.shape) == 3 and frame.shape[2] == 4:
                if self.color_order == "rgb":
                    frame = cv2.cvtColor(frame, cv2.COLOR_RGBA2BGR)
                else:
                    frame = cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)
            elif len(frame.shape) == 3 and frame.shape[2] == 3:
                if self.color_order == "rgb":
                    frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
                elif self.color_order == "auto":
                    pass
            frame = cv2.rotate(frame, cv2.ROTATE_180)
            ok, jpg = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 70])
            if not ok:
                continue
            with self._lock:
                self.frame = jpg.tobytes()

    def get_frame(self):
        with self._lock:
            return self.frame

    def stop(self):
        self.running = False
        try:
            self._cam.stop()
        except Exception:
            pass


def _detect_cams(max_id=6):
    """自动检测可读帧的 USB 摄像头."""
    found = []
    for i in range(max_id):
        c = cv2.VideoCapture(i)
        ok = False
        if c.isOpened():
            ok, _ = c.read()
        if c.isOpened() and ok:
            found.append(i)
        c.release()
    return found


_cameras = []
_car_ctrl = None
_last_fb_dict = {}   # motor_id -> feedback, shared with web

INDEX_HTML = """<!DOCTYPE html>
<html lang="zh">
<head>
<meta charset="utf-8">
<title>底盘 + 机械臂 控制台</title>
<style>
  *{box-sizing:border-box;margin:0;padding:0}
  body{background:#0d1117;color:#e6edf3;font-family:'Segoe UI',sans-serif;padding:16px}
  h1{font-size:20px;margin-bottom:14px;color:#58a6ff}
  .grid{display:flex;flex-wrap:wrap;gap:16px}
  .cam-box{flex:1;min-width:280px;background:#161b22;border:1px solid #30363d;
           border-radius:8px;overflow:hidden}
  .cam-box h3{padding:8px 12px;font-size:13px;background:#21262d;color:#8b949e}
  .cam-box img{width:100%;display:block}
  .status{flex:0 0 280px;background:#161b22;border:1px solid #30363d;
          border-radius:8px;padding:14px;font-size:13px;line-height:2}
  .status h3{font-size:14px;color:#58a6ff;margin-bottom:8px}
  .label{color:#8b949e;display:inline-block;min-width:60px}
  .val{color:#7ee787;font-weight:bold}
  .val-warn{color:#f0883e;font-weight:bold}
  .section{margin-top:12px;padding-top:8px;border-top:1px solid #30363d}
  .section h4{font-size:13px;color:#d2a8ff;margin-bottom:4px}
  .keys{background:#161b22;border:1px solid #30363d;border-radius:8px;
        padding:14px;margin-top:16px;font-size:12px;color:#8b949e;line-height:1.9}
  .keys h3{color:#58a6ff;font-size:14px;margin-bottom:6px}
  kbd{background:#21262d;border:1px solid #444;border-radius:4px;
      padding:1px 5px;font-size:12px;color:#e6edf3}
</style>
</head>
<body>
<h1>🚗🦾 底盘 + 机械臂 + 撑脚 控制台</h1>
<div class="grid">
  {% for i, label in cams %}
  <div class="cam-box">
    <h3>📷 摄像头 {{ loop.index }} — {{ label }}</h3>
    <img src="/stream/{{ i }}" alt="camera {{i}}">
  </div>
  {% endfor %}

  <div class="status">
    <h3>📊 小车状态</h3>
    <div id="car-box">
      <div style="color:#f0883e">⚠️ 等待底盘反馈<br>上电约 10 秒后自动连接</div>
    </div>

    <div class="section">
      <h4>🦾 机械臂</h4>
      <div id="arm-box">等待数据…</div>
    </div>
  </div>
</div>

<div class="keys">
  <h3>⌨️ SSH 终端按键 (长按运动，松开停车)</h3>
  <b>底盘</b>: <kbd>W</kbd>前进 <kbd>S</kbd>后退
  <kbd>A</kbd>左转 <kbd>D</kbd>右转
  <kbd>Q</kbd>加速档 <kbd>E</kbd>减速档<br>
  <b>机械臂</b>: <kbd>J</kbd>抬升 <kbd>K</kbd>下落
  <kbd>1</kbd><kbd>2</kbd>选电机 <kbd>0</kbd>全部
  <kbd>F</kbd>使能<br>
  <b>撑脚</b>: <kbd>T</kbd>伸出(抬车) <kbd>V</kbd>收回<br>
  <b>调参</b>: <kbd>[</kbd><kbd>]</kbd>KP
  <kbd>;</kbd><kbd>'</kbd>KD
  <kbd>,</kbd><kbd>.</kbd>KG
  <kbd>N</kbd><kbd>M</kbd>bias
  <kbd>G</kbd>重力 <kbd>B</kbd>零点 <kbd>C</kbd>校准
  <kbd>H</kbd>浮动 <kbd>L</kbd>锁存<br>
  <b>共用</b>: <kbd>Space</kbd>全停
  <kbd>P</kbd>底盘反馈 <kbd>R</kbd>臂状态
  <kbd>X</kbd>扫CAN <kbd>ESC</kbd>退出
</div>
<script>
async function refreshStatus() {
  try {
    const s = await (await fetch('/api/status')).json();
    const cb = document.getElementById('car-box');
    if (!s.car_available) {
      cb.innerHTML = '<div style="color:#f0883e">⚠️ 等待底盘反馈</div>';
    } else {
      const stopCls = s.car.flag_stop ? 'val-warn' : 'val';
      const voltCls = s.car.volt < 10.5 ? 'val-warn' : 'val';
      cb.innerHTML = `
        <div><span class="label">状态</span><span class="${stopCls}">${s.car.flag_stop ? '⛔ 失能' : '✅ 使能'}</span></div>
        <div><span class="label">电压</span><span class="${voltCls}">${s.car.volt.toFixed(2)} V</span></div>
        <div><span class="label">Vx</span><span class="val">${s.car.vx >= 0 ? '+' : ''}${s.car.vx.toFixed(3)} m/s</span></div>
        <div><span class="label">Vz</span><span class="val">${s.car.vz >= 0 ? '+' : ''}${s.car.vz.toFixed(3)} rad/s</span></div>
        <div><span class="label">加速度</span><span class="val">${s.car.ax.toFixed(1)}, ${s.car.ay.toFixed(1)}, ${s.car.az.toFixed(1)} m/s²</span></div>
        <div><span class="label">角速度</span><span class="val">${s.car.gz.toFixed(2)} rad/s</span></div>
      `;
    }
    const ab = document.getElementById('arm-box');
    if (s.arm && s.arm.length > 0) {
      ab.innerHTML = s.arm.map(m =>
        `<div>ID${m.id} ${m.enabled ? '✅' : '⬜'}
         θ:${(m.pos*57.3).toFixed(1)}°
         τ:${m.tau.toFixed(2)}Nm</div>`
      ).join('');
    } else {
      ab.innerHTML = '等待数据…';
    }
  } catch (_) {}
}
setInterval(refreshStatus, 500);
</script>
</body>
</html>"""

_flask_app = None

def init_web_and_cameras(args):
    """初始化摄像头 + Flask 网页，严格参照 wheeltec_all_in_one.py."""
    global _flask_app
    if cv2 is None or Flask is None:
        print("[错误] 缺少依赖: pip3 install flask opencv-python-headless")
        return

    app = Flask(__name__)
    _flask_app = app

    # ── USB 摄像头（自动检测或手动指定）──
    dev_ids = args.cam_devices if args.cam_devices else _detect_cams()
    if not dev_ids:
        print("[摄像头] 未检测到可读帧的 /dev/video* 设备")
    for dev_id in dev_ids:
        try:
            _cameras.append(UsbCamera(dev_id, args.cam_w, args.cam_h, args.cam_fps, args.usb_color))
        except Exception as e:
            print(f"  [跳过] video{dev_id}: {e}")

    # ── CSI 摄像头（默认启用，--no-csi 关闭）──
    if not getattr(args, 'no_csi', False):
        try:
            _cameras.append(CsiCamera(args.cam_w, args.cam_h, args.cam_fps, args.csi_color))
        except Exception as e:
            print(f"  [跳过] CSI 摄像头: {e}")
            if Picamera2 is None:
                print("  提示: sudo apt install -y python3-picamera2")

    # ── Flask 路由 ──
    @app.route('/')
    def index():
        fb = _car_ctrl.feedback if _car_ctrl else None
        cams_info = [
            (i, getattr(c, "label", f"/dev/video{getattr(c, 'dev_id', '?')}"))
            for i, c in enumerate(_cameras)
        ]
        return render_template_string(INDEX_HTML, fb=fb, cams=cams_info)

    @app.route('/stream/<int:idx>')
    def stream(idx):
        if idx < 0 or idx >= len(_cameras):
            return "not found", 404
        cam = _cameras[idx]
        def gen():
            while True:
                f = cam.get_frame()
                if f:
                    yield b'--frame\r\nContent-Type: image/jpeg\r\n\r\n' + f + b'\r\n'
                time.sleep(0.033)
        return Response(gen(), mimetype='multipart/x-mixed-replace; boundary=frame')

    @app.route('/api/status')
    def api_status():
        result = {"car_available": False, "arm": []}
        fb = _car_ctrl.feedback if _car_ctrl else None
        if fb:
            result["car_available"] = True
            result["car"] = {
                "flag_stop": int(fb["flag_stop"]),
                "vx": float(fb["vx"]),  "vz": float(fb["vz"]),
                "ax": float(fb.get("ax", 0)), "ay": float(fb.get("ay", 0)),
                "az": float(fb.get("az", 0)), "gz": float(fb.get("gz", 0)),
                "volt": float(fb["volt"]),
            }
        for mid in ALL_IDS:
            mfb = _last_fb_dict.get(mid)
            if mfb:
                result["arm"].append({
                    "id": mid, "enabled": int(mfb["enabled"]),
                    "pos": float(mfb["pos"]), "vel": float(mfb["vel"]),
                    "tau": float(mfb["tau"]), "status": int(mfb["status"]),
                })
        return jsonify(result)

    # ── 启动 Flask 后台线程（抑制请求日志）──
    logging.getLogger('werkzeug').setLevel(logging.ERROR)
    threading.Thread(
        target=lambda: app.run(host='0.0.0.0', port=args.web_port,
                               threaded=True, use_reloader=False),
        daemon=True).start()
    print(f"\033[1;33m[网页] http://192.168.10.2:{args.web_port}\033[0m")


# ═══════════════════════════════════════════════════════════════════════
#  四、主程序
# ═══════════════════════════════════════════════════════════════════════

BANNER = """\033[36m
╔═══════════════════════════════════════════════════════════╗
║  底盘 + 机械臂 + 撑脚 统一控制台                        ║
╠═══════════════════════════════════════════════════════════╣
║  底盘: W前 S后 A左转 D右转  Q/E 速度档+/-               ║
║  机械臂: J抬(松手锁位) K落  1/2选 0=全部               ║
║  撑脚: T伸出(抬车) V收回  (松开自停)                     ║
║  调参: [/] KP  ;/' KD  ,/. KG  N/M bias                 ║
║        G重力开关 B零点 C校准 H浮动 L锁存 Y复位bias      ║
║  F=紧急失能  Space全停  R状态  P底盘  X扫CAN  ESC退出   ║
╚═══════════════════════════════════════════════════════════╝\033[0m"""

CAR_DIR = {'w': (1,0), 's': (-1,0), 'a': (0,1), 'd': (0,-1)}


def main():
    ap = argparse.ArgumentParser(description="底盘+机械臂+撑脚 统一控制")
    ap.add_argument('--arm-port', default='/dev/ttyUSB0', help='STM32 串口')
    ap.add_argument('--arm-baud', type=int, default=115200)
    ap.add_argument('--car-port', default='/dev/ttyACM0', help='底盘串口')
    ap.add_argument('--car-baud', type=int, default=115200)
    ap.add_argument('--no-car', action='store_true', help='不连接底盘')
    ap.add_argument('--no-arm', action='store_true', help='不连接机械臂')
    ap.add_argument('--speed-rad-s', type=float, default=0.40, help='臂默认速度 rad/s')
    ap.add_argument('--status-ms', type=int, default=200, help='STM32状态轮询 ms')
    # 摄像头+网页（始终启动，用 --no-cam 关闭）
    ap.add_argument('--no-cam', action='store_true', help='禁用摄像头+网页')
    ap.add_argument('--cam-devices', type=int, nargs='+', help='USB摄像头编号（默认自动检测）')
    ap.add_argument('--usb-color', choices=['auto','bgr','rgb'], default='rgb',
                    help='USB摄像头颜色通道 (偏红用rgb修正)')
    ap.add_argument('--no-csi', action='store_true', help='不启用CSI摄像头')
    ap.add_argument('--csi-color', choices=['auto','bgr','rgb'], default='rgb')
    ap.add_argument('--cam-w', type=int, default=640)
    ap.add_argument('--cam-h', type=int, default=480)
    ap.add_argument('--cam-fps', type=int, default=30)
    ap.add_argument('--web-port', type=int, default=8080)
    args = ap.parse_args()

    global _car_ctrl
    # ── 初始化底盘 ────────────────────────────────────────────
    car = None
    if not args.no_car:
        try:
            car = WheeltecCtrl(args.car_port, args.car_baud)
            _car_ctrl = car
        except Exception as e:
            print(f"\033[31m[底盘] 串口失败: {e}\033[0m (用 --no-car 跳过)")

    # ── 初始化 STM32 ─────────────────────────────────────────
    arm_ser = None
    if not args.no_arm:
        try:
            arm_ser = serial.Serial(args.arm_port, args.arm_baud,
                                    timeout=0.1, dsrdtr=False, rtscts=False)
            arm_ser.setDTR(False); arm_ser.setRTS(False)
            print(f"\033[32m[OK] STM32: {args.arm_port}\033[0m")
        except Exception as e:
            print(f"\033[31m[STM32] 串口失败: {e}\033[0m (用 --no-arm 跳过)")

    # ── 摄像头 + 网页 ──────────────────────────────────────────
    if not args.no_cam:
        init_web_and_cameras(args)

    if arm_ser is None and car is None:
        print("底盘和机械臂都未连接，退出。")
        return 1

    # ── 机械臂调参状态 ────────────────────────────────────────
    parser_state = FrameParser()
    last_fb = {}  # motor_id -> feedback dict
    status_period_s = args.status_ms / 1000.0 if args.status_ms > 0 else None

    tune = {
        mid: {"kp": 0.0, "kd": 0.0, "kg": 0.0, "bias": 0.0,
              "viscous": 0.0, "coulomb": 0.0, "grav_en": True,
              "_zero_pos": 0.0, "_calibrated": False}
        for mid in ARM_IDS
    }
    # DM4310 defaults
    tune[1]["kp"]=12.0; tune[1]["kd"]=1.5; tune[1]["kg"]=0.55

    hold_speed = max(0.05, min(1.2, args.speed_rad_s))
    arm_motor = 1  # 当前选中电机（按1/2切换，0=全部）
    arm_enabled = True   # 默认全程使能，F紧急失能
    arm_drive_active = False
    arm_last_key_t = 0.0
    car_level = 3
    car_last_key_t = 0.0
    car_last_dir = None
    car_keys_held = {}  # key -> last press time，支持斜向运动

    CAR_HOLD_TIMEOUT = 0.06   # 底盘松键超时（OS键重复~30ms，防方向残影）
    ARM_HOLD_TIMEOUT = 0.40   # 臂松键超时（保证点按有明显位移）
    arm_drive_vel = 0.0        # 当前臂目标速度（持续刷新用）

    def on_feedback(fr):
        last_fb[int(fr["id"])] = fr
        _last_fb_dict[int(fr["id"])] = fr  # share with web API

    def cur_tune(mid):
        if mid == 0xFF: return tune[1]
        return tune.get(mid, tune[1])

    def push_tune(mid, enable=None):
        """Send MIT hold command (v_des=0, hold at latched/current position)."""
        def _push(i):
            ts = tune[i]
            fb = last_fb.get(i)
            p = ts.get("_latched", fb["pos"] if fb else 0.0)
            en = arm_enabled if enable is None else bool(enable)
            send_gateway_control(arm_ser, i, en, p, 0.0, ts["kp"], ts["kd"], 0.0)
            send_set_gravity_comp(arm_ser, i, ts["grav_en"], ts["kg"],
                                  ts["bias"], ts["viscous"], ts["coulomb"])
        if mid == 0xFF:
            for i in ARM_IDS:
                _push(i)
                time.sleep(0.06)  # 等待固件 DM_Multi_EnableCompat 完成(~35ms)，避免 UART overrun 丢帧
        else:
            _push(mid)

    def drive_arm(vel_rads):
        """A/D: send velocity to firmware ramp generator (10ms tick ramps p_des)."""
        mids = ARM_IDS if arm_motor == 0xFF else [arm_motor]
        for mid in mids:
            send_set_motor_vel_mrad(arm_ser, mid, int(vel_rads * 1000))

    def hold_arm():
        """Release: send vel=0, firmware latches current position and holds."""
        mids = ARM_IDS if arm_motor == 0xFF else [arm_motor]
        for mid in mids:
            send_set_motor_vel_mrad(arm_ser, mid, 0)

    def do_rezero(mid):
        send_rezero_gravity(arm_ser, mid)
        for i in (ARM_IDS if mid==0xFF else [mid]):
            fb = last_fb.get(i)
            if fb: tune[i]["_zero_pos"] = fb["pos"]

    stop_done = [False]
    def ensure_stopped():
        if stop_done[0]: return
        stop_done[0] = True
        if arm_ser:
            try: force_stop_all_motors(arm_ser)
            except Exception: pass
        if car:
            try: car.stop()
            except Exception: pass

    def _sig(signum, _):
        ensure_stopped(); raise SystemExit(128+signum)
    signal.signal(signal.SIGINT, _sig)
    signal.signal(signal.SIGTERM, _sig)

    # ── 键盘循环 ─────────────────────────────────────────────
    print(BANNER)
    if car: print(f"  底盘速度档: {car_level} (Vx≈{LEVELS_VX[car_level]}mm/s)")
    print(f"  臂速度: {hold_speed:.2f} rad/s")

    # 等待 STM32 启动，自动使能所有臂电机
    if arm_ser:
        time.sleep(0.8); arm_ser.flushInput()
        # 先读几帧反馈获取当前位置
        for _ in range(5):
            send_get_status(arm_ser)
            poll_stm32(arm_ser, parser_state, budget_s=0.05, feedback_cb=on_feedback)
        # 使能所有臂电机并锁定当前位置
        push_tune(0xFF, enable=True)
        print("\033[K[INIT] 机械臂已使能并锁位。A/D转动, 松手定住。F=紧急失能")

    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    tty.setcbreak(fd)
    last_status_t = 0.0

    try:
        while True:
            now = time.time()
            rlist, _, _ = select.select([sys.stdin], [], [], 0.02)

            # ── 松手超时检测 ──
            if car and car_keys_held:
                car_keys_held = {k: t for k, t in car_keys_held.items() if now - t <= CAR_HOLD_TIMEOUT}
                if not car_keys_held:
                    car.stop(); car_last_dir = None
            if arm_ser and arm_drive_active and (now - arm_last_key_t) > ARM_HOLD_TIMEOUT:
                hold_arm(); arm_drive_active = False
            elif arm_ser and arm_drive_active:
                drive_arm(arm_drive_vel)  # 持续刷新，确保点按也有响应
            # ── STM32 心跳 + 反馈 ──
            if arm_ser:
                # 看门狗保活：每400ms发一次标准帧（看门狗超时500ms，gateway帧不重置它）
                _kp_iv = status_period_s if (status_period_s and status_period_s < 0.4) else 0.4
                if (now - last_status_t) >= _kp_iv:
                    send_get_status(arm_ser); last_status_t = now
                poll_stm32(arm_ser, parser_state, budget_s=0.005, feedback_cb=on_feedback)

            if not rlist:
                # 状态行
                parts = []
                mt = "ALL" if arm_motor==0xFF else f"M{arm_motor}"
                parts.append(f"[{mt}]{'EN' if arm_enabled else 'DIS'}")
                ts = cur_tune(arm_motor)
                parts.append(f"kp:{ts['kp']:.1f} kd:{ts['kd']:.2f} kg:{ts['kg']:.2f}")
                fbk = last_fb.get(arm_motor if arm_motor!=0xFF else 1)
                if fbk:
                    parts.append(f"θ:{fbk['pos']*57.3:+.1f}° τ:{fbk['tau']:+.2f}Nm")
                if car and car.feedback:
                    parts.append(f"CAR:{car.vx:+d}/{car.vz:+d}")
                print(f"\r\033[K{' | '.join(parts)}", end="", flush=True)
                continue

            ch = sys.stdin.read(1).lower()

            # ── 退出 ──
            if ch == '\x1b':
                ensure_stopped(); print("\n退出。"); break

            # ── 全停 ──
            if ch == ' ':
                if arm_ser: force_stop_all_motors(arm_ser); arm_enabled = False; arm_drive_active = False
                if car: car.stop(); car_last_dir = None
                print(f"\r\033[K[STOP] 全部停止! 按A/D重新使能运动")
                continue

            # ── 底盘 W/A/S/D（支持斜向：同时按住前进+左/右转）──
            if ch in CAR_DIR and car:
                car_keys_held[ch] = now
                car_last_key_t = now; car_last_dir = ch
                dx_t = max(-1, min(1, sum(CAR_DIR[k][0] for k in car_keys_held)))
                dz_t = max(-1, min(1, sum(CAR_DIR[k][1] for k in car_keys_held)))
                car.set_vel(dx_t * LEVELS_VX[car_level], dz_t * LEVELS_VZ[car_level])

            # ── 底盘速度 Q/E ──
            elif ch == 'q':
                car_level = min(len(LEVELS_VX)-1, car_level+1)
                print(f"\r\033[K[CAR] 速度档 ↑ {car_level} Vx≈{LEVELS_VX[car_level]}mm/s")
            elif ch == 'e':
                car_level = max(1, car_level-1)
                print(f"\r\033[K[CAR] 速度档 ↓ {car_level} Vx≈{LEVELS_VX[car_level]}mm/s")

            # ── 电机选择 ──
            elif ch in ['1','2'] and arm_ser:
                arm_motor = int(ch); arm_drive_active = False
                m = MOTOR_MAP.get(arm_motor, {})
                print(f"\r\033[K[SEL] M{arm_motor} {m.get('type','?')} {m.get('desc','?')}")
            elif ch == '0' and arm_ser:
                arm_motor = 0xFF; arm_drive_active = False
                print(f"\r\033[K[SEL] ALL 臂电机(M1+M2)")

            # ── F 紧急失能（所有臂电机）──
            elif ch == 'f' and arm_ser:
                force_stop_all_motors(arm_ser)
                arm_enabled = False; arm_drive_active = False
                print(f"\r\033[K[DIS] 所有电机已失能! 按A/D重新使能运动")

            # ── 臂 J/K 运动（按住运动，松手锁位）──
            elif ch == 'j' and arm_ser:
                if not arm_enabled:
                    arm_enabled = True  # 重新使能：push_tune重建CAN使能态
                    push_tune(arm_motor)
                    if arm_motor != 0xFF:
                        time.sleep(0.06)  # 单电机：等DM_Multi_EnableCompat(~35ms)完成
                arm_drive_vel = hold_speed
                drive_arm(arm_drive_vel)
                arm_drive_active = True; arm_last_key_t = now
            elif ch == 'k' and arm_ser:
                if not arm_enabled:
                    arm_enabled = True  # 重新使能：push_tune重建CAN使能态
                    push_tune(arm_motor)
                    if arm_motor != 0xFF:
                        time.sleep(0.06)  # 单电机：等DM_Multi_EnableCompat(~35ms)完成
                arm_drive_vel = -hold_speed
                drive_arm(arm_drive_vel)
                arm_drive_active = True; arm_last_key_t = now

            # ── 调参 ──
            elif ch == 'g' and arm_ser:
                ts = cur_tune(arm_motor); ts["grav_en"] = not ts["grav_en"]
                push_tune(arm_motor)
            elif ch == 'b' and arm_ser:
                do_rezero(arm_motor)
            elif ch == 'c' and arm_ser:
                for mid in (ARM_IDS if arm_motor==0xFF else [arm_motor]):
                    fb = last_fb.get(mid)
                    if fb: tune[mid]["_zero_pos"]=fb["pos"]; tune[mid]["_calibrated"]=True
                do_rezero(arm_motor)
                print(f"\r\033[K[CAL] 水平零点已校准")
            elif ch == 'y' and arm_ser:
                for mid in (ARM_IDS if arm_motor==0xFF else [arm_motor]):
                    tune[mid]["bias"] = 0.0
                push_tune(arm_motor)
                print(f"\r\033[K[BIAS] 已复位")
            elif ch == 'h' and arm_ser:
                for mid in (ARM_IDS if arm_motor==0xFF else [arm_motor]):
                    tune[mid]["kp"]=0.0; tune[mid]["kd"]=0.0
                push_tune(arm_motor)
                print(f"\r\033[K[FLOAT] KP=0 KD=0 仅重力补偿. ,/.调KG → E锁存 → ]加KP")
            elif ch == 'l' and arm_ser:
                for mid in (ARM_IDS if arm_motor==0xFF else [arm_motor]):
                    fb = last_fb.get(mid)
                    if fb: tune[mid]["_latched"] = fb["pos"]
                push_tune(arm_motor)
                print(f"\r\033[K[LATCH] 位置已锁存, ]增加KP")
            elif ch == '[' and arm_ser:
                ts = cur_tune(arm_motor); ts["kp"] = max(0.0, ts["kp"]-0.5); push_tune(arm_motor)
            elif ch == ']' and arm_ser:
                ts = cur_tune(arm_motor); ts["kp"] = min(80.0, ts["kp"]+0.5); push_tune(arm_motor)
            elif ch == ';' and arm_ser:
                ts = cur_tune(arm_motor); ts["kd"] = max(0.0, ts["kd"]-0.05); push_tune(arm_motor)
            elif ch == "'" and arm_ser:
                ts = cur_tune(arm_motor); ts["kd"] = min(15.0, ts["kd"]+0.05); push_tune(arm_motor)
            elif ch == ',' and arm_ser:
                ts = cur_tune(arm_motor); ts["kg"] = max(-8.0, ts["kg"]-0.02); push_tune(arm_motor)
            elif ch == '.' and arm_ser:
                ts = cur_tune(arm_motor); ts["kg"] = min(8.0, ts["kg"]+0.02); push_tune(arm_motor)
            elif ch == 'n' and arm_ser:
                ts = cur_tune(arm_motor); ts["bias"] = max(-3.2, ts["bias"]-0.005); push_tune(arm_motor)
            elif ch == 'm' and arm_ser:
                ts = cur_tune(arm_motor); ts["bias"] = min(3.2, ts["bias"]+0.005); push_tune(arm_motor)

            # ── 状态 ──
            elif ch == 'r' and arm_ser:
                send_get_status(arm_ser)
            elif ch == 'p' and car:
                f = car.feedback
                if f:
                    print(f"\r\033[K[CAR] {'停止' if f['flag_stop'] else '使能'} "
                          f"电压:{f['volt']:.1f}V Vx:{f['vx']:+.3f}m/s Vz:{f['vz']:+.3f}r/s")
                else:
                    print(f"\r\033[K[CAR] 等待反馈（上电约10秒）")
            elif ch == 'x' and arm_ser:
                print(f"\r\033[K[SCAN] 正在扫描CAN总线...")
                send_dm_scan_ids(arm_ser)
                time.sleep(0.15)  # 等待STM32扫描31个ID (~62ms)
                poll_stm32(arm_ser, parser_state, budget_s=0.2, feedback_cb=on_feedback)

    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        ensure_stopped()
        if arm_ser:
            try: arm_ser.close()
            except Exception: pass
        if car:
            try: car.close()
            except Exception: pass
        for c in _cameras:
            try: c.stop()
            except Exception: pass
        print("已退出。")

    return 0


if __name__ == '__main__':
    raise SystemExit(main())
