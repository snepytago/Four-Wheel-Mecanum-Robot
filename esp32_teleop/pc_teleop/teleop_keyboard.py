#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
teleop_keyboard.py
------------------------------------------------------------------------
Điều khiển robot mecanum bằng bàn phím theo thời gian thực, chạy trên PC.

Luồng dữ liệu:
    Bàn phím (PC) -> script này -> Serial/USB -> ESP32 "server"
    -> ESP-NOW -> ESP32 "trên robot" -> UART -> STM32F401RE

Cách dùng:
    python3 teleop_keyboard.py --port COM5           (Windows)
    python3 teleop_keyboard.py --port /dev/ttyUSB0    (Linux)
    python3 teleop_keyboard.py --port /dev/cu.usbserial-XXXX  (macOS)

    Không truyền --port -> script tự liệt kê cổng serial đang có để chọn.

Phím điều khiển (giữ phím = robot tiếp tục di chuyển, thả ra = dừng trục đó):
    W / S       : tiến / lùi            (vx)
    A / D       : dịch ngang trái/phải  (vy)   -- đặc trưng mecanum
    Q / E       : xoay trái / xoay phải (wz)
    W+A, W+D... : đi chéo (giữ đồng thời 2 phím)
    R / F       : tăng / giảm tốc độ dài  (V_LIN)
    T / G       : tăng / giảm tốc độ xoay (V_ANG)
    SPACE       : dừng khẩn cấp ngay lập tức
    ESC         : thoát chương trình (tự gửi lệnh dừng trước khi thoát)

An toàn:
    - Script gửi lệnh liên tục ở tần số cố định (mặc định 20Hz) kể cả khi
      không có phím nào được giữ (gửi 0,0,0) -- đây là tín hiệu heartbeat
      để ESP32 server / ESP32 robot / STM32 phát hiện mất kết nối.
    - Khi thoát (ESC, Ctrl+C, đóng cửa sổ terminal...) script cố gắng gửi
      vài gói lệnh dừng trước khi đóng cổng serial.
    - Tốc độ mặc định để THẤP (0.15 m/s, 0.4 rad/s) vì firmware STM32
      hiện tại là feed-forward thuần (chưa có PID theo encoder) -- tốc độ
      càng cao, sai số bám vận tốc đặt càng lớn (xem báo cáo tiến độ,
      mục 14). Tăng dần bằng phím R/T sau khi đã thử phản ứng của robot.

Cài thư viện cần thiết:
    pip install pyserial pynput

Lưu ý về pynput trên Linux: cần có phiên làm việc đồ hoạ (X11/Wayland
qua xwayland) đang chạy, KHÔNG chạy được qua SSH thuần không có X. Trên
macOS lần đầu chạy sẽ được hỏi cấp quyền Accessibility/Input Monitoring
cho Terminal/IDE -- cần đồng ý.
------------------------------------------------------------------------
"""

import argparse
import sys
import time
import threading

try:
    import serial
    from serial.tools import list_ports
except ImportError:
    print("Thieu thu vien pyserial. Cai bang: pip install pyserial")
    sys.exit(1)

try:
    from pynput import keyboard
except ImportError:
    print("Thieu thu vien pynput. Cai bang: pip install pynput")
    sys.exit(1)


# ======================= CẤU HÌNH MẶC ĐỊNH ================================

DEFAULT_BAUD = 115200
SEND_HZ = 20.0                 # tần số gửi lệnh (heartbeat), 20Hz = 50ms/lần
SEND_PERIOD_S = 1.0 / SEND_HZ

V_LIN_DEFAULT = 0.15           # m/s  - tốc độ dài mặc định (THẤP, an toàn)
V_ANG_DEFAULT = 0.4            # rad/s - tốc độ xoay mặc định
V_LIN_STEP = 0.05
V_ANG_STEP = 0.1
V_LIN_MAX = 1.0
V_ANG_MAX = 2.0

STOP_BURST_ON_EXIT = 5         # số gói lệnh dừng gửi liên tiếp khi thoát


HELP_TEXT = """
==================== DIEU KHIEN ROBOT BANG BAN PHIM ====================
  W / S     : tien / lui              (vx)
  A / D     : dich ngang trai / phai  (vy)
  Q / E     : xoay trai / xoay phai   (wz)
  (co the giu nhieu phim cung luc de di cheo, vd W + D)
  R / F     : tang / giam toc do dai  (V_LIN)
  T / G     : tang / giam toc do xoay (V_ANG)
  SPACE     : dung khan cap
  ESC       : thoat chuong trinh
==========================================================================
"""


def list_serial_ports():
    ports = list(list_ports.comports())
    return ports


def pick_port_interactively():
    ports = list_serial_ports()
    if not ports:
        print("Khong tim thay cong serial nao. Cam ESP32 server vao USB roi thu lai.")
        sys.exit(1)
    print("Cac cong serial dang co:")
    for i, p in enumerate(ports):
        print(f"  [{i}] {p.device}  ({p.description})")
    idx = input("Chon so thu tu cong ket noi ESP32 server: ").strip()
    try:
        idx = int(idx)
        return ports[idx].device
    except (ValueError, IndexError):
        print("Lua chon khong hop le.")
        sys.exit(1)


class KeyState:
    """Theo dõi trạng thái các phím đang được giữ, thread-safe."""

    def __init__(self):
        self._lock = threading.Lock()
        self._held = set()
        self.v_lin = V_LIN_DEFAULT
        self.v_ang = V_ANG_DEFAULT
        self.emergency_stop = False
        self.quit = False

    def on_press(self, key):
        try:
            if key == keyboard.Key.esc:
                self.quit = True
                return
            if key == keyboard.Key.space:
                with self._lock:
                    self.emergency_stop = True
                return

            c = key.char.lower() if hasattr(key, "char") and key.char else None
            if c is None:
                return

            with self._lock:
                if c in ("w", "a", "s", "d", "q", "e"):
                    self._held.add(c)
                    self.emergency_stop = False
                elif c == "r":
                    self.v_lin = min(V_LIN_MAX, self.v_lin + V_LIN_STEP)
                    print(f"[toc do dai] V_LIN = {self.v_lin:.2f} m/s")
                elif c == "f":
                    self.v_lin = max(0.0, self.v_lin - V_LIN_STEP)
                    print(f"[toc do dai] V_LIN = {self.v_lin:.2f} m/s")
                elif c == "t":
                    self.v_ang = min(V_ANG_MAX, self.v_ang + V_ANG_STEP)
                    print(f"[toc do xoay] V_ANG = {self.v_ang:.2f} rad/s")
                elif c == "g":
                    self.v_ang = max(0.0, self.v_ang - V_ANG_STEP)
                    print(f"[toc do xoay] V_ANG = {self.v_ang:.2f} rad/s")
        except AttributeError:
            pass

    def on_release(self, key):
        try:
            c = key.char.lower() if hasattr(key, "char") and key.char else None
            if c is None:
                return
            with self._lock:
                self._held.discard(c)
        except AttributeError:
            pass

    def compute_velocity(self):
        """Trả về (vx, vy, wz) tính bằng m/s, m/s, rad/s theo trạng thái phím hiện tại."""
        with self._lock:
            if self.emergency_stop:
                return 0.0, 0.0, 0.0

            held = set(self._held)
            v_lin = self.v_lin
            v_ang = self.v_ang

        vx = 0.0
        vy = 0.0
        wz = 0.0

        if "w" in held:
            vx += v_lin
        if "s" in held:
            vx -= v_lin
        if "a" in held:
            vy += v_lin
        if "d" in held:
            vy -= v_lin
        if "e" in held:
            wz -= v_ang
        if "q" in held:
            wz += v_ang

        return vx, vy, wz


def encode_line(vx_ms, vy_ms, wz_rads):
    """Chuyển (m/s, m/s, rad/s) -> chuỗi 'V,vx_mm,vy_mm,wz_mrad\\n' cho ESP32 server."""
    vx_mm = int(round(vx_ms * 1000))
    vy_mm = int(round(vy_ms * 1000))
    wz_mrad = int(round(wz_rads * 1000))
    return f"V,{vx_mm},{vy_mm},{wz_mrad}\n"


def main():
    parser = argparse.ArgumentParser(description="Dieu khien robot mecanum bang ban phim qua ESP32 server.")
    parser.add_argument("--port", type=str, default=None, help="Cong serial cua ESP32 server (vd COM5, /dev/ttyUSB0)")
    parser.add_argument("--baud", type=int, default=DEFAULT_BAUD, help="Baudrate (mac dinh 115200)")
    args = parser.parse_args()

    port = args.port or pick_port_interactively()

    print(f"Dang mo cong {port} @ {args.baud} baud...")
    try:
        ser = serial.Serial(port, args.baud, timeout=0)
    except serial.SerialException as ex:
        print(f"Khong mo duoc cong serial: {ex}")
        sys.exit(1)

    time.sleep(2.0)  # cho ESP32 reset xong sau khi mo cong serial (DTR toggle)

    state = KeyState()
    listener = keyboard.Listener(on_press=state.on_press, on_release=state.on_release)
    listener.start()

    print(HELP_TEXT)
    print(f"Dang gui lenh toi {port} o {SEND_HZ:.0f}Hz. Nhan ESC de thoat.\n")

    try:
        while not state.quit:
            t0 = time.time()

            vx, vy, wz = state.compute_velocity()
            line = encode_line(vx, vy, wz)
            try:
                ser.write(line.encode("ascii"))
            except serial.SerialException as ex:
                print(f"Loi ghi serial: {ex}")
                break

            elapsed = time.time() - t0
            time.sleep(max(0.0, SEND_PERIOD_S - elapsed))
    except KeyboardInterrupt:
        pass
    finally:
        print("\nDang dung robot va thoat...")
        for _ in range(STOP_BURST_ON_EXIT):
            try:
                ser.write(encode_line(0.0, 0.0, 0.0).encode("ascii"))
            except serial.SerialException:
                break
            time.sleep(0.03)
        listener.stop()
        ser.close()
        print("Da thoat.")


if __name__ == "__main__":
    main()