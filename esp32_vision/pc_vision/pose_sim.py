"""
pose_sim.py - phat pose gia de test khi chua co camera/ArUco that.

Dung dung giao thuc that cua he thong:
    pose:  <x_cm>;<y_cm>;<theta_rad>#
    x, y   tinh bang CENTIMET
    theta  tinh bang RADIAN, duong = nguoc chieu kim dong ho nhin tu tren xuong

Hai cho co the cam vao:

  --target direct  (mac dinh)
      Gui thang vao STM32 qua USB-TTL noi chan PC7 (USART6_RX).
      Khong can ESP32 nao. Day la cach nhanh nhat de xac nhan rieng
      firmware chay dung, tach hoan toan khoi phan truyen khong day.
          python pose_sim.py COM7

  --target gateway
      Gui vao cong Serial cua esp32C, them tien to ID robot o dau goi
      dung nhu Python that van lam:  29;<x_cm>;<y_cm>;<theta_rad>#
      Dung de test ca chuoi gateway -> ESP-NOW -> esp32R -> STM32.
          python pose_sim.py COM5 --target gateway

Cac che do quy dao:
    --mode static   dung yen tai (100cm, 200cm, 45 do)
    --mode circle   chay vong tron gia ban kinh 50cm, giu nguyen huong
    --mode step     nhay giua 4 diem moi 3 giay - de nhin ro luc STM32 snap

Kiem tra ket qua o dau kia: mo terminal UART cua STM32 (115200) va xem dong
    PLINK,<valid>,<age_ms>,<x>,<y>,<theta>,<ok>,<other>,<err>,<drop>
valid phai = 1, ok tang deu, err va drop dung yen. Dong POSE,... phai bam
theo dung gia tri phat di, nhung da doi sang MET va DO.
"""

import argparse
import math
import sys
import time

try:
    import serial
except ImportError:
    sys.exit("Thieu pyserial. Cai bang: pip install pyserial")


def gen_static(_t):
    # x_cm, y_cm, theta_rad
    return 100.0, 200.0, math.radians(45.0)


def gen_circle(t, radius_cm=50.0, omega=0.3):
    phi = omega * t
    x = radius_cm * math.sin(phi)
    y = radius_cm * (1.0 - math.cos(phi))
    # giu nguyen huong than xe - giong bai test cung tron giu huong truoc day
    return x, y, 0.0


def gen_step(t):
    pts = [
        (0.0, 0.0, 0.0),
        (100.0, 0.0, math.pi / 2),
        (100.0, 100.0, math.pi),
        (0.0, 100.0, -math.pi / 2),
    ]
    return pts[int(t // 3) % len(pts)]


MODES = {"static": gen_static, "circle": gen_circle, "step": gen_step}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("port", help="cong COM, vd COM7 hoac /dev/ttyUSB0")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--hz", type=float, default=20.0, help="tan so phat pose")
    ap.add_argument("--mode", choices=MODES.keys(), default="static")
    ap.add_argument("--target", choices=["direct", "gateway"], default="direct")
    ap.add_argument("--id", type=int, default=29, help="robot ID khi --target gateway")
    args = ap.parse_args()

    gen = MODES[args.mode]
    period = 1.0 / args.hz
    prefix = f"{args.id};" if args.target == "gateway" else ""

    with serial.Serial(args.port, args.baud, timeout=0.1) as ser:
        if args.target == "gateway":
            time.sleep(2.0)   # ESP32 tu reset khi mo cong serial

        print(f"Phat pose '{args.mode}' -> {args.port} ({args.target}) @ {args.hz}Hz. Ctrl+C de dung.")
        t0 = time.time()
        n = 0
        try:
            while True:
                t = time.time() - t0
                x_cm, y_cm, th_rad = gen(t)

                line = "%s%.1f;%.1f;%.5f#" % (prefix, x_cm, y_cm, th_rad)
                ser.write(line.encode("ascii"))

                n += 1
                if n % int(max(args.hz, 1)) == 0:
                    print(f"  t={t:6.1f}s  x={x_cm:7.1f}cm  y={y_cm:7.1f}cm  "
                          f"th={math.degrees(th_rad):7.2f}deg  (sent={n})")

                time.sleep(period)
        except KeyboardInterrupt:
            print("\nDung phat pose.")


if __name__ == "__main__":
    main()
