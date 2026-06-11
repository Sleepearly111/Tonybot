#!/usr/bin/env python3
"""
RPLIDAR A1M8 实时可视化工具
=============================

用法:
    python lidar_viewer.py COM3          # Windows, 指定串口
    python lidar_viewer.py /dev/ttyUSB0  # Linux/Mac

功能:
    - 实时极坐标图（雷达扫描视图）
    - 四个方向距离数字显示
    - 物体检测覆盖标记
    - 支持回放日志文件

ESP32 端输出格式（每行一个扫描点）:
    <angle> <distance_mm> <quality>

依赖: pip install pyserial matplotlib numpy
"""

import sys
import time
import struct
import threading
from collections import deque

import numpy as np
import matplotlib
import matplotlib.pyplot as plt
import matplotlib.animation as animation

matplotlib.use("TkAgg")

# ============================================================
# 配置
# ============================================================
BAUDRATE = 115200
SCAN_MARKER = b"SCAN"
END_MARKER = b"END"
MAX_POINTS = 360
DIST_MAX = 4000  # mm，显示范围上限

# ============================================================
# 数据读取
# ============================================================
class ScanReader:
    """从串口或日志文件读取 LiDAR 扫描数据"""

    def __init__(self, source):
        """
        source: 串口设备路径 (str)，或日志文件对象
        """
        self.source = source
        self.serial_port = None
        self.file = None
        self._open()

    def _open(self):
        if isinstance(self.source, str):
            import serial
            self.serial_port = serial.Serial(self.source, BAUDRATE, timeout=1)
            self.serial_port.reset_input_buffer()
        else:
            self.file = self.source

    def read_line(self):
        """读取一行，返回 str 或 None"""
        if self.serial_port:
            try:
                line = self.serial_port.readline()
                if line:
                    return line.decode("utf-8", errors="ignore").strip()
            except (serial.SerialException, UnicodeDecodeError):
                pass
            return None
        elif self.file:
            line = self.file.readline()
            return line.strip() if line else None
        return None

    def close(self):
        if self.serial_port:
            self.serial_port.close()


# ============================================================
# 可视化
# ============================================================
class LidarViewer:
    def __init__(self, reader: ScanReader):
        self.reader = reader
        self.angles = np.linspace(0, 2 * np.pi, MAX_POINTS)
        self.distances = np.full(MAX_POINTS, np.nan)
        self.qualities = np.zeros(MAX_POINTS)
        self.ts = 0
        self.running = True

        # -- 图形设置 --
        self.fig = plt.figure(figsize=(10, 9))
        self.fig.canvas.manager.set_window_title("LiDAR Viewer — Tonybot")

        # 极坐标子图
        self.ax = self.fig.add_subplot(111, projection="polar")
        self.ax.set_theta_zero_location("N")   # 上方 = 0°
        self.ax.set_theta_direction(-1)         # 顺时针
        self.ax.set_thetagrids([0, 90, 180, 270],
                               ["前 0°", "右 90°", "后 180°", "左 270°"],
                               fontsize=9)
        self.ax.set_rlim(0, DIST_MAX)
        self.ax.set_rticks([500, 1000, 1500, 2000, 3000, 4000])
        self.ax.set_title("LiDAR 实时扫描 — 等待数据...", fontsize=12, pad=20)

        self.scatter = self.ax.scatter([], [], s=3, c=[], cmap="RdYlGn",
                                       vmin=0, vmax=255, alpha=0.8)
        self.cbar = self.fig.colorbar(self.scatter, ax=self.ax, label="信号质量")

        # -- 方向文字 --
        self.text_front  = self.fig.text(0.02, 0.85, "前 : ---", fontsize=11, family="monospace")
        self.text_right  = self.fig.text(0.02, 0.82, "右 : ---", fontsize=11, family="monospace")
        self.text_back   = self.fig.text(0.02, 0.79, "后 : ---", fontsize=11, family="monospace")
        self.text_left   = self.fig.text(0.02, 0.76, "左 : ---", fontsize=11, family="monospace")
        self.text_status = self.fig.text(0.02, 0.72, "", fontsize=10, family="monospace", color="gray")

    def update_data(self):
        """从串口读取数据，解析扫描点"""
        new_scan = False
        buf = []
        ts_now = 0

        while self.running:
            line = self.reader.read_line()
            if not line:
                break

            # 新扫描开始标记
            if line.startswith("SCAN"):
                parts = line.split()
                if len(parts) >= 3:
                    ts_now = int(parts[1])
                buf = []
                continue

            # 扫描结束标记
            if line == "END":
                if buf:
                    self._apply_points(buf, ts_now)
                    new_scan = True
                break

            # 扫描点行: angle dist quality
            parts = line.split()
            if len(parts) >= 2:
                try:
                    angle = int(float(parts[0]))
                    dist = float(parts[1])
                    qual = int(parts[2]) if len(parts) >= 3 else 0
                    buf.append((angle, dist, qual))
                except ValueError:
                    pass

        return new_scan

    def _apply_points(self, points, ts):
        """将一组点应用到当前地图"""
        self.distances.fill(np.nan)
        self.qualities.fill(0)

        for angle, dist, qual in points:
            idx = angle % MAX_POINTS
            if 0 < dist < DIST_MAX * 2:
                self.distances[idx] = dist
                self.qualities[idx] = qual

        self.ts = ts

    def animate(self, frame):
        """matplotlib animation 回调"""
        new = self.update_data()

        # 等待新扫描
        if not new and np.all(np.isnan(self.distances)):
            self.ax.set_title("LiDAR 实时扫描 — 等待数据...", fontsize=12, pad=20)
            return [self.scatter]

        # 有效数据 → 更新图
        mask = ~np.isnan(self.distances)
        angles = self.angles[mask]
        dists = self.distances[mask]
        quals = self.qualities[mask]

        self.scatter.set_offsets(np.column_stack([angles, dists]))
        self.scatter.set_array(quals)

        self.ax.set_title(f"LiDAR 实时扫描 — {len(dists)} 点 | t={self.ts}ms", fontsize=12, pad=20)

        # 四个方向
        def fmt(v):
            return f"{v:.0f}mm" if not np.isnan(v) and v > 0 else "---"

        self.text_front.set_text(f"前 (0°)   : {fmt(self.distances[0])}")
        self.text_right.set_text(f"右 (90°)  : {fmt(self.distances[90])}")
        self.text_back.set_text(f"后 (180°) : {fmt(self.distances[180])}")
        self.text_left.set_text(f"左 (270°) : {fmt(self.distances[270])}")

        # 检测前方物体
        nearest = np.inf
        nearest_ang = -1
        for i in list(range(300, 360)) + list(range(0, 61)):
            d = self.distances[i]
            if not np.isnan(d) and 50 < d < nearest:
                nearest = d
                nearest_ang = i

        if nearest_ang >= 0:
            self.text_status.set_text(
                f"前方物体: {nearest_ang}° {nearest:.0f}mm"
            )
        else:
            self.text_status.set_text("前方物体: 无")

        return [self.scatter]

    def run(self):
        """主循环"""
        ani = animation.FuncAnimation(
            self.fig, self.animate, interval=50, blit=False, cache_frame_data=False
        )
        plt.tight_layout()
        plt.show()


# ============================================================
# 日志录制模式
# ============================================================
def record_mode(port, output_file):
    """录制串口数据到日志文件"""
    import serial
    ser = serial.Serial(port, BAUDRATE, timeout=1)
    print(f"[REC] 录制 {port} → {output_file}  (Ctrl+C 停止)")

    with open(output_file, "w") as f:
        try:
            while True:
                line = ser.readline().decode("utf-8", errors="ignore").strip()
                if line:
                    f.write(line + "\n")
                    f.flush()
                    if "END" in line:
                        print(".", end="", flush=True)
        except KeyboardInterrupt:
            print(f"\n[REC] 已保存到 {output_file}")
    ser.close()


# ============================================================
# main
# ============================================================
if __name__ == "__main__":
    if len(sys.argv) < 2:
        print(__doc__)
        print("示例:")
        print("  python lidar_viewer.py COM3              # 实时显示")
        print("  python lidar_viewer.py COM3 --record     # 录制日志")
        print("  python lidar_viewer.py scan.log          # 回放日志")
        sys.exit(1)

    source = sys.argv[1]

    # --record 模式：录制
    if len(sys.argv) >= 3 and sys.argv[2] == "--record":
        record_mode(source, "scan_" + time.strftime("%Y%m%d_%H%M%S") + ".log")
        sys.exit(0)

    # 回放 .log 文件 or 实时串口
    if source.endswith(".log"):
        f = open(source, "r")
        reader = ScanReader(f)
    else:
        reader = ScanReader(source)

    viewer = LidarViewer(reader)
    try:
        viewer.run()
    except KeyboardInterrupt:
        pass
    finally:
        reader.close()
