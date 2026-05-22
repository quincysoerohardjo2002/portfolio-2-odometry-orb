#!/usr/bin/env python3
"""
Standalone 2D Path Viewer (no ROS required).

Connects to the Duckiebot's sensor_fusion_node TCP stream (port 9998)
and displays a real-time bird's-eye view of the Duckiebot trajectory.

Three paths are shown:
  - Blue   : Wheel encoder odometry
  - Green  : ORB-SLAM (visual, scaled)
  - Red    : Fused EKF estimate

Usage:
    python3 path_viewer_standalone.py [duckiebot-ip]

Default IP: 192.168.0.166 (duckiebot21)

Controls:
  r  — reset view
  +  — zoom in
  -  — zoom out
  q  — quit
"""

import pickle
import socket
import struct
import sys
import threading
import time

import cv2
import numpy as np

# ---------------------------------------------------------------------------
# Drawing constants
# ---------------------------------------------------------------------------
WINDOW_W = 800
WINDOW_H = 800
BG_COLOR = (30, 30, 30)

COLOR_ODOM  = (255, 180, 0)    # blue-ish (BGR)
COLOR_SLAM  = (0, 220, 0)      # green
COLOR_FUSED = (0, 0, 255)      # red

LEGEND = [
    ("Odometry (encoders)", COLOR_ODOM),
    ("ORB-SLAM (visual)",   COLOR_SLAM),
    ("Fused (EKF)",         COLOR_FUSED),
]


# ---------------------------------------------------------------------------
# Network receiver
# ---------------------------------------------------------------------------

def recv_exact(sock, n):
    """Receive exactly n bytes from socket."""
    data = b''
    while len(data) < n:
        chunk = sock.recv(n - len(data))
        if not chunk:
            raise ConnectionError("Connection closed")
        data += chunk
    return data


def network_receiver(host, port, state, lock, running):
    """Connect to Duckiebot sensor_fusion_node and update shared state."""
    print(f"[Receiver] Connecting to {host}:{port}...")
    while running[0]:
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.settimeout(5.0)
            sock.connect((host, port))
            sock.settimeout(None)
            print(f"[Receiver] Connected to {host}:{port}")

            while running[0]:
                header = recv_exact(sock, 4)
                length = struct.unpack('!I', header)[0]
                payload = recv_exact(sock, length)
                data = pickle.loads(payload)

                with lock:
                    if 'path_odom' in data:
                        state['path_odom'] = data['path_odom']
                    if 'path_slam' in data:
                        state['path_slam'] = data['path_slam']
                    if 'path_fused' in data:
                        state['path_fused'] = data['path_fused']

        except (ConnectionError, ConnectionRefusedError, OSError) as e:
            print(f"[Receiver] Connection lost: {e}. Retrying in 2s...")
            time.sleep(2)
        except Exception as e:
            print(f"[Receiver] Error: {e}")
            time.sleep(2)


# ---------------------------------------------------------------------------
# Drawing helpers
# ---------------------------------------------------------------------------

class PathRenderer:
    def __init__(self):
        self.pixels_per_metre = 200.0
        self.origin_x = WINDOW_W / 2.0
        self.origin_y = WINDOW_H / 2.0

    def world_to_pixel(self, wx, wy):
        px = int(self.origin_x + wx * self.pixels_per_metre)
        py = int(self.origin_y - wy * self.pixels_per_metre)
        return (px, py)

    def draw_path(self, canvas, path, color, thickness=2):
        if len(path) < 2:
            if len(path) == 1:
                pt = self.world_to_pixel(path[0][0], path[0][1])
                cv2.circle(canvas, pt, 5, color, -1, cv2.LINE_AA)
            return
        pts = [self.world_to_pixel(x, y) for x, y in path]
        for i in range(1, len(pts)):
            cv2.line(canvas, pts[i - 1], pts[i], color, thickness, cv2.LINE_AA)
        cv2.circle(canvas, pts[-1], 5, color, -1, cv2.LINE_AA)

    def draw_grid(self, canvas):
        step = self.pixels_per_metre
        if step < 20:
            step = self.pixels_per_metre * 5
        grid_color = (55, 55, 55)

        x = self.origin_x % step
        while x < WINDOW_W:
            cv2.line(canvas, (int(x), 0), (int(x), WINDOW_H), grid_color, 1)
            x += step
        y = self.origin_y % step
        while y < WINDOW_H:
            cv2.line(canvas, (0, int(y)), (WINDOW_W, int(y)), grid_color, 1)
            y += step

        ox, oy = int(self.origin_x), int(self.origin_y)
        cv2.line(canvas, (ox - 10, oy), (ox + 10, oy), (100, 100, 100), 1)
        cv2.line(canvas, (ox, oy - 10), (ox, oy + 10), (100, 100, 100), 1)

    def draw_legend(self, canvas):
        y0 = 25
        for i, (label, color) in enumerate(LEGEND):
            y = y0 + i * 25
            cv2.line(canvas, (15, y), (40, y), color, 3, cv2.LINE_AA)
            cv2.putText(canvas, label, (50, y + 5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1, cv2.LINE_AA)

    def draw_info(self, canvas, n_odom, n_slam, n_fused, connected):
        status = "CONNECTED" if connected else "WAITING..."
        info = f"{status} | Zoom: {self.pixels_per_metre:.0f} px/m | Odom: {n_odom} | SLAM: {n_slam} | Fused: {n_fused}"
        cv2.putText(canvas, info, (15, WINDOW_H - 15),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (140, 140, 140), 1, cv2.LINE_AA)
        cv2.putText(canvas, "Keys: r=reset  +/-=zoom  q=quit", (15, WINDOW_H - 35),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (100, 100, 100), 1, cv2.LINE_AA)

    def zoom_in(self):
        self.pixels_per_metre = min(self.pixels_per_metre * 1.3, 2000)

    def zoom_out(self):
        self.pixels_per_metre = max(self.pixels_per_metre / 1.3, 20)


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    host = sys.argv[1] if len(sys.argv) > 1 else "192.168.0.166"
    port = 9998

    print("=" * 52)
    print(" Duckiebot 2D Path Viewer (Sensor Fusion)")
    print("=" * 52)
    print(f" Duckiebot: {host}:{port}")
    print(" Blue=Odometry  Green=ORB-SLAM  Red=Fused(EKF)")
    print("=" * 52)

    state = {'path_odom': [], 'path_slam': [], 'path_fused': []}
    lock = threading.Lock()
    running = [True]

    # Start network receiver in background thread
    recv_thread = threading.Thread(
        target=network_receiver, args=(host, port, state, lock, running), daemon=True
    )
    recv_thread.start()

    renderer = PathRenderer()
    cv2.namedWindow("Duckiebot 2D Path Viewer", cv2.WINDOW_AUTOSIZE)

    while True:
        canvas = np.full((WINDOW_H, WINDOW_W, 3), BG_COLOR, dtype=np.uint8)
        renderer.draw_grid(canvas)

        with lock:
            path_odom = list(state['path_odom'])
            path_slam = list(state['path_slam'])
            path_fused = list(state['path_fused'])

        connected = len(path_odom) > 0 or len(path_slam) > 0 or len(path_fused) > 0

        renderer.draw_path(canvas, path_odom, COLOR_ODOM, 2)
        renderer.draw_path(canvas, path_slam, COLOR_SLAM, 2)
        renderer.draw_path(canvas, path_fused, COLOR_FUSED, 2)

        renderer.draw_legend(canvas)
        renderer.draw_info(canvas, len(path_odom), len(path_slam), len(path_fused), connected)

        cv2.imshow("Duckiebot 2D Path Viewer", canvas)

        key = cv2.waitKey(33) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('r'):
            with lock:
                state['path_odom'] = []
                state['path_slam'] = []
                state['path_fused'] = []
            print("[Viewer] Paths reset.")
        elif key == ord('+') or key == ord('='):
            renderer.zoom_in()
        elif key == ord('-'):
            renderer.zoom_out()

    running[0] = False
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
