#!/usr/bin/env python3
"""
2D Path Viewer Node — real-time OpenCV window showing the Duckiebot trajectory.

Subscribes to:
  /{veh}/pose            (nav_msgs/Odometry)          — wheel encoder odometry  (blue)
  /orb_slam/camera_poses (std_msgs/Float64MultiArray)  — ORB-SLAM visual poses   (green)
  /sensor_fusion/pose    (nav_msgs/Odometry)          — fused EKF pose          (red)

Opens a separate OpenCV window (independent of the Pangolin 3D viewer) that
draws the 2D path (top-down, bird's-eye view) for each source in a different
colour so you can compare them live.

Controls:
  r  — reset / clear all paths
  +  — zoom in
  -  — zoom out
  q  — quit
"""

import math
import threading

import cv2
import numpy as np
import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray


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


class PathViewerNode:
    """Draws 2D paths from odometry, ORB-SLAM and sensor fusion in an OpenCV window."""

    def __init__(self):
        rospy.init_node("path_viewer_node", anonymous=False)

        self._veh = rospy.get_param("~veh", "duckiebot21")
        self._lock = threading.Lock()

        # Paths: list of (x, y) in metres
        self._path_odom  = []
        self._path_slam  = []
        self._path_fused = []

        # View parameters
        self._pixels_per_metre = 200.0   # zoom level
        self._origin_x = WINDOW_W / 2.0  # pixel position of world origin
        self._origin_y = WINDOW_H / 2.0

        # ---- Subscribers ----
        odom_topic  = rospy.get_param("~odom_topic",  f"/{self._veh}/pose")
        slam_topic  = rospy.get_param("~slam_topic",  "/orb_slam/camera_poses")
        fused_topic = rospy.get_param("~fused_topic", "/sensor_fusion/pose")

        rospy.Subscriber(odom_topic,  Odometry,           self._odom_cb,  queue_size=1)
        rospy.Subscriber(slam_topic,  Float64MultiArray,  self._slam_cb,  queue_size=1)
        rospy.Subscriber(fused_topic, Odometry,           self._fused_cb, queue_size=1)

        rospy.loginfo("[PathViewerNode] Subscribed — odom: %s, slam: %s, fused: %s",
                      odom_topic, slam_topic, fused_topic)

        # SLAM scale (reuse sensor_fusion scale param or default)
        self._slam_scale = rospy.get_param("~slam_scale", 1.0)

        # Start the OpenCV rendering loop in a separate thread
        self._running = True
        self._render_thread = threading.Thread(target=self._render_loop, daemon=True)
        self._render_thread.start()

        rospy.on_shutdown(self._shutdown)

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    @staticmethod
    def _pose2d_from_odom(msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        return (x, y)

    @staticmethod
    def _pose2d_from_slam_matrix(mat4x4):
        """Camera x → world x, camera z → world y (ground plane)."""
        return (mat4x4[0, 3], mat4x4[2, 3])

    def _world_to_pixel(self, wx, wy):
        """Convert world (metres) to pixel coordinates."""
        px = int(self._origin_x + wx * self._pixels_per_metre)
        py = int(self._origin_y - wy * self._pixels_per_metre)  # y-axis flipped
        return (px, py)

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------

    def _odom_cb(self, msg):
        pt = self._pose2d_from_odom(msg)
        with self._lock:
            self._path_odom.append(pt)

    def _slam_cb(self, msg):
        data = np.array(msg.data, dtype=np.float64)
        if len(data) < 16:
            return
        latest = data[-16:].reshape(4, 4)
        pt = self._pose2d_from_slam_matrix(latest)
        with self._lock:
            self._path_slam.append(pt)

    def _fused_cb(self, msg):
        pt = self._pose2d_from_odom(msg)
        with self._lock:
            self._path_fused.append(pt)

    # ------------------------------------------------------------------
    # Rendering
    # ------------------------------------------------------------------

    def _draw_path(self, canvas, path, color, thickness=2):
        """Draw a polyline on the canvas."""
        if len(path) < 2:
            return
        pts = [self._world_to_pixel(x, y) for x, y in path]
        for i in range(1, len(pts)):
            cv2.line(canvas, pts[i - 1], pts[i], color, thickness, cv2.LINE_AA)
        # Draw current position as a circle
        cv2.circle(canvas, pts[-1], 5, color, -1, cv2.LINE_AA)

    def _draw_legend(self, canvas):
        """Draw colour legend in the top-left corner."""
        y0 = 25
        for i, (label, color) in enumerate(LEGEND):
            y = y0 + i * 25
            cv2.line(canvas, (15, y), (40, y), color, 3, cv2.LINE_AA)
            cv2.putText(canvas, label, (50, y + 5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1, cv2.LINE_AA)

    def _draw_grid(self, canvas):
        """Draw a subtle metre grid."""
        step = self._pixels_per_metre  # 1 metre grid
        if step < 20:
            step = self._pixels_per_metre * 5
        grid_color = (55, 55, 55)

        # Vertical lines
        x = self._origin_x % step
        while x < WINDOW_W:
            cv2.line(canvas, (int(x), 0), (int(x), WINDOW_H), grid_color, 1)
            x += step
        # Horizontal lines
        y = self._origin_y % step
        while y < WINDOW_H:
            cv2.line(canvas, (0, int(y)), (WINDOW_W, int(y)), grid_color, 1)
            y += step

        # Origin cross
        ox, oy = int(self._origin_x), int(self._origin_y)
        cv2.line(canvas, (ox - 10, oy), (ox + 10, oy), (100, 100, 100), 1)
        cv2.line(canvas, (ox, oy - 10), (ox, oy + 10), (100, 100, 100), 1)

    def _draw_info(self, canvas):
        """Draw zoom and point count info."""
        with self._lock:
            n_o = len(self._path_odom)
            n_s = len(self._path_slam)
            n_f = len(self._path_fused)
        info = f"Zoom: {self._pixels_per_metre:.0f} px/m | Odom: {n_o} | SLAM: {n_s} | Fused: {n_f}"
        cv2.putText(canvas, info, (15, WINDOW_H - 15),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (140, 140, 140), 1, cv2.LINE_AA)
        cv2.putText(canvas, "Keys: r=reset  +/-=zoom  q=quit", (15, WINDOW_H - 35),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (100, 100, 100), 1, cv2.LINE_AA)

    def _render_loop(self):
        """Main OpenCV render loop running at ~30 fps."""
        cv2.namedWindow("Duckiebot 2D Path Viewer", cv2.WINDOW_AUTOSIZE)

        while self._running and not rospy.is_shutdown():
            canvas = np.full((WINDOW_H, WINDOW_W, 3), BG_COLOR, dtype=np.uint8)

            self._draw_grid(canvas)

            with self._lock:
                path_odom  = list(self._path_odom)
                path_slam  = list(self._path_slam)
                path_fused = list(self._path_fused)

            self._draw_path(canvas, path_odom,  COLOR_ODOM,  2)
            self._draw_path(canvas, path_slam,  COLOR_SLAM,  2)
            self._draw_path(canvas, path_fused, COLOR_FUSED, 2)

            self._draw_legend(canvas)
            self._draw_info(canvas)

            cv2.imshow("Duckiebot 2D Path Viewer", canvas)

            key = cv2.waitKey(33) & 0xFF  # ~30 fps
            if key == ord('q'):
                rospy.signal_shutdown("User quit path viewer")
                break
            elif key == ord('r'):
                with self._lock:
                    self._path_odom.clear()
                    self._path_slam.clear()
                    self._path_fused.clear()
                rospy.loginfo("[PathViewerNode] Paths reset.")
            elif key == ord('+') or key == ord('='):
                self._pixels_per_metre = min(self._pixels_per_metre * 1.3, 2000)
            elif key == ord('-'):
                self._pixels_per_metre = max(self._pixels_per_metre / 1.3, 20)

        cv2.destroyAllWindows()

    def _shutdown(self):
        self._running = False


if __name__ == "__main__":
    PathViewerNode()
    rospy.spin()
