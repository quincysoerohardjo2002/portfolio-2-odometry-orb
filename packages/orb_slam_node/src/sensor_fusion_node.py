#!/usr/bin/env python3
"""
Sensor Fusion Node — Extended Kalman Filter combining wheel odometry and ORB-SLAM.

Subscribes to:
  /{veh}/pose               (nav_msgs/Odometry)          — wheel encoder odometry
  /orb_slam/camera_poses    (std_msgs/Float64MultiArray)  — ORB-SLAM camera poses

Publishes:
  /sensor_fusion/pose       (nav_msgs/Odometry)           — fused pose estimate

The EKF uses odometry as the prediction model (high-frequency, smooth, drifts)
and ORB-SLAM visual pose as the measurement update (lower-frequency, noisy but
drift-correcting).  A running scale factor maps monocular ORB-SLAM units to
metric odometry units.
"""

import math
import pickle
import socket
import struct
import threading

import numpy as np
import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray


def _normalize_angle(a):
    """Wrap angle to [-pi, pi]."""
    return (a + math.pi) % (2.0 * math.pi) - math.pi


class SensorFusionNode:
    """EKF sensor fusion: wheel odometry (predict) + ORB-SLAM (correct)."""

    def __init__(self):
        rospy.init_node("sensor_fusion_node", anonymous=False)

        self._veh = rospy.get_param("~veh", "duckiebot21")

        # ---- EKF state: [x, y, theta] in metres / radians ----
        self._x = np.zeros(3)                        # state mean
        self._P = np.diag([0.01, 0.01, 0.01])        # state covariance

        # Process noise (odometry uncertainty per step)
        self._Q = np.diag([
            rospy.get_param("~q_x", 0.002),
            rospy.get_param("~q_y", 0.002),
            rospy.get_param("~q_theta", 0.005),
        ])

        # Measurement noise (ORB-SLAM uncertainty)
        self._R_meas = np.diag([
            rospy.get_param("~r_x", 0.05),
            rospy.get_param("~r_y", 0.05),
            rospy.get_param("~r_theta", 0.1),
        ])

        self._lock = threading.Lock()

        # ---- Odometry tracking ----
        self._odom_prev = None          # previous odometry message (x, y, theta)
        self._odom_initialized = False

        # ---- ORB-SLAM tracking ----
        self._slam_prev_pose2d = None   # previous SLAM 2D pose (x, y, theta)
        self._slam_initialized = False

        # ---- Scale estimation (monocular SLAM → metres) ----
        self._scale = 1.0
        self._scale_alpha = rospy.get_param("~scale_alpha", 0.1)  # EMA smoothing
        self._odom_accum_dist = 0.0     # accumulated odometry distance for scale
        self._slam_accum_dist = 0.0     # accumulated SLAM distance for scale
        self._scale_window = rospy.get_param("~scale_window", 0.05)  # min dist before update

        # ---- Path history for TCP streaming ----
        self._path_odom = []   # [(x, y)] from encoder odometry
        self._path_slam = []   # [(x, y)] from ORB-SLAM
        self._path_fused = []  # [(x, y)] from EKF fusion

        # ---- TCP streaming server (port 9998) ----
        self._tcp_port = rospy.get_param("~tcp_port", 9998)
        self._tcp_clients = []
        self._tcp_lock = threading.Lock()
        self._start_tcp_server()

        # ---- Publisher ----
        self._fused_pub = rospy.Publisher(
            "/sensor_fusion/pose", Odometry, queue_size=1
        )

        # ---- Subscribers ----
        odom_topic = rospy.get_param("~odom_topic", f"/{self._veh}/pose")
        slam_topic = rospy.get_param("~slam_topic", "/orb_slam/camera_poses")

        rospy.Subscriber(odom_topic, Odometry, self._odom_cb, queue_size=1)
        rospy.Subscriber(slam_topic, Float64MultiArray, self._slam_cb, queue_size=1)

        rospy.loginfo(
            "[SensorFusionNode] Ready — odom: %s, slam: %s", odom_topic, slam_topic
        )

    # ------------------------------------------------------------------
    # TCP streaming server
    # ------------------------------------------------------------------

    def _start_tcp_server(self):
        server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server.bind(('0.0.0.0', self._tcp_port))
        server.listen(5)
        server.settimeout(1.0)
        t = threading.Thread(target=self._tcp_accept_loop, args=(server,), daemon=True)
        t.start()
        rospy.loginfo("[SensorFusionNode] TCP stream on port %d", self._tcp_port)

    def _tcp_accept_loop(self, server):
        while not rospy.is_shutdown():
            try:
                client, addr = server.accept()
                client.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                with self._tcp_lock:
                    self._tcp_clients.append(client)
                rospy.loginfo("[SensorFusionNode] Viewer connected from %s", addr)
            except socket.timeout:
                continue
            except Exception:
                break

    def _tcp_broadcast(self):
        with self._lock:
            data = pickle.dumps({
                'path_odom': self._path_odom[-2000:],
                'path_slam': self._path_slam[-2000:],
                'path_fused': self._path_fused[-2000:],
            }, protocol=pickle.HIGHEST_PROTOCOL)
        header = struct.pack('!I', len(data))
        with self._tcp_lock:
            dead = []
            for i, client in enumerate(self._tcp_clients):
                try:
                    client.sendall(header + data)
                except (BrokenPipeError, ConnectionResetError, OSError):
                    dead.append(i)
            for i in reversed(dead):
                self._tcp_clients[i].close()
                del self._tcp_clients[i]

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    @staticmethod
    def _pose2d_from_odom(msg):
        """Extract (x, y, theta) from nav_msgs/Odometry."""
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        # quaternion → yaw
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        theta = 2.0 * math.atan2(qz, qw)
        return np.array([x, y, theta])

    @staticmethod
    def _pose2d_from_slam_matrix(mat4x4):
        """Extract 2D pose (x, y, theta) from a 4×4 camera-to-world matrix.

        Convention: camera x → world x, camera z → world y (ground plane).
        Heading = atan2(R[2,0], R[0,0]) projected onto the ground plane.
        """
        x = mat4x4[0, 3]
        y = mat4x4[2, 3]
        theta = math.atan2(mat4x4[2, 0], mat4x4[0, 0])
        return np.array([x, y, theta])

    # ------------------------------------------------------------------
    # EKF predict (odometry)
    # ------------------------------------------------------------------

    def _ekf_predict(self, delta):
        """Predict step: propagate state with odometry delta [dx, dy, dtheta]."""
        theta = self._x[2]
        ct, st = math.cos(theta), math.sin(theta)

        # Rotate delta into world frame
        dx_w = ct * delta[0] - st * delta[1]
        dy_w = st * delta[0] + ct * delta[1]

        self._x[0] += dx_w
        self._x[1] += dy_w
        self._x[2] = _normalize_angle(self._x[2] + delta[2])

        # Jacobian of the motion model w.r.t. state
        F = np.eye(3)
        F[0, 2] = -st * delta[0] - ct * delta[1]
        F[1, 2] =  ct * delta[0] - st * delta[1]

        self._P = F @ self._P @ F.T + self._Q

    # ------------------------------------------------------------------
    # EKF update (ORB-SLAM measurement)
    # ------------------------------------------------------------------

    def _ekf_update(self, z_delta):
        """Update step: correct state with scaled ORB-SLAM pose delta."""
        # Measurement model: H = I (direct observation of delta pose)
        H = np.eye(3)
        y = z_delta - np.array([0.0, 0.0, 0.0])  # innovation (vs predicted delta = 0 after predict already applied)

        # We correct the state by the difference between the SLAM-observed
        # delta and the odometry-predicted delta.  Since predict already moved
        # the state, the "expected measurement" is zero additional correction.
        S = H @ self._P @ H.T + self._R_meas
        K = self._P @ H.T @ np.linalg.inv(S)

        self._x = self._x + K @ y
        self._x[2] = _normalize_angle(self._x[2])
        self._P = (np.eye(3) - K @ H) @ self._P

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------

    def _odom_cb(self, msg):
        """Wheel odometry callback — EKF prediction step."""
        pose2d = self._pose2d_from_odom(msg)

        with self._lock:
            if not self._odom_initialized:
                self._odom_prev = pose2d.copy()
                self._x[:] = pose2d
                self._odom_initialized = True
                rospy.loginfo("[SensorFusionNode] Odometry initialized.")
                return

            # Compute delta in body frame
            dx_global = pose2d[0] - self._odom_prev[0]
            dy_global = pose2d[1] - self._odom_prev[1]
            dtheta = _normalize_angle(pose2d[2] - self._odom_prev[2])

            # Rotate global delta back to body frame of previous pose
            ct = math.cos(self._odom_prev[2])
            st = math.sin(self._odom_prev[2])
            dx_body = ct * dx_global + st * dy_global
            dy_body = -st * dx_global + ct * dy_global

            delta = np.array([dx_body, dy_body, dtheta])

            # Track distance for scale estimation
            self._odom_accum_dist += math.sqrt(dx_global**2 + dy_global**2)

            self._odom_prev = pose2d.copy()
            self._path_odom.append((pose2d[0], pose2d[1]))
            self._ekf_predict(delta)

        self._publish_fused()
        self._tcp_broadcast()

    def _slam_cb(self, msg):
        """ORB-SLAM poses callback — EKF update step."""
        data = np.array(msg.data, dtype=np.float64)
        if len(data) < 16:
            return

        n_poses = len(data) // 16
        latest = data[-16:].reshape(4, 4)
        pose2d = self._pose2d_from_slam_matrix(latest)

        with self._lock:
            if not self._odom_initialized:
                return  # wait for odometry to initialise the state first

            if not self._slam_initialized:
                self._slam_prev_pose2d = pose2d.copy()
                self._slam_initialized = True
                rospy.loginfo("[SensorFusionNode] ORB-SLAM initialized.")
                return

            # Compute SLAM delta
            dx = pose2d[0] - self._slam_prev_pose2d[0]
            dy = pose2d[1] - self._slam_prev_pose2d[1]
            dtheta = _normalize_angle(pose2d[2] - self._slam_prev_pose2d[2])
            self._slam_prev_pose2d = pose2d.copy()

            slam_dist = math.sqrt(dx**2 + dy**2)
            self._slam_accum_dist += slam_dist

            # ---- Update scale factor ----
            if (self._odom_accum_dist > self._scale_window
                    and self._slam_accum_dist > 1e-6):
                new_scale = self._odom_accum_dist / self._slam_accum_dist
                self._scale = (
                    self._scale_alpha * new_scale
                    + (1.0 - self._scale_alpha) * self._scale
                )
                self._odom_accum_dist = 0.0
                self._slam_accum_dist = 0.0
                rospy.loginfo_throttle(
                    5.0, "[SensorFusionNode] Scale: %.4f", self._scale
                )

            # Scale the SLAM delta to metric units
            scaled_delta = np.array([
                dx * self._scale,
                dy * self._scale,
                dtheta,  # rotation has no scale ambiguity
            ])

            # The EKF predict already moved the state by odometry.
            # The SLAM update provides an independent delta observation.
            # Correction = scaled_slam_delta − odometry_delta (already applied).
            # We pass the residual as the innovation.
            correction = scaled_delta  # full SLAM delta as observation
            self._ekf_update(correction)

            self._path_slam.append((pose2d[0] * self._scale, pose2d[1] * self._scale))

        self._publish_fused()
        self._tcp_broadcast()

    # ------------------------------------------------------------------
    # Publish
    # ------------------------------------------------------------------

    def _publish_fused(self):
        """Publish the fused pose as nav_msgs/Odometry."""
        with self._lock:
            x, y, theta = self._x

        odom = Odometry()
        odom.header.stamp = rospy.Time.now()
        odom.header.frame_id = "map"
        odom.child_frame_id = "base_link"

        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.position.z = 0.0

        odom.pose.pose.orientation.x = 0.0
        odom.pose.pose.orientation.y = 0.0
        odom.pose.pose.orientation.z = math.sin(theta / 2.0)
        odom.pose.pose.orientation.w = math.cos(theta / 2.0)

        # Publish covariance (6×6, only fill x/y/yaw)
        P = self._P
        cov = [0.0] * 36
        cov[0] = P[0, 0]   # x-x
        cov[1] = P[0, 1]   # x-y
        cov[5] = P[0, 2]   # x-yaw
        cov[6] = P[1, 0]   # y-x
        cov[7] = P[1, 1]   # y-y
        cov[11] = P[1, 2]  # y-yaw
        cov[30] = P[2, 0]  # yaw-x
        cov[31] = P[2, 1]  # yaw-y
        cov[35] = P[2, 2]  # yaw-yaw
        odom.pose.covariance = cov

        self._fused_pub.publish(odom)

        with self._lock:
            self._path_fused.append((x, y))


if __name__ == "__main__":
    SensorFusionNode()
    rospy.spin()
