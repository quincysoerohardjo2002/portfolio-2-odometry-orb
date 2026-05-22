#!/usr/bin/env python3

import pickle
import socket
import struct
import threading

import cv2
import numpy as np
import rospy
import std_msgs.msg
from sensor_msgs import point_cloud2 as pc2
from sensor_msgs.msg import CameraInfo, CompressedImage, PointCloud2, PointField
from std_msgs.msg import Float64MultiArray, MultiArrayDimension

# TCP streaming port for the standalone visualizer
TCP_STREAM_PORT = 9999


class OrbSlamNode:
    """
    Subscribes to a compressed camera image stream, runs a two-view ORB-SLAM
    pipeline (feature extraction → matching → pose estimation → triangulation)
    on every consecutive pair of frames, and publishes the growing 3-D point
    cloud on /orb_slam/point_cloud.

    ROS parameters
    ~~~~~~~~~~~~~~
    ~image_topic        : input CompressedImage topic  (default: /camera_node/image/compressed)
    ~camera_info_topic  : CameraInfo topic             (default: /camera_node/camera_info)
    ~n_features         : ORB feature count per frame  (default: 5000)
    ~max_map_points     : cap on accumulated 3-D points (default: 10000)
    ~camera_matrix      : flat 9-element K matrix list  (default: notebook values)
    """

    def __init__(self):
        rospy.init_node("orb_slam_node", anonymous=False)

        n_features = rospy.get_param("~n_features", 5000)
        self._orb = cv2.ORB_create(nfeatures=n_features)
        self._bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)

        # Camera intrinsics — updated from camera_info when available
        self._K_lock = threading.Lock()
        self._K = None
        k_flat = rospy.get_param(
            "~camera_matrix",
            [3177.0, 0.0, 1632.0, 0.0, 3177.0, 1224.0, 0.0, 0.0, 1.0],
        )
        self._default_K = np.array(k_flat, dtype=np.float64).reshape(3, 3)

        # Previous-frame ORB state
        self._prev_kp = None
        self._prev_desc = None

        # Accumulated global pose (camera-to-world transform)
        # x_world = R_cw @ x_cam + t_cw
        self._R_cw = np.eye(3)
        self._t_cw = np.zeros((3, 1))

        # Map accumulation
        self._map_lock = threading.Lock()
        self._map_pts: list = []
        self._max_pts: int = rospy.get_param("~max_map_points", 10000)

        # Accumulated camera poses (4x4 matrices)
        self._poses_lock = threading.Lock()
        self._poses: list = []

        # TCP streaming server for standalone visualizer
        self._tcp_clients: list = []
        self._tcp_lock = threading.Lock()
        self._start_tcp_server()

        # Publishers
        self._pc_pub = rospy.Publisher("/orb_slam/point_cloud", PointCloud2, queue_size=1)
        self._img_pub = rospy.Publisher("/orb_slam/annotated_image", CompressedImage, queue_size=1)
        self._poses_pub = rospy.Publisher("/orb_slam/camera_poses", Float64MultiArray, queue_size=1)

        # Subscribers
        rospy.Subscriber(
            rospy.get_param("~camera_info_topic", "/camera_node/camera_info"),
            CameraInfo,
            self._camera_info_cb,
            queue_size=1,
        )
        rospy.Subscriber(
            rospy.get_param("~image_topic", "/camera_node/image/compressed"),
            CompressedImage,
            self._image_cb,
            queue_size=1,
            buff_size=2**24,
        )
        rospy.loginfo("[OrbSlamNode] Ready.")

    # ------------------------------------------------------------------
    # TCP streaming server
    # ------------------------------------------------------------------

    def _start_tcp_server(self):
        """Start a TCP server that streams SLAM data to connected clients."""
        server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server.bind(('0.0.0.0', TCP_STREAM_PORT))
        server.listen(5)
        server.settimeout(1.0)
        t = threading.Thread(target=self._tcp_accept_loop, args=(server,), daemon=True)
        t.start()
        rospy.loginfo("[OrbSlamNode] TCP stream server started on port %d", TCP_STREAM_PORT)

    def _tcp_accept_loop(self, server):
        """Accept incoming TCP connections."""
        while not rospy.is_shutdown():
            try:
                client, addr = server.accept()
                client.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                with self._tcp_lock:
                    self._tcp_clients.append(client)
                rospy.loginfo("[OrbSlamNode] Visualizer connected from %s", addr)
            except socket.timeout:
                continue
            except Exception:
                break

    def _tcp_broadcast(self, data_dict):
        """Send a pickled data frame to all connected TCP clients."""
        payload = pickle.dumps(data_dict, protocol=pickle.HIGHEST_PROTOCOL)
        header = struct.pack('!I', len(payload))
        with self._tcp_lock:
            dead = []
            for i, client in enumerate(self._tcp_clients):
                try:
                    client.sendall(header + payload)
                except (BrokenPipeError, ConnectionResetError, OSError):
                    dead.append(i)
            for i in reversed(dead):
                self._tcp_clients[i].close()
                del self._tcp_clients[i]

    # ------------------------------------------------------------------

    def _camera_info_cb(self, msg: CameraInfo):
        with self._K_lock:
            if self._K is None:
                self._K = np.array(msg.K, dtype=np.float64).reshape(3, 3)
                rospy.loginfo("[OrbSlamNode] Camera intrinsics loaded from camera_info.")

    def _image_cb(self, msg: CompressedImage):
        buf = np.frombuffer(msg.data, dtype=np.uint8)
        frame_color = cv2.imdecode(buf, cv2.IMREAD_COLOR)
        if frame_color is None:
            return
        frame = cv2.cvtColor(frame_color, cv2.COLOR_BGR2GRAY)

        kp, desc = self._orb.detectAndCompute(frame, None)
        if desc is None or len(kp) < 8:
            self._prev_kp, self._prev_desc = kp, desc
            return

        if self._prev_kp is None or self._prev_desc is None:
            self._prev_kp, self._prev_desc = kp, desc
            return

        with self._K_lock:
            K = self._K if self._K is not None else self._default_K

        # Feature matching
        matches = sorted(
            self._bf.match(self._prev_desc, desc), key=lambda m: m.distance
        )
        if len(matches) < 8:
            self._prev_kp, self._prev_desc = kp, desc
            return

        pts1 = np.float32([self._prev_kp[m.queryIdx].pt for m in matches])
        pts2 = np.float32([kp[m.trainIdx].pt for m in matches])

        # Fundamental matrix with RANSAC to filter outliers
        try:
            F, mask = cv2.findFundamentalMat(pts1, pts2, cv2.FM_RANSAC)
        except cv2.error:
            self._prev_kp, self._prev_desc = kp, desc
            return
        if F is None or mask is None:
            self._prev_kp, self._prev_desc = kp, desc
            return

        pts1_in = pts1[mask.ravel() == 1]
        pts2_in = pts2[mask.ravel() == 1]
        if len(pts1_in) < 5:
            self._prev_kp, self._prev_desc = kp, desc
            return

        # Essential matrix → relative pose
        E = K.T @ F @ K
        _, R, t, _ = cv2.recoverPose(E, pts1_in, pts2_in, K)

        # ------- Motion threshold -------
        # Skip pose update if estimated motion is below noise level
        # (prevents drift when the Duckiebot is stationary)
        t_norm = np.linalg.norm(t)
        angle = np.arccos(np.clip((np.trace(R) - 1.0) / 2.0, -1.0, 1.0))
        MIN_TRANSLATION = 0.01   # minimum translation to count as real motion
        MIN_ROTATION = np.radians(0.5)  # minimum rotation (0.5 degrees)

        if t_norm < MIN_TRANSLATION and angle < MIN_ROTATION:
            # No significant motion — still publish annotated image but skip map update
            self._publish_annotated_image(frame_color, pts1_in, pts2_in)
            self._prev_kp, self._prev_desc = kp, desc
            return

        # Triangulate in the previous camera frame
        p1_n = cv2.undistortPoints(pts1_in.reshape(-1, 1, 2), K, None).reshape(-1, 2)
        p2_n = cv2.undistortPoints(pts2_in.reshape(-1, 1, 2), K, None).reshape(-1, 2)
        P1 = np.hstack((np.eye(3), np.zeros((3, 1))))
        P2 = np.hstack((R, t))
        pts_4d = cv2.triangulatePoints(P1, P2, p1_n.T, p2_n.T).T

        # Dehomogenize and discard near-zero-weight points
        w = pts_4d[:, 3:4]
        valid_w = np.abs(w.ravel()) > 1e-8
        pts_4d = pts_4d[valid_w] / w[valid_w]
        pts_local = pts_4d[:, :3]

        # Keep only points in front of both cameras (positive depth)
        pts_cam2 = (R @ pts_local.T + t).T
        good = (pts_local[:, 2] > 0) & (pts_cam2[:, 2] > 0)
        pts_local = pts_local[good]

        # ------- Outlier filtering -------
        # Reject points that are unrealistically far from the camera
        MAX_DEPTH = 50.0  # max distance in camera units
        if len(pts_local) > 0:
            dists = np.linalg.norm(pts_local, axis=1)
            pts_local = pts_local[dists < MAX_DEPTH]

        if len(pts_local) > 0:
            # Transform from previous camera frame → world frame
            pts_world = (self._R_cw @ pts_local.T).T + self._t_cw.T
            with self._map_lock:
                self._map_pts.extend(pts_world.tolist())
                if len(self._map_pts) > self._max_pts:
                    self._map_pts = self._map_pts[-self._max_pts:]
            self._publish_cloud()

        # Update global pose for the current frame
        # R_cw_new = R_cw_old @ R^T
        # t_cw_new = t_cw_old - R_cw_new @ t
        R_cw_new = self._R_cw @ R.T
        self._t_cw = self._t_cw - R_cw_new @ t
        self._R_cw = R_cw_new

        # Store and publish camera pose
        pose = np.eye(4)
        pose[:3, :3] = self._R_cw
        pose[:3, 3] = self._t_cw.ravel()
        with self._poses_lock:
            self._poses.append(pose)
        self._publish_poses()

        # Draw feature matches on the color image and publish
        self._publish_annotated_image(frame_color, pts1_in, pts2_in)

        self._prev_kp, self._prev_desc = kp, desc

    def _publish_annotated_image(self, img, pts1, pts2):
        """Draw matched feature points and lines on the image, then publish."""
        vis = img.copy()
        for (x1, y1), (x2, y2) in zip(pts1.astype(int), pts2.astype(int)):
            cv2.circle(vis, (x2, y2), 2, (77, 243, 255), -1)
            cv2.line(vis, (x1, y1), (x2, y2), (255, 0, 0), 1)
            cv2.circle(vis, (x1, y1), 2, (204, 77, 255), -1)

        msg = CompressedImage()
        msg.header.stamp = rospy.Time.now()
        msg.format = "jpeg"
        msg.data = np.array(cv2.imencode('.jpg', vis)[1]).tobytes()
        self._img_pub.publish(msg)

        # Stream to standalone TCP visualizer
        with self._tcp_lock:
            has_clients = len(self._tcp_clients) > 0
        if has_clients:
            with self._map_lock:
                pts = np.array(self._map_pts, dtype=np.float64) if self._map_pts else np.empty((0, 3))
            with self._poses_lock:
                poses = np.array(self._poses, dtype=np.float64) if self._poses else np.empty((0, 4, 4))
            self._tcp_broadcast({
                'points': pts,
                'poses': poses,
                'image': vis,
            })

    def _publish_poses(self):
        """Publish all camera poses as a flat array of 4x4 matrices."""
        with self._poses_lock:
            if not self._poses:
                return
            data = np.array(self._poses, dtype=np.float64).flatten()

        msg = Float64MultiArray()
        msg.layout.dim = [
            MultiArrayDimension(label="poses", size=len(self._poses), stride=16 * len(self._poses)),
            MultiArrayDimension(label="matrix", size=16, stride=16),
        ]
        msg.data = data.tolist()
        self._poses_pub.publish(msg)

    def _publish_cloud(self):
        with self._map_lock:
            pts = np.array(self._map_pts, dtype=np.float32)

        header = std_msgs.msg.Header()
        header.stamp = rospy.Time.now()
        header.frame_id = "map"

        fields = [
            PointField("x", 0, PointField.FLOAT32, 1),
            PointField("y", 4, PointField.FLOAT32, 1),
            PointField("z", 8, PointField.FLOAT32, 1),
        ]
        self._pc_pub.publish(pc2.create_cloud(header, fields, pts))


if __name__ == "__main__":
    OrbSlamNode()
    rospy.spin()
