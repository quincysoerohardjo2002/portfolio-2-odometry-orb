#!/usr/bin/env python3
"""
PointCloud visualizer ROS node using Pangolin.

Subscribes to:
  /orb_slam/point_cloud      (sensor_msgs/PointCloud2)     — 3D map points
  /orb_slam/annotated_image  (sensor_msgs/CompressedImage) — camera feed with features
  /orb_slam/camera_poses     (std_msgs/Float64MultiArray)  — camera trajectory

Renders the scene in a Pangolin window with:
  - Red point cloud (map points)
  - Green camera poses (trajectory wireframes)
  - Camera image preview (bottom-left)

The Pangolin event loop runs in a separate process (multiprocessing),
communicating with the ROS callbacks via Queues.

ROS parameters
~~~~~~~~~~~~~~
~point_cloud_topic : input topic  (default: /orb_slam/point_cloud)
~image_topic       : annotated image topic (default: /orb_slam/annotated_image)
~poses_topic       : camera poses topic (default: /orb_slam/camera_poses)
"""

from multiprocessing import Process, Queue

import cv2
import numpy as np
import rospy
from sensor_msgs import point_cloud2 as pc2
from sensor_msgs.msg import CompressedImage, PointCloud2
from std_msgs.msg import Float64MultiArray


# ---------------------------------------------------------------------------
# Pangolin viewer process
# ---------------------------------------------------------------------------

def viewer_init(w, h):
    """Initialize the Pangolin window and OpenGL state."""
    import pangolin
    import OpenGL.GL as gl

    pangolin.CreateWindowAndBind('ORB-SLAM Duckiebot', w, h)
    gl.glEnable(gl.GL_DEPTH_TEST)

    scam = pangolin.OpenGlRenderState(
        pangolin.ProjectionMatrix(w, h, 420, 420, w // 2, h // 2, 0.2, 10000),
        pangolin.ModelViewLookAt(0, -10, -8, 0, 0, 0, 0, -1, 0))
    handler = pangolin.Handler3D(scam)

    dcam = pangolin.CreateDisplay()
    dcam.SetBounds(0.0, 1.0, 0.0, 1.0, -w / h)
    dcam.SetHandler(handler)

    # Image display (bottom-left corner)
    img_width, img_height = 480, 270
    dimg = pangolin.Display('image')
    dimg.SetBounds(0, img_height / float(h), 0.0, img_width / float(w), float(w) / float(h))
    dimg.SetLock(pangolin.Lock.LockLeft, pangolin.Lock.LockTop)

    texture = pangolin.GlTexture(img_width, img_height, gl.GL_RGB, False, 0, gl.GL_RGB, gl.GL_UNSIGNED_BYTE)
    image = np.ones((img_height, img_width, 3), 'uint8')

    return scam, dcam, dimg, texture, image, img_width, img_height


def viewer_refresh(scam, dcam, dimg, texture, image, img_width, img_height, state):
    """Render one frame of the Pangolin viewer."""
    import pangolin
    import OpenGL.GL as gl

    gl.glClear(gl.GL_COLOR_BUFFER_BIT | gl.GL_DEPTH_BUFFER_BIT)
    gl.glClearColor(1.0, 1.0, 1.0, 1.0)
    dcam.Activate(scam)

    # Draw camera poses (green wireframes)
    if state['poses'] is not None and len(state['poses']) > 0:
        gl.glLineWidth(1)
        gl.glColor3f(0.0, 1.0, 0.0)
        pangolin.DrawCameras(state['poses'])

    # Draw point cloud (red)
    if state['points'] is not None and len(state['points']) > 0:
        gl.glPointSize(2)
        gl.glColor3f(1.0, 0.0, 0.0)
        pangolin.DrawPoints(state['points'])

    # Draw camera image
    if state['image'] is not None:
        image = state['image']
        if image.ndim == 3:
            image = image[::-1, :, ::-1]
        else:
            image = np.repeat(image[::-1, :, np.newaxis], 3, axis=2)
        image = cv2.resize(image, (img_width, img_height))
        texture.Upload(image, gl.GL_RGB, gl.GL_UNSIGNED_BYTE)
        dimg.Activate()
        gl.glColor3f(1.0, 1.0, 1.0)
        texture.RenderToViewport()

    pangolin.FinishFrame()


def viewer_thread(q_state, q_image):
    """Main loop for the Pangolin viewer process."""
    import pangolin

    scam, dcam, dimg, texture, image, img_w, img_h = viewer_init(1280, 720)

    state = {'poses': None, 'points': None, 'image': None}

    while not pangolin.ShouldQuit():
        # Get latest state (point cloud + poses)
        while not q_state.empty():
            state_update = q_state.get()
            state['poses'] = state_update.get('poses', state['poses'])
            state['points'] = state_update.get('points', state['points'])

        # Get latest image
        while not q_image.empty():
            state['image'] = q_image.get()

        viewer_refresh(scam, dcam, dimg, texture, image, img_w, img_h, state)


# ---------------------------------------------------------------------------
# ROS node
# ---------------------------------------------------------------------------

class PointCloudVisualizerNode:
    def __init__(self):
        rospy.init_node("pointcloud_visualizer_node", anonymous=False)

        # Queues for inter-process communication with Pangolin viewer
        self._q_state = Queue()
        self._q_image = Queue()

        # Start Pangolin viewer in a separate process
        self._viewer_proc = Process(target=viewer_thread, args=(self._q_state, self._q_image))
        self._viewer_proc.daemon = True
        self._viewer_proc.start()
        rospy.loginfo("[PointCloudVisualizerNode] Pangolin viewer started.")

        # State
        self._points = np.empty((0, 3), dtype=np.float64)
        self._poses = np.empty((0, 4, 4), dtype=np.float64)

        # Subscribers
        rospy.Subscriber(
            rospy.get_param("~point_cloud_topic", "/orb_slam/point_cloud"),
            PointCloud2,
            self._pc_callback,
            queue_size=1,
        )
        rospy.Subscriber(
            rospy.get_param("~image_topic", "/orb_slam/annotated_image"),
            CompressedImage,
            self._image_callback,
            queue_size=1,
        )
        rospy.Subscriber(
            rospy.get_param("~poses_topic", "/orb_slam/camera_poses"),
            Float64MultiArray,
            self._poses_callback,
            queue_size=1,
        )
        rospy.loginfo("[PointCloudVisualizerNode] Subscribed to topics. Ready.")

    def _pc_callback(self, msg: PointCloud2):
        raw = list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True))
        if raw:
            self._points = np.array(raw, dtype=np.float64)
            self._send_state()

    def _poses_callback(self, msg: Float64MultiArray):
        data = np.array(msg.data, dtype=np.float64)
        if len(data) >= 16:
            n_poses = len(data) // 16
            self._poses = data.reshape(n_poses, 4, 4)
            self._send_state()

    def _image_callback(self, msg: CompressedImage):
        buf = np.frombuffer(msg.data, dtype=np.uint8)
        img = cv2.imdecode(buf, cv2.IMREAD_COLOR)
        if img is not None:
            self._q_image.put(img)

    def _send_state(self):
        """Send latest point cloud and poses to the viewer process."""
        self._q_state.put({
            'poses': self._poses.copy() if len(self._poses) > 0 else None,
            'points': self._points.copy() if len(self._points) > 0 else None,
        })


if __name__ == "__main__":
    node = PointCloudVisualizerNode()
    rospy.spin()
