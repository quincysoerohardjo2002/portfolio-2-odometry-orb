#!/usr/bin/env python3
"""
Standalone ORB-SLAM Visualizer (no ROS required).

Connects to the Duckiebot's TCP stream and renders:
  - Red 3D point cloud (map points)
  - Green camera poses (trajectory wireframes)
  - Camera image with features (bottom-left)

Usage:
    python3 visualizer_standalone.py [duckiebot-ip]

Default IP: 192.168.0.166 (duckiebot21)
"""

import pickle
import socket
import struct
import sys
from multiprocessing import Process, Queue

import cv2
import numpy as np
import OpenGL.GL as gl
import pangolin

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


def network_receiver(host, port, q_state, q_image):
    """Connect to Duckiebot and forward received data to queues."""
    print(f"[Receiver] Connecting to {host}:{port}...")
    while True:
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.connect((host, port))
            print(f"[Receiver] Connected to {host}:{port}")

            while True:
                # Read 4-byte length header
                header = recv_exact(sock, 4)
                length = struct.unpack('!I', header)[0]

                # Read payload
                payload = recv_exact(sock, length)
                data = pickle.loads(payload)

                # Forward to queues
                if 'points' in data or 'poses' in data:
                    q_state.put({
                        'points': data.get('points'),
                        'poses': data.get('poses'),
                    })
                if 'image' in data and data['image'] is not None:
                    q_image.put(data['image'])

        except (ConnectionError, ConnectionRefusedError, OSError) as e:
            print(f"[Receiver] Connection lost: {e}. Retrying in 2s...")
            import time
            time.sleep(2)
        except Exception as e:
            print(f"[Receiver] Error: {e}")
            import time
            time.sleep(2)


# ---------------------------------------------------------------------------
# Pangolin viewer
# ---------------------------------------------------------------------------

def viewer_main(q_state, q_image):
    """Pangolin visualization loop."""
    w, h = 1280, 720
    img_width, img_height = 480, 270

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
    dimg = pangolin.Display('image')
    dimg.SetBounds(0, img_height / float(h), 0.0, img_width / float(w), float(w) / float(h))
    dimg.SetLock(pangolin.Lock.LockLeft, pangolin.Lock.LockTop)

    texture = pangolin.GlTexture(img_width, img_height, gl.GL_RGB, False, 0, gl.GL_RGB, gl.GL_UNSIGNED_BYTE)
    image = np.ones((img_height, img_width, 3), 'uint8')

    state = {'poses': None, 'points': None}

    while not pangolin.ShouldQuit():
        # Drain queues — use latest data
        while not q_state.empty():
            update = q_state.get()
            if update.get('poses') is not None:
                state['poses'] = update['poses']
            if update.get('points') is not None:
                state['points'] = update['points']

        while not q_image.empty():
            image = q_image.get()

        # Clear
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

        # Draw camera image (bottom-left)
        if image is not None:
            disp_img = image
            if disp_img.ndim == 3:
                disp_img = disp_img[::-1, :, ::-1]
            else:
                disp_img = np.repeat(disp_img[::-1, :, np.newaxis], 3, axis=2)
            disp_img = cv2.resize(disp_img, (img_width, img_height))
            texture.Upload(disp_img, gl.GL_RGB, gl.GL_UNSIGNED_BYTE)
            dimg.Activate()
            gl.glColor3f(1.0, 1.0, 1.0)
            texture.RenderToViewport()

        pangolin.FinishFrame()


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    host = sys.argv[1] if len(sys.argv) > 1 else "192.168.0.166"
    port = 9999

    print("=" * 50)
    print(" ORB-SLAM Standalone Visualizer (Pangolin)")
    print("=" * 50)
    print(f" Duckiebot: {host}:{port}")
    print("=" * 50)

    q_state = Queue()
    q_image = Queue()

    # Start network receiver in a separate process
    receiver = Process(target=network_receiver, args=(host, port, q_state, q_image), daemon=True)
    receiver.start()

    # Run Pangolin viewer in the main process (requires main thread)
    viewer_main(q_state, q_image)
