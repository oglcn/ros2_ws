"""
ROS2 node serving the delivery robot web dashboard via aiohttp.

Provides:
  GET /          -- single-page web app
  GET /stream    -- MJPEG proxy from /camera/image_raw/compressed
  GET /snapshot  -- latest single JPEG frame
  GET /api/markers -- ArUco marker map positions (JSON)
  WS  /ws        -- bidirectional bridge (cmd_vel in, status + localization out)
"""

import asyncio
import json
import math
import os
import struct
import threading
import time

import cv2
import numpy as np
import yaml

import rclpy
from aiohttp import web
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import CompressedImage, Image, PointCloud2
from std_msgs.msg import Header

from delivery_robot_msgs.msg import ArucoDetections, MotorStatus

FAST_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1,
)

STATIC_DIR = os.path.join(os.path.dirname(__file__), 'static')

CALIBRATION_OUTPUT = '/home/pi/ros2_ws/src/delivery_robot/delivery_robot_bringup/config/camera_calibration.yaml'


class WebUINode(Node):
    def __init__(self):
        super().__init__('web_ui')

        self.declare_parameter('port', 8080)
        self.declare_parameter('markers_file', '')
        self.port = self.get_parameter('port').value
        markers_file = self.get_parameter('markers_file').value

        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # Camera frame
        self._latest_frame = b''
        self._frame_lock = threading.Lock()

        # Motor status
        self._motor_status = {'duty_cycles': [0, 0, 0, 0], 'active': [False] * 4, 'mode': 'manual'}
        self._status_lock = threading.Lock()

        # Localization state
        self._loc_lock = threading.Lock()
        self._vslam_last_time = 0.0
        self._vslam_ever_received = False
        self._ekf_last_time = 0.0
        self._ekf_ever_received = False
        self._position = {'x': 0.0, 'y': 0.0, 'yaw': 0.0}
        self._aruco_detections = []

        # Point cloud
        self._pc_lock = threading.Lock()
        self._point_cloud = None
        self._pc_updated = False

        # Calibration state
        self._cal_lock = threading.Lock()
        self._calibrating = False
        self._cal_frames = []
        self._cal_last_capture = 0.0
        self._cal_objp_template = None
        self._cal_image_size = None
        self._cal_rows = 7
        self._cal_cols = 9
        self._cal_square_size = 0.025
        self._cal_target_frames = 20
        self._cal_result = None

        # Marker map
        self._marker_map = self._load_markers(markers_file)

        self._ws_clients: list[web.WebSocketResponse] = []
        self._bridge = CvBridge()

        # Subscribers
        self.create_subscription(
            CompressedImage, 'camera/image_raw/compressed',
            self._image_cb, FAST_QOS
        )
        self.create_subscription(
            MotorStatus, 'motor_status',
            self._motor_status_cb, 10
        )
        self.create_subscription(
            Odometry, 'visual_odom',
            self._vslam_cb, FAST_QOS
        )
        self.create_subscription(
            Odometry, 'odometry/filtered',
            self._ekf_cb, 10
        )
        self.create_subscription(
            ArucoDetections, 'aruco/detections',
            self._aruco_detections_cb, 10
        )
        self.create_subscription(
            PointCloud2, 'vslam/map_points',
            self._pointcloud_cb, 10
        )

        self._raw_image_sub = None

        self._aio_thread = threading.Thread(target=self._run_server, daemon=True)
        self._aio_thread.start()

        self.get_logger().info(f'Web UI at http://0.0.0.0:{self.port}')

    def _load_markers(self, path):
        """Load marker positions from YAML file."""
        if not path:
            default = '/home/pi/ros2_ws/src/delivery_robot/delivery_robot_bringup/config/aruco_markers.yaml'
            if os.path.exists(default):
                path = default
            else:
                return {}
        try:
            with open(path, 'r') as f:
                data = yaml.safe_load(f)
            markers = {}
            for mid, pos in data.get('markers', {}).items():
                markers[int(mid)] = {
                    'x': float(pos.get('x', 0.0)),
                    'y': float(pos.get('y', 0.0)),
                    'z': float(pos.get('z', 0.0)),
                }
            return markers
        except Exception:
            return {}

    # --- ROS2 Callbacks ---

    def _image_cb(self, msg: CompressedImage):
        with self._frame_lock:
            self._latest_frame = bytes(msg.data)

    def _motor_status_cb(self, msg: MotorStatus):
        with self._status_lock:
            self._motor_status = {
                'duty_cycles': [float(d) for d in msg.duty_cycles],
                'active': [bool(a) for a in msg.active],
                'mode': str(msg.mode),
            }

    def _vslam_cb(self, msg: Odometry):
        with self._loc_lock:
            self._vslam_last_time = time.time()
            self._vslam_ever_received = True

    def _ekf_cb(self, msg: Odometry):
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp) * 180.0 / math.pi

        with self._loc_lock:
            self._ekf_last_time = time.time()
            self._ekf_ever_received = True
            self._position = {
                'x': float(msg.pose.pose.position.x),
                'y': float(msg.pose.pose.position.y),
                'yaw': round(yaw, 1),
            }

    def _aruco_detections_cb(self, msg: ArucoDetections):
        detections = []
        for i, mid in enumerate(msg.marker_ids):
            base = i * 8
            if base + 7 < len(msg.corners):
                corners = [
                    [float(msg.corners[base]), float(msg.corners[base + 1])],
                    [float(msg.corners[base + 2]), float(msg.corners[base + 3])],
                    [float(msg.corners[base + 4]), float(msg.corners[base + 5])],
                    [float(msg.corners[base + 6]), float(msg.corners[base + 7])],
                ]
                detections.append({'id': int(mid), 'corners': corners})

        with self._loc_lock:
            self._aruco_detections = detections

    def _pointcloud_cb(self, msg: PointCloud2):
        points = []
        point_step = msg.point_step
        data = bytes(msg.data)
        for i in range(msg.width):
            offset = i * point_step
            x, y, z = struct.unpack_from('fff', data, offset)
            points.append([round(x, 3), round(y, 3), round(z, 3)])

        with self._pc_lock:
            self._point_cloud = {'count': len(points), 'points': points}
            self._pc_updated = True

    def _raw_image_cb(self, msg: Image):
        """Process raw images for calibration."""
        with self._cal_lock:
            if not self._calibrating:
                return
            now = time.time()
            if now - self._cal_last_capture < 0.5:
                return

        frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        with self._cal_lock:
            if self._cal_image_size is None:
                self._cal_image_size = gray.shape[::-1]
            if self._cal_objp_template is None:
                objp = np.zeros((self._cal_rows * self._cal_cols, 3), np.float32)
                objp[:, :2] = np.mgrid[0:self._cal_cols, 0:self._cal_rows].T.reshape(-1, 2) * self._cal_square_size
                self._cal_objp_template = objp

        found, corners = cv2.findChessboardCorners(
            gray, (self._cal_cols, self._cal_rows), None
        )

        corner_list = None
        if found:
            criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
            corners_refined = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
            corner_list = corners_refined.reshape(-1, 2).tolist()

            with self._cal_lock:
                self._cal_last_capture = time.time()
                self._cal_frames.append((self._cal_objp_template, corners_refined))

                if len(self._cal_frames) >= self._cal_target_frames:
                    self._run_calibration()

        with self._cal_lock:
            frame_count = len(self._cal_frames)
            cal_result = self._cal_result

        # Push calibration state to clients
        if cal_result:
            cal_data = cal_result
        else:
            cal_data = {
                'state': 'detecting',
                'frames': frame_count,
                'total': self._cal_target_frames,
                'found': found,
            }
            if corner_list:
                cal_data['corners'] = corner_list

        self._push_to_clients({'calibration': cal_data})

    def _run_calibration(self):
        """Run OpenCV camera calibration (called under _cal_lock)."""
        obj_points = [f[0] for f in self._cal_frames]
        img_points = [f[1] for f in self._cal_frames]

        ret, mtx, dist, _, _ = cv2.calibrateCamera(
            obj_points, img_points, self._cal_image_size, None, None
        )

        self._save_calibration(mtx, dist, ret)
        self._cal_result = {
            'state': 'complete',
            'rms_error': round(ret, 4),
        }
        self._calibrating = False
        self.get_logger().info(f'Calibration complete. RMS error: {ret:.4f}')

    def _save_calibration(self, mtx, dist, rms):
        """Save calibration YAML."""
        w, h = self._cal_image_size
        fx, fy = mtx[0, 0], mtx[1, 1]
        cx, cy = mtx[0, 2], mtx[1, 2]
        d = dist.ravel()

        import datetime
        lines = [
            f'# Camera calibration for Pi Camera Module 3 (IMX708) at {w}x{h}',
            f'# Generated by web UI calibration on {datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")}',
            f'# RMS reprojection error: {rms:.4f}',
            '',
            f'image_width: {w}',
            f'image_height: {h}',
            '',
            'camera_matrix:',
            '  rows: 3',
            '  cols: 3',
            f'  data: [{fx}, 0.0, {cx},',
            f'         0.0, {fy}, {cy},',
            '         0.0, 0.0, 1.0]',
            '',
            'distortion_model: plumb_bob',
            'distortion_coefficients:',
            '  rows: 1',
            f'  cols: {len(d)}',
            f'  data: [{", ".join(f"{v:.6f}" for v in d)}]',
            '',
            'rectification_matrix:',
            '  rows: 3',
            '  cols: 3',
            '  data: [1.0, 0.0, 0.0,',
            '         0.0, 1.0, 0.0,',
            '         0.0, 0.0, 1.0]',
            '',
            'projection_matrix:',
            '  rows: 3',
            '  cols: 4',
            f'  data: [{fx}, 0.0, {cx}, 0.0,',
            f'         0.0, {fy}, {cy}, 0.0,',
            '         0.0, 0.0, 1.0, 0.0]',
        ]

        try:
            with open(CALIBRATION_OUTPUT, 'w') as f:
                f.write('\n'.join(lines) + '\n')
        except Exception as e:
            self.get_logger().error(f'Failed to save calibration: {e}')

    # --- aiohttp Server ---

    def _run_server(self):
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        self._aio_loop = loop
        app = web.Application()
        app.router.add_get('/', self._handle_index)
        app.router.add_get('/stream', self._handle_stream)
        app.router.add_get('/snapshot', self._handle_snapshot)
        app.router.add_get('/api/markers', self._handle_markers)
        app.router.add_get('/ws', self._handle_ws)
        runner = web.AppRunner(app)
        loop.run_until_complete(runner.setup())
        site = web.TCPSite(runner, '0.0.0.0', self.port, reuse_address=True)
        loop.run_until_complete(site.start())
        loop.run_forever()

    async def _handle_index(self, request):
        path = os.path.join(STATIC_DIR, 'index.html')
        return web.FileResponse(path)

    async def _handle_snapshot(self, request):
        with self._frame_lock:
            frame = self._latest_frame
        if not frame:
            return web.Response(status=503, text='No frame available')
        return web.Response(body=frame, content_type='image/jpeg')

    async def _handle_stream(self, request):
        boundary = 'frameboundary'
        response = web.StreamResponse(
            status=200,
            headers={
                'Content-Type': f'multipart/x-mixed-replace; boundary={boundary}',
                'Cache-Control': 'no-cache',
            },
        )
        await response.prepare(request)

        prev_frame = None
        try:
            while True:
                with self._frame_lock:
                    frame = self._latest_frame
                if frame and frame is not prev_frame:
                    prev_frame = frame
                    await response.write(
                        f'--{boundary}\r\n'
                        f'Content-Type: image/jpeg\r\n'
                        f'Content-Length: {len(frame)}\r\n'
                        f'\r\n'.encode() + frame + b'\r\n'
                    )
                else:
                    await asyncio.sleep(0.03)
        except (ConnectionResetError, asyncio.CancelledError):
            pass
        return response

    async def _handle_markers(self, request):
        return web.json_response(self._marker_map)

    async def _handle_ws(self, request):
        ws = web.WebSocketResponse()
        await ws.prepare(request)
        self._ws_clients.append(ws)
        self.get_logger().info('WebSocket client connected')

        status_task = asyncio.create_task(self._push_status(ws))

        try:
            async for msg_raw in ws:
                if msg_raw.type == web.WSMsgType.TEXT:
                    try:
                        data = json.loads(msg_raw.data)
                    except json.JSONDecodeError:
                        continue
                    if 'cmd_vel' in data:
                        cv = data['cmd_vel']
                        twist = Twist()
                        twist.linear.x = float(cv.get('lx', 0))
                        twist.linear.y = float(cv.get('ly', 0))
                        twist.angular.z = float(cv.get('az', 0))
                        self.cmd_vel_pub.publish(twist)
                    elif 'calibration' in data:
                        self._handle_calibration_cmd(data['calibration'])
                elif msg_raw.type == web.WSMsgType.ERROR:
                    break
        finally:
            status_task.cancel()
            if ws in self._ws_clients:
                self._ws_clients.remove(ws)
            self.get_logger().info('WebSocket client disconnected')

        return ws

    def _handle_calibration_cmd(self, cmd):
        """Handle calibration start/stop from WebSocket."""
        if cmd == 'start':
            with self._cal_lock:
                self._calibrating = True
                self._cal_frames = []
                self._cal_last_capture = 0.0
                self._cal_image_size = None
                self._cal_objp_template = None
                self._cal_result = None

            if self._raw_image_sub is None:
                self._raw_image_sub = self.create_subscription(
                    Image, 'camera/image_raw', self._raw_image_cb, FAST_QOS
                )
            self.get_logger().info('Calibration started from web UI')

        elif cmd == 'stop':
            with self._cal_lock:
                self._calibrating = False
                self._cal_result = None
            self.get_logger().info('Calibration cancelled from web UI')

    async def _push_status(self, ws: web.WebSocketResponse):
        """Push motor status + localization data at 5Hz, point cloud when available."""
        try:
            while not ws.closed:
                # Motor status
                with self._status_lock:
                    status = dict(self._motor_status)
                status['connected'] = True

                # Localization
                now = time.time()
                with self._loc_lock:
                    if not self._vslam_ever_received:
                        vslam_state = 'initializing'
                    elif now - self._vslam_last_time < 2.0:
                        vslam_state = 'tracking'
                    else:
                        vslam_state = 'lost'

                    ekf_active = self._ekf_ever_received and (now - self._ekf_last_time < 2.0)
                    position = dict(self._position)
                    aruco_dets = list(self._aruco_detections)

                localization = {
                    'vslam_state': vslam_state,
                    'ekf_active': ekf_active,
                    'position': position,
                    'aruco_detections': aruco_dets,
                }

                msg = {'status': status, 'localization': localization}

                # Include point cloud if updated
                with self._pc_lock:
                    if self._pc_updated:
                        msg['point_cloud'] = self._point_cloud
                        self._pc_updated = False

                await ws.send_json(msg)
                await asyncio.sleep(0.2)
        except (ConnectionResetError, asyncio.CancelledError):
            pass

    def _push_to_clients(self, data):
        """Push a message to all connected WebSocket clients (thread-safe)."""
        if not self._ws_clients:
            return
        if hasattr(self, '_aio_loop'):
            asyncio.run_coroutine_threadsafe(
                self._async_push(data), self._aio_loop
            )

    async def _async_push(self, data):
        for ws in list(self._ws_clients):
            if not ws.closed:
                try:
                    await ws.send_json(data)
                except Exception:
                    pass

    def destroy_node(self):
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = WebUINode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
