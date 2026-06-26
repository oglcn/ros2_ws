"""
ROS2 node serving the delivery robot web dashboard via aiohttp.

Provides:
  GET /          -- single-page web app (robot dashboard)
  GET /order     -- customer ordering page (speech-to-text)
  GET /admin     -- admin order management page
  GET /stream    -- MJPEG proxy from /camera/image_raw/compressed
  GET /snapshot  -- latest single JPEG frame
  GET /api/markers   -- ArUco marker map positions (JSON)
  GET /api/menu      -- menu items (JSON from menu.yaml)
  GET /api/orders    -- all orders (JSON)
  POST /api/orders   -- submit a new order
  GET /api/locations -- named locations from active map (JSON)
  WS  /ws        -- bidirectional bridge (cmd_vel in, status + localization out)
"""

import asyncio
import datetime
import json
import math
import os
import signal
import struct
import subprocess
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
from sensor_msgs.msg import CompressedImage, Image, Imu, PointCloud2
from std_msgs.msg import Bool, Empty, Float64, Header, Int32

from delivery_robot_msgs.msg import (
    ApproachStatus, ArucoDetections, DeliveryGoal, DeliveryStatus, MotorStatus,
)
from geometry_msgs.msg import Point

FAST_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1,
)

STATIC_DIR = os.path.join(os.path.dirname(__file__), 'static')

CALIBRATION_OUTPUT = '/home/pi/ros2_ws/src/delivery_robot/delivery_robot_bringup/config/camera_calibration.yaml'

MAPS_DIR = os.path.expanduser('~/ros2_ws/aruco_maps')
ACTIVE_MAP_FILE = os.path.join(MAPS_DIR, 'active_map.txt')
LEGACY_MARKERS = '/home/pi/ros2_ws/src/delivery_robot/delivery_robot_bringup/config/aruco_markers.yaml'


class WebUINode(Node):
    def __init__(self):
        super().__init__('web_ui')

        self.declare_parameter('port', 8080)
        self.declare_parameter('markers_file', '')
        self.declare_parameter('menu_file', '')
        self.declare_parameter('auto_return_home', True)
        self.declare_parameter('return_delay', 5.0)
        self.port = self.get_parameter('port').value
        markers_file = self.get_parameter('markers_file').value
        self._menu_file = self.get_parameter('menu_file').value
        self._auto_return_home = self.get_parameter('auto_return_home').value
        self._return_delay = self.get_parameter('return_delay').value

        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.approach_armed_pub = self.create_publisher(
            Bool, 'aruco_approach/armed', 10)
        self.approach_target_pub = self.create_publisher(
            Int32, 'aruco_approach/set_target', 10)

        self.delivery_goal_pub = self.create_publisher(
            DeliveryGoal, 'delivery/goal', 10)
        self.delivery_cancel_pub = self.create_publisher(
            Empty, 'delivery/cancel', 10)
        self.delivery_pause_pub = self.create_publisher(
            Bool, 'delivery/pause', 10)

        # Camera frame
        self._latest_frame = b''
        self._frame_lock = threading.Lock()

        # Motor status
        self._motor_status = {'duty_cycles': [0, 0, 0, 0], 'active': [False] * 4, 'mode': 'manual'}
        self._status_lock = threading.Lock()

        self._frame_count = 0
        self._fps_window_start = time.monotonic()
        self._camera_fps = 0.0
        self._fps_lock = threading.Lock()

        # Localization state
        self._loc_lock = threading.Lock()
        self._vslam_last_time = 0.0
        self._vslam_ever_received = False
        self._ekf_last_time = 0.0
        self._ekf_ever_received = False
        self._position = {'x': 0.0, 'y': 0.0, 'yaw': 0.0}
        self._aruco_detections = []

        # IMU state
        self._imu_lock = threading.Lock()
        self._imu_last_time = 0.0
        self._imu_ever_received = False
        self._imu_heading_deg = 0.0
        self._imu_gyro_z = 0.0

        # Point cloud
        self._pc_lock = threading.Lock()
        self._point_cloud = None
        self._pc_updated = False

        # Approach status
        self._approach_lock = threading.Lock()
        self._approach_status = {
            'target_id': 0, 'armed': False, 'visible': False,
            'size_norm': 0.0, 'state': 'IDLE',
        }

        # Delivery status
        self._delivery_lock = threading.Lock()
        self._delivery_status = {
            'state': 'IDLE', 'target_marker_id': 0,
            'current_waypoint_index': 0, 'total_waypoints': 0,
            'distance_to_waypoint': 0.0, 'heading_error': 0.0,
            'message': '',
        }

        # Calibration state
        self._cal_lock = threading.Lock()
        self._calibrating = False
        self._cal_frames = []
        self._cal_last_capture = 0.0
        self._cal_last_attempt = 0.0
        self._cal_objp_template = None
        self._cal_image_size = None
        self._cal_rows = 6
        self._cal_cols = 9
        self._cal_square_size = 0.025
        self._cal_target_frames = 20
        self._cal_result = None

        # Localization subprocess
        self._loc_proc_lock = threading.Lock()
        self._loc_proc = None  # subprocess.Popen or None
        self._loc_error = ''

        # Marker maps directory
        self._bootstrap_maps_dir()
        self._active_map_file = self._get_active_map_path()
        self._marker_map = self._load_markers(self._active_map_file)

        # Menu & orders
        self._menu_data = self._load_menu()
        ordering = self._menu_data.get('ordering', {})
        if 'auto_return_home' in ordering:
            self._auto_return_home = bool(ordering['auto_return_home'])
        if 'return_delay' in ordering:
            self._return_delay = float(ordering['return_delay'])
        self._orders_lock = threading.Lock()
        self._orders: list[dict] = []
        self._next_order_id = 1
        self._return_timer = None

        self._ws_clients: list[web.WebSocketResponse] = []
        self._bridge = CvBridge()


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
            ApproachStatus, 'aruco_approach/status',
            self._approach_status_cb, 10
        )
        self.create_subscription(
            PointCloud2, 'vslam/map_points',
            self._pointcloud_cb, 10
        )
        self.create_subscription(
            Imu, 'imu/data_raw',
            self._imu_cb, FAST_QOS
        )
        self.create_subscription(
            Float64, 'imu/heading',
            self._heading_cb, 10
        )
        self.create_subscription(
            DeliveryStatus, 'delivery/status',
            self._delivery_status_cb, 10
        )

        self._raw_image_sub = None

        self._aio_thread = threading.Thread(target=self._run_server, daemon=True)
        self._aio_thread.start()

        # Auto-start localization stack after a short delay (let camera start first)
        self._loc_start_timer = self.create_timer(3.0, self._auto_start_localization)

        self.get_logger().info(f'Web UI at http://0.0.0.0:{self.port}')

    def _auto_start_localization(self):
        """Start localization stack automatically on boot."""
        self._loc_start_timer.cancel()
        self._start_localization()
        self._broadcast_loc_stack_status()
        self.get_logger().info('Localization stack auto-started')

    def _bootstrap_maps_dir(self):
        """Create maps directory and migrate legacy config on first run."""
        if os.path.isdir(MAPS_DIR):
            return
        os.makedirs(MAPS_DIR, exist_ok=True)
        if os.path.exists(LEGACY_MARKERS):
            try:
                with open(LEGACY_MARKERS, 'r') as f:
                    data = yaml.safe_load(f) or {}
                data['name'] = 'Default'
                if 'anchors' not in data:
                    data['anchors'] = list(data.get('markers', {}).keys())
                dest = os.path.join(MAPS_DIR, 'default.yaml')
                with open(dest, 'w') as f:
                    yaml.safe_dump(data, f, default_flow_style=False, sort_keys=False)
                with open(ACTIVE_MAP_FILE, 'w') as f:
                    f.write('default.yaml\n')
                self.get_logger().info(f'Migrated legacy markers to {dest}')
            except Exception as e:
                self.get_logger().error(f'Failed to migrate markers: {e}')
        else:
            with open(ACTIVE_MAP_FILE, 'w') as f:
                f.write('\n')

    def _get_active_map_path(self):
        """Read active_map.txt and return the full path to the active map."""
        try:
            with open(ACTIVE_MAP_FILE, 'r') as f:
                filename = f.read().strip()
            if filename:
                path = os.path.join(MAPS_DIR, filename)
                if os.path.exists(path):
                    return path
        except Exception:
            pass
        # Fallback: first yaml in the directory
        for fn in sorted(os.listdir(MAPS_DIR)):
            if fn.endswith('.yaml'):
                return os.path.join(MAPS_DIR, fn)
        return ''

    def _set_active_map(self, filename):
        """Write the active map filename and reload markers."""
        path = os.path.join(MAPS_DIR, filename)
        if not os.path.exists(path):
            return False
        with open(ACTIVE_MAP_FILE, 'w') as f:
            f.write(filename + '\n')
        self._active_map_file = path
        self._marker_map = self._load_markers(path)
        return True

    def _list_maps(self):
        """List all map files with metadata."""
        maps = []
        active_filename = ''
        try:
            with open(ACTIVE_MAP_FILE, 'r') as f:
                active_filename = f.read().strip()
        except Exception:
            pass
        for fn in sorted(os.listdir(MAPS_DIR)):
            if not fn.endswith('.yaml'):
                continue
            path = os.path.join(MAPS_DIR, fn)
            try:
                with open(path, 'r') as f:
                    data = yaml.safe_load(f) or {}
                maps.append({
                    'filename': fn,
                    'name': data.get('name', fn.replace('.yaml', '')),
                    'marker_count': len(data.get('markers', {})),
                    'anchors': data.get('anchors', []),
                    'active': fn == active_filename,
                })
            except Exception:
                maps.append({
                    'filename': fn, 'name': fn, 'marker_count': 0,
                    'anchors': [], 'active': fn == active_filename,
                })
        return maps

    def _load_markers(self, path):
        """Load marker positions from YAML file."""
        if not path or not os.path.exists(path):
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

    # --- Menu ---

    def _load_menu(self):
        """Load menu from YAML config file."""
        path = self._menu_file
        if not path or not os.path.exists(path):
            return {}
        try:
            with open(path, 'r') as f:
                return yaml.safe_load(f) or {}
        except Exception as e:
            self.get_logger().error(f'Failed to load menu: {e}')
            return {}

    # --- Locations (tables / home) ---

    def _load_locations(self):
        """Load named locations from the active map YAML."""
        path = self._active_map_file
        if not path or not os.path.exists(path):
            return {}
        try:
            with open(path, 'r') as f:
                data = yaml.safe_load(f) or {}
            return data.get('locations', {})
        except Exception:
            return {}

    def _save_location(self, loc_id, name, x, y, is_home):
        """Save or update a named location in the active map YAML."""
        path = self._active_map_file
        if not path:
            return False
        try:
            if os.path.exists(path):
                with open(path, 'r') as f:
                    data = yaml.safe_load(f) or {}
            else:
                data = {}

            if 'locations' not in data:
                data['locations'] = {}

            if is_home:
                for lid, loc in data['locations'].items():
                    loc['is_home'] = False

            data['locations'][loc_id] = {
                'name': name,
                'x': round(float(x), 3),
                'y': round(float(y), 3),
                'is_home': bool(is_home),
            }

            with open(path, 'w') as f:
                yaml.safe_dump(data, f, default_flow_style=False, sort_keys=False)
            return True
        except Exception as e:
            self.get_logger().error(f'Failed to save location: {e}')
            return False

    def _delete_location(self, loc_id):
        """Delete a named location from the active map YAML."""
        path = self._active_map_file
        if not path or not os.path.exists(path):
            return False
        try:
            with open(path, 'r') as f:
                data = yaml.safe_load(f) or {}
            locs = data.get('locations', {})
            if loc_id not in locs:
                return False
            del locs[loc_id]
            data['locations'] = locs
            with open(path, 'w') as f:
                yaml.safe_dump(data, f, default_flow_style=False, sort_keys=False)
            return True
        except Exception as e:
            self.get_logger().error(f'Failed to delete location: {e}')
            return False

    def _get_home_location(self):
        """Return (x, y) of the home location, or None."""
        locs = self._load_locations()
        for loc in locs.values():
            if loc.get('is_home'):
                return (float(loc['x']), float(loc['y']))
        return None

    # --- Orders ---

    def _create_order(self, items, total):
        """Create a new order, return the order dict."""
        with self._orders_lock:
            order = {
                'id': self._next_order_id,
                'items': items,
                'total': round(float(total), 2),
                'status': 'pending',
                'timestamp': datetime.datetime.now().isoformat(),
            }
            self._next_order_id += 1
            self._orders.append(order)
            return dict(order)

    def _update_order_status(self, order_id, status):
        """Update order status, return True if found."""
        with self._orders_lock:
            for order in self._orders:
                if order['id'] == order_id:
                    order['status'] = status
                    return True
        return False

    def _get_all_orders(self):
        """Return a copy of all orders."""
        with self._orders_lock:
            return [dict(o) for o in self._orders]

    def _schedule_return_home(self):
        """Schedule a return-to-home DeliveryGoal after return_delay seconds."""
        if not self._auto_return_home:
            return
        home = self._get_home_location()
        if not home:
            self.get_logger().warn('No home location defined, skipping return')
            return

        if self._return_timer is not None:
            self._return_timer.cancel()

        self._return_timer = self.create_timer(
            self._return_delay, lambda: self._do_return_home(home))

    def _do_return_home(self, home):
        """Publish DeliveryGoal to navigate to home coordinates."""
        if self._return_timer is not None:
            self._return_timer.cancel()
            self._return_timer = None

        goal = DeliveryGoal()
        goal.target_marker_id = 255
        goal.destination.x = float(home[0])
        goal.destination.y = float(home[1])
        goal.destination.z = 0.0
        self.delivery_goal_pub.publish(goal)
        self.get_logger().info(
            f'Return-to-home: navigating to ({home[0]:.2f}, {home[1]:.2f})')

    def _dispatch_to_location(self, loc_id):
        """Publish DeliveryGoal to navigate to a named location."""
        locs = self._load_locations()

        if loc_id == '__home__':
            home = self._get_home_location()
            if not home:
                self.get_logger().warn('No home location defined')
                return
            goal = DeliveryGoal()
            goal.target_marker_id = 255
            goal.destination.x = float(home[0])
            goal.destination.y = float(home[1])
            goal.destination.z = 0.0
            self.delivery_goal_pub.publish(goal)
            self.get_logger().info(
                f'Dispatched to home ({home[0]:.2f}, {home[1]:.2f})')
            return

        loc = locs.get(loc_id)
        if not loc:
            self.get_logger().warn(f'Location not found: {loc_id}')
            return

        goal = DeliveryGoal()
        goal.target_marker_id = 255
        goal.destination.x = float(loc['x'])
        goal.destination.y = float(loc['y'])
        goal.destination.z = 0.0
        self.delivery_goal_pub.publish(goal)
        self.get_logger().info(
            f'Dispatched to {loc.get("name", loc_id)} '
            f'({loc["x"]:.2f}, {loc["y"]:.2f})')

    # --- ROS2 Callbacks ---

    def _image_cb(self, msg: CompressedImage):
        with self._frame_lock:
            self._latest_frame = bytes(msg.data)
        with self._fps_lock:
            self._frame_count += 1
            now = time.monotonic()
            elapsed = now - self._fps_window_start
            if elapsed >= 1.0:
                self._camera_fps = self._frame_count / elapsed
                self._frame_count = 0
                self._fps_window_start = now

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

    def _imu_cb(self, msg: Imu):
        with self._imu_lock:
            self._imu_gyro_z = float(msg.angular_velocity.z)
            self._imu_last_time = time.time()
            self._imu_ever_received = True

    def _heading_cb(self, msg: Float64):
        with self._imu_lock:
            self._imu_heading_deg = msg.data

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

    def _approach_status_cb(self, msg: ApproachStatus):
        with self._approach_lock:
            self._approach_status = {
                'target_id': int(msg.target_marker_id),
                'armed': bool(msg.armed),
                'visible': bool(msg.marker_visible),
                'size_norm': float(msg.size_norm),
                'state': str(msg.state),
            }

    def _delivery_status_cb(self, msg: DeliveryStatus):
        with self._delivery_lock:
            self._delivery_status = {
                'state': msg.state,
                'target_marker_id': int(msg.target_marker_id),
                'current_waypoint_index': int(msg.current_waypoint_index),
                'total_waypoints': int(msg.total_waypoints),
                'distance_to_waypoint': round(float(msg.distance_to_waypoint), 2),
                'heading_error': round(float(msg.heading_error), 1),
                'message': msg.message,
            }

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
        now = time.time()
        with self._cal_lock:
            if not self._calibrating:
                return
            # Throttle detection attempts to ~2 Hz to avoid blocking the executor
            if now - self._cal_last_attempt < 0.5:
                return
            self._cal_last_attempt = now

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
            gray, (self._cal_cols, self._cal_rows),
            cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_FAST_CHECK
        )

        corner_list = None
        if found:
            criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
            corners_refined = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
            corner_list = corners_refined.reshape(-1, 2).tolist()

            with self._cal_lock:
                # Enforce minimum 1s between captures for pose variety
                if now - self._cal_last_capture < 1.0:
                    found = False
                else:
                    self._cal_last_capture = now
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
        app.router.add_get('/api/maps', self._handle_list_maps)
        app.router.add_post('/api/maps', self._handle_create_map)
        app.router.add_delete('/api/maps/{filename}', self._handle_delete_map)
        app.router.add_get('/api/maps/active', self._handle_get_active_map)
        app.router.add_post('/api/maps/active', self._handle_set_active_map)
        app.router.add_delete('/api/maps/markers/{marker_id}', self._handle_delete_marker)
        app.router.add_get('/order', self._handle_order_page)
        app.router.add_get('/admin', self._handle_admin_page)
        app.router.add_get('/api/menu', self._handle_get_menu)
        app.router.add_get('/api/orders', self._handle_get_orders)
        app.router.add_post('/api/orders', self._handle_create_order)
        app.router.add_get('/api/locations', self._handle_get_locations)
        app.router.add_get('/ws', self._handle_ws)
        runner = web.AppRunner(app)
        loop.run_until_complete(runner.setup())
        site = web.TCPSite(runner, '0.0.0.0', self.port, reuse_address=True)
        loop.run_until_complete(site.start())
        loop.run_forever()

    async def _handle_index(self, request):
        path = os.path.join(STATIC_DIR, 'index.html')
        return web.FileResponse(path)

    async def _handle_order_page(self, request):
        path = os.path.join(STATIC_DIR, 'order.html')
        return web.FileResponse(path)

    async def _handle_admin_page(self, request):
        path = os.path.join(STATIC_DIR, 'admin.html')
        return web.FileResponse(path)

    async def _handle_get_menu(self, request):
        return web.json_response(self._menu_data)

    async def _handle_get_orders(self, request):
        return web.json_response(self._get_all_orders())

    async def _handle_create_order(self, request):
        try:
            body = await request.json()
        except Exception:
            return web.json_response({'error': 'Invalid JSON'}, status=400)

        items = body.get('items', [])
        total = body.get('total', 0)
        if not items:
            return web.json_response({'error': 'No items'}, status=400)

        order = self._create_order(items, total)
        self.get_logger().info(
            f'New order #{order["id"]}: {len(items)} items, '
            f'{total} {self._menu_data.get("menu", {}).get("currency", "TL")}')

        self._push_to_clients({'new_order': order})
        self._schedule_return_home()

        return web.json_response(order, status=201)

    async def _handle_get_locations(self, request):
        return web.json_response(self._load_locations())

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
        self._marker_map = self._load_markers(self._active_map_file)
        marker_list = [
            {'id': mid, **pos} for mid, pos in self._marker_map.items()
        ]
        return web.json_response(marker_list)

    async def _handle_list_maps(self, request):
        return web.json_response(self._list_maps())

    async def _handle_create_map(self, request):
        try:
            body = await request.json()
        except Exception:
            return web.json_response({'error': 'Invalid JSON'}, status=400)

        name = body.get('name', '').strip()
        if not name:
            return web.json_response({'error': 'Name is required'}, status=400)

        filename = name.lower().replace(' ', '_') + '.yaml'
        path = os.path.join(MAPS_DIR, filename)
        if os.path.exists(path):
            return web.json_response({'error': 'Map already exists'}, status=409)

        data = {
            'name': name,
            'marker_size': body.get('marker_size', 0.17),
            'dictionary': body.get('dictionary', 'DICT_4X4_50'),
            'anchors': list(body.get('anchors', [])),
            'markers': body.get('markers', {}),
        }
        # Ensure marker keys are ints in YAML
        if data['markers']:
            data['markers'] = {
                int(k): v for k, v in data['markers'].items()
            }

        with open(path, 'w') as f:
            yaml.safe_dump(data, f, default_flow_style=False, sort_keys=False)

        self.get_logger().info(f'Created map: {filename}')
        return web.json_response({'filename': filename, 'name': name})

    async def _handle_delete_map(self, request):
        filename = request.match_info['filename']
        if not filename.endswith('.yaml'):
            filename += '.yaml'
        path = os.path.join(MAPS_DIR, filename)
        if not os.path.exists(path):
            return web.json_response({'error': 'Not found'}, status=404)

        # Don't allow deleting the active map
        try:
            with open(ACTIVE_MAP_FILE, 'r') as f:
                active = f.read().strip()
            if filename == active:
                return web.json_response({'error': 'Cannot delete active map'}, status=400)
        except Exception:
            pass

        os.remove(path)
        self.get_logger().info(f'Deleted map: {filename}')
        return web.json_response({'deleted': filename})

    async def _handle_get_active_map(self, request):
        maps = self._list_maps()
        active = next((m for m in maps if m['active']), None)
        if active:
            return web.json_response(active)
        return web.json_response({'error': 'No active map'}, status=404)

    async def _handle_set_active_map(self, request):
        try:
            body = await request.json()
        except Exception:
            return web.json_response({'error': 'Invalid JSON'}, status=400)

        filename = body.get('filename', '').strip()
        if not filename:
            return web.json_response({'error': 'filename is required'}, status=400)

        if not self._set_active_map(filename):
            return web.json_response({'error': 'Map not found'}, status=404)

        # Restart localization with new map
        self._stop_localization()
        self._start_localization()
        self._broadcast_loc_stack_status()

        self.get_logger().info(f'Switched to map: {filename}')
        return web.json_response({'active': filename})

    async def _handle_delete_marker(self, request):
        try:
            marker_id = int(request.match_info['marker_id'])
        except (ValueError, KeyError):
            return web.json_response({'error': 'Invalid marker ID'}, status=400)

        path = self._active_map_file
        if not path or not os.path.exists(path):
            return web.json_response({'error': 'No active map'}, status=404)

        try:
            with open(path, 'r') as f:
                data = yaml.safe_load(f) or {}
        except Exception:
            return web.json_response({'error': 'Failed to read map'}, status=500)

        markers = data.get('markers', {})
        if marker_id not in markers and str(marker_id) not in markers:
            return web.json_response({'error': f'Marker {marker_id} not found'}, status=404)

        markers.pop(marker_id, None)
        markers.pop(str(marker_id), None)
        data['markers'] = markers

        anchors = data.get('anchors', [])
        if marker_id in anchors:
            anchors.remove(marker_id)
            data['anchors'] = anchors

        try:
            with open(path, 'w') as f:
                yaml.safe_dump(data, f, default_flow_style=False, sort_keys=False)
        except Exception:
            return web.json_response({'error': 'Failed to write map'}, status=500)

        self._marker_map = self._load_markers(path)
        self.get_logger().info(f'Deleted marker {marker_id} from active map')

        marker_list = [
            {'id': mid, **pos} for mid, pos in self._marker_map.items()
        ]
        return web.json_response({'deleted': marker_id, 'markers': marker_list})

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
                    elif 'approach' in data:
                        ap = data['approach']
                        if 'armed' in ap:
                            self.approach_armed_pub.publish(
                                Bool(data=bool(ap['armed'])))
                        if 'target_id' in ap:
                            self.approach_target_pub.publish(
                                Int32(data=int(ap['target_id'])))
                    elif 'delivery' in data:
                        d = data['delivery']
                        action = d.get('action', '')
                        if action == 'start':
                            goal = DeliveryGoal()
                            goal.target_marker_id = int(d.get('target_marker_id', 255))
                            if 'destination' in d:
                                goal.destination.x = float(d['destination']['x'])
                                goal.destination.y = float(d['destination']['y'])
                                goal.destination.z = 0.0
                            for wp in d.get('waypoints', []):
                                pt = Point()
                                pt.x = float(wp['x'])
                                pt.y = float(wp['y'])
                                pt.z = 0.0
                                goal.waypoints.append(pt)
                            self.delivery_goal_pub.publish(goal)
                        elif action == 'cancel':
                            self.delivery_cancel_pub.publish(Empty())
                        elif action == 'pause':
                            self.delivery_pause_pub.publish(Bool(data=True))
                        elif action == 'resume':
                            self.delivery_pause_pub.publish(Bool(data=False))
                    elif 'ping' in data:
                        await ws.send_json({
                            'pong': data['ping'],
                            'server_ts': int(time.time() * 1000),
                        })
                    elif 'calibration' in data:
                        self._handle_calibration_cmd(data['calibration'])
                    elif 'localization' in data:
                        self._handle_localization_cmd(data['localization'])
                    elif 'order_action' in data:
                        self._handle_order_action(data['order_action'])
                    elif 'dispatch' in data:
                        self._handle_dispatch(data['dispatch'])
                    elif 'save_location' in data:
                        self._handle_save_location(data['save_location'])
                    elif 'delete_location' in data:
                        self._handle_delete_location_cmd(
                            data['delete_location'])
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
                self._cal_last_attempt = 0.0
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

    def _handle_localization_cmd(self, cmd):
        """Handle localization stack start/stop from WebSocket."""
        if cmd == 'start':
            self._start_localization()
        elif cmd == 'stop':
            self._stop_localization()
        self._broadcast_loc_stack_status()

    def _handle_order_action(self, data):
        """Handle accept/reject from admin."""
        order_id = data.get('id')
        action = data.get('action', '')
        if order_id is None or action not in ('accept', 'reject'):
            return
        status = 'accepted' if action == 'accept' else 'rejected'
        if self._update_order_status(int(order_id), status):
            self._push_to_clients({'order_update': {'id': int(order_id), 'status': status}})
            self.get_logger().info(f'Order #{order_id} {status}')

    def _handle_dispatch(self, data):
        """Handle dispatch-to-location from admin."""
        loc_id = data.get('location_id', '')
        if loc_id:
            self._dispatch_to_location(loc_id)

    def _handle_save_location(self, data):
        """Handle save location from admin."""
        loc_id = data.get('id', '')
        name = data.get('name', loc_id)
        x = data.get('x', 0.0)
        y = data.get('y', 0.0)
        is_home = data.get('is_home', False)
        if not loc_id:
            return
        if self._save_location(loc_id, name, x, y, is_home):
            self.get_logger().info(f'Saved location: {name} ({x:.2f}, {y:.2f})')
            self._push_to_clients({'locations_update': self._load_locations()})

    def _handle_delete_location_cmd(self, data):
        """Handle delete location from admin."""
        loc_id = data.get('id', '')
        if loc_id and self._delete_location(loc_id):
            self.get_logger().info(f'Deleted location: {loc_id}')
            self._push_to_clients({'locations_update': self._load_locations()})

    def _start_localization(self):
        """Launch the localization stack as a subprocess."""
        with self._loc_proc_lock:
            if self._loc_proc is not None and self._loc_proc.poll() is None:
                return
            self._loc_error = ''
            try:
                map_arg = ''
                if self._active_map_file and os.path.exists(self._active_map_file):
                    map_arg = f' marker_map_file:={self._active_map_file}'
                launch_cmd = (
                    'bash -c "'
                    'source /opt/ros/jazzy/setup.bash && '
                    'source /home/pi/ros2_ws/install/setup.bash && '
                    'ros2 launch delivery_robot_bringup localization.launch.py'
                    f'{map_arg}'
                    '"'
                )
                self._loc_proc = subprocess.Popen(
                    launch_cmd,
                    shell=True,
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.PIPE,
                    preexec_fn=os.setsid,
                )
                self.get_logger().info('Localization stack started')
            except Exception as e:
                self._loc_error = str(e)
                self.get_logger().error(f'Failed to start localization: {e}')

    def _stop_localization(self):
        """Stop the localization stack subprocess."""
        with self._loc_proc_lock:
            if self._loc_proc is None:
                return
            try:
                os.killpg(os.getpgid(self._loc_proc.pid), signal.SIGINT)
                self._loc_proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(os.getpgid(self._loc_proc.pid), signal.SIGKILL)
                self._loc_proc.wait(timeout=2)
            except Exception:
                pass
            self._loc_proc = None
            self._loc_error = ''
            self.get_logger().info('Localization stack stopped')

    def _is_localization_running(self):
        """Check if the localization subprocess is alive."""
        with self._loc_proc_lock:
            if self._loc_proc is None:
                return False
            ret = self._loc_proc.poll()
            if ret is not None:
                if ret != 0:
                    try:
                        err = self._loc_proc.stderr.read().decode()[-200:]
                    except Exception:
                        err = ''
                    self._loc_error = f'Exited with code {ret}' + (f': {err}' if err else '')
                self._loc_proc = None
                return False
            return True

    def _broadcast_loc_stack_status(self):
        """Push localization stack running status to all clients."""
        data = {
            'loc_stack': {
                'running': self._is_localization_running(),
                'error': self._loc_error,
            }
        }
        self._push_to_clients(data)

    async def _push_status(self, ws: web.WebSocketResponse):
        """Push motor status + localization data at 5Hz, point cloud when available."""
        try:
            while not ws.closed:
                # Motor status
                with self._status_lock:
                    status = dict(self._motor_status)
                with self._fps_lock:
                    status['camera_fps'] = round(self._camera_fps, 1)
                status['connected'] = True
                status['server_ts'] = int(time.time() * 1000)

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

                with self._imu_lock:
                    imu_active = self._imu_ever_received and (now - self._imu_last_time < 2.0)
                    imu_heading = round(self._imu_heading_deg, 1)
                    imu_gyro_z = round(math.degrees(self._imu_gyro_z), 2)

                if not ekf_active and imu_active:
                    position['yaw'] = imu_heading

                localization = {
                    'vslam_state': vslam_state,
                    'ekf_active': ekf_active,
                    'imu_active': imu_active,
                    'imu_heading': imu_heading,
                    'imu_gyro_z': imu_gyro_z,
                    'position': position,
                    'aruco_detections': aruco_dets,
                }

                with self._approach_lock:
                    approach = dict(self._approach_status)

                with self._delivery_lock:
                    delivery = dict(self._delivery_status)

                msg = {
                    'status': status,
                    'localization': localization,
                    'approach': approach,
                    'delivery': delivery,
                    'loc_stack': {
                        'running': self._is_localization_running(),
                        'error': self._loc_error,
                    },
                }

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
        self._stop_localization()
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
