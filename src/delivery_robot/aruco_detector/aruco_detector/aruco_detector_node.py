#!/usr/bin/env python3
"""
ArUco marker detector node.

Subscribes to camera images and CameraInfo, detects ArUco markers, and for
markers with known world positions, publishes the robot's pose as
PoseWithCovarianceStamped for fusion with the EKF. Also publishes raw
detection data (marker IDs + pixel corners) for UI overlay.
"""

import numpy as np
import cv2
import yaml

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from cv_bridge import CvBridge
from tf2_ros import TransformBroadcaster

from delivery_robot_msgs.msg import ArucoDetections

FAST_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1,
)

ARUCO_DICTS = {
    'DICT_4X4_50': cv2.aruco.DICT_4X4_50,
    'DICT_4X4_100': cv2.aruco.DICT_4X4_100,
    'DICT_4X4_250': cv2.aruco.DICT_4X4_250,
    'DICT_5X5_50': cv2.aruco.DICT_5X5_50,
    'DICT_5X5_100': cv2.aruco.DICT_5X5_100,
    'DICT_6X6_50': cv2.aruco.DICT_6X6_50,
    'DICT_6X6_100': cv2.aruco.DICT_6X6_100,
}


class ArucoDetectorNode(Node):
    def __init__(self):
        super().__init__('aruco_detector')

        self.declare_parameter('marker_size', 0.15)
        self.declare_parameter('dictionary', 'DICT_4X4_50')
        self.declare_parameter('marker_map_file', '')
        self.declare_parameter('detection_rate_hz', 10.0)

        self.marker_size = self.get_parameter('marker_size').value
        dict_name = self.get_parameter('dictionary').value
        marker_map_file = self.get_parameter('marker_map_file').value
        detection_rate = self.get_parameter('detection_rate_hz').value

        dict_id = ARUCO_DICTS.get(dict_name, cv2.aruco.DICT_4X4_50)
        self.aruco_dict = cv2.aruco.Dictionary_get(dict_id)
        self.aruco_params = cv2.aruco.DetectorParameters_create()

        self._marker_map_file = marker_map_file
        self.marker_map = self._load_marker_map(marker_map_file)
        self._anchor_ids = self._load_anchor_ids(marker_map_file)
        self._observations = {}  # marker_id -> list of (x, y, z) observations
        self._min_obs = 15  # observations needed before promoting a marker
        self._max_std = 0.15  # max std dev (meters) for observations to be consistent

        self.camera_matrix = None
        self.dist_coeffs = None
        self.bridge = CvBridge()
        self.tf_broadcaster = TransformBroadcaster(self)

        self._min_interval = 1.0 / detection_rate
        self._last_detect_time = 0.0

        self.pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, 'aruco/pose', 10
        )

        self.detections_pub = self.create_publisher(
            ArucoDetections, 'aruco/detections', 10
        )

        self.create_subscription(
            CameraInfo, 'camera/camera_info', self._info_cb, FAST_QOS
        )
        self.create_subscription(
            Image, 'camera/image_raw', self._image_cb, FAST_QOS
        )

        self.get_logger().info(
            f'ArUco detector started: dict={dict_name}, '
            f'marker_size={self.marker_size}m, '
            f'{len(self.marker_map)} known markers'
        )

    def _load_marker_map(self, path):
        """Load marker world positions from YAML."""
        if not path:
            self.get_logger().info('No marker map file, will detect but not localize')
            return {}

        try:
            with open(path, 'r') as f:
                data = yaml.safe_load(f)
            markers = {}
            for mid, pos in data.get('markers', {}).items():
                markers[int(mid)] = np.array([
                    pos.get('x', 0.0),
                    pos.get('y', 0.0),
                    pos.get('z', 0.0),
                ], dtype=np.float64)
            self.get_logger().info(f'Loaded {len(markers)} marker positions from {path}')
            return markers
        except Exception as e:
            self.get_logger().error(f'Failed to load marker map: {e}')
            return {}

    def _load_anchor_ids(self, path):
        """Load the set of anchor marker IDs (manually measured, never overwritten)."""
        if not path:
            return set(self.marker_map.keys())
        try:
            with open(path, 'r') as f:
                data = yaml.safe_load(f)
            anchors = data.get('anchors', [])
            if anchors:
                return set(int(a) for a in anchors)
            return set(self.marker_map.keys())
        except Exception:
            return set(self.marker_map.keys())

    def _save_marker_to_file(self, marker_id, position):
        """Persist a learned marker back to the YAML map file."""
        if not self._marker_map_file:
            return
        try:
            with open(self._marker_map_file, 'r') as f:
                data = yaml.safe_load(f) or {}

            if 'markers' not in data:
                data['markers'] = {}

            data['markers'][int(marker_id)] = {
                'x': round(float(position[0]), 4),
                'y': round(float(position[1]), 4),
                'z': round(float(position[2]), 4),
            }

            with open(self._marker_map_file, 'w') as f:
                yaml.safe_dump(data, f, default_flow_style=False, sort_keys=False)

            self.get_logger().info(f'Saved marker {marker_id} to {self._marker_map_file}')
        except Exception as e:
            self.get_logger().warn(f'Failed to save marker {marker_id}: {e}')

    def _info_cb(self, msg):
        if self.camera_matrix is None:
            self.camera_matrix = np.array(msg.k, dtype=np.float64).reshape(3, 3)
            self.dist_coeffs = np.array(msg.d, dtype=np.float64)
            self.get_logger().info('Camera intrinsics received')

    def _image_cb(self, msg):
        import time
        now = time.time()
        if now - self._last_detect_time < self._min_interval:
            return
        self._last_detect_time = now

        if self.camera_matrix is None:
            return

        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        corners, ids, _ = cv2.aruco.detectMarkers(
            gray, self.aruco_dict, parameters=self.aruco_params
        )

        if ids is None or len(ids) == 0:
            self._publish_detections(msg.header.stamp, np.array([]), [])
            return

        self._publish_detections(msg.header.stamp, ids, corners)

        known_detections = []
        unknown_tvecs = []
        for i, marker_id in enumerate(ids.flatten()):
            rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(
                corners[i:i+1], self.marker_size, self.camera_matrix, self.dist_coeffs
            )
            rvec = rvec[0][0]
            tvec = tvec[0][0]

            self._publish_marker_tf(msg.header.stamp, marker_id, rvec, tvec)

            mid = int(marker_id)
            if mid in self.marker_map:
                known_detections.append((mid, tvec))
            else:
                unknown_tvecs.append((mid, tvec))

        if known_detections:
            yaw = self._publish_fused_pose(msg.header.stamp, known_detections)
            if yaw is not None and len(known_detections) >= 2:
                self._observe_unknown_markers(yaw, known_detections, unknown_tvecs)

    def _publish_detections(self, stamp, ids, corners):
        """Publish raw detection data for the UI overlay."""
        det_msg = ArucoDetections()
        det_msg.header.stamp = stamp
        det_msg.header.frame_id = 'camera_link'

        marker_ids = []
        flat_corners = []
        for i, marker_id in enumerate(ids.flatten()):
            marker_ids.append(int(marker_id))
            for corner in corners[i][0]:
                flat_corners.append(float(corner[0]))
                flat_corners.append(float(corner[1]))

        det_msg.marker_ids = marker_ids
        det_msg.corners = flat_corners
        self.detections_pub.publish(det_msg)

    def _publish_marker_tf(self, stamp, marker_id, rvec, tvec):
        """Broadcast TF from camera_link to detected marker."""
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = 'camera_link'
        t.child_frame_id = f'aruco_marker_{marker_id}'
        t.transform.translation.x = float(tvec[0])
        t.transform.translation.y = float(tvec[1])
        t.transform.translation.z = float(tvec[2])

        rot_matrix, _ = cv2.Rodrigues(rvec)
        quat = self._rotation_matrix_to_quaternion(rot_matrix)
        t.transform.rotation.x = quat[0]
        t.transform.rotation.y = quat[1]
        t.transform.rotation.z = quat[2]
        t.transform.rotation.w = quat[3]

        self.tf_broadcaster.sendTransform(t)

    def _publish_fused_pose(self, stamp, detections):
        """Compute robot 2D pose from all visible known markers.

        Uses pure geometry:
        - 2+ markers: heading from inter-marker vector (no axis convention needed)
        - 1 marker: position only, heading from rough tvec angle
        - Position averaged across all visible markers

        Camera frame: X=right, Y=down, Z=forward
        Map frame: X=along wall, Y=away from wall toward robot

        Returns the computed yaw (or None on failure).
        """
        if len(detections) >= 2:
            # Pick best marker pair for heading: prefer anchor-anchor pairs
            pair = self._select_heading_pair(detections)
            if pair:
                (id1, tvec1), (id2, tvec2) = pair

                dx_cam = tvec2[0] - tvec1[0]
                dz_cam = tvec2[2] - tvec1[2]

                m1 = self.marker_map[id1]
                m2 = self.marker_map[id2]
                dx_map = m2[0] - m1[0]
                dy_map = m2[1] - m1[1]

                angle_cam = np.arctan2(dz_cam, dx_cam)
                angle_map = np.arctan2(dy_map, dx_map)
                yaw = float(angle_map - angle_cam + np.pi / 2)

                both_anchors = id1 in self._anchor_ids and id2 in self._anchor_ids
                yaw_cov = 0.1 if both_anchors else 0.5
            else:
                _, tvec = detections[0]
                yaw = float(-np.pi / 2 - np.arctan2(tvec[0], tvec[2]))
                yaw_cov = 2.0
        else:
            # Single marker: rough heading from angle to marker
            _, tvec = detections[0]
            yaw = float(-np.pi / 2 - np.arctan2(tvec[0], tvec[2]))
            yaw_cov = 2.0

        # Normalize to [-pi, pi]
        yaw = float(np.arctan2(np.sin(yaw), np.cos(yaw)))

        # Position: for each marker, compute camera position in map and average
        positions = []
        for mid, tvec in detections:
            marker_pos = self.marker_map[mid]
            cx = marker_pos[0] - (tvec[0] * np.sin(yaw) + tvec[2] * np.cos(yaw))
            cy = marker_pos[1] + (tvec[0] * np.cos(yaw) - tvec[2] * np.sin(yaw))
            positions.append((cx, cy))

        robot_x = float(np.mean([p[0] for p in positions]))
        robot_y = float(np.mean([p[1] for p in positions]))

        # Camera-to-base_link offset (camera is 5cm forward)
        robot_x -= 0.05 * np.cos(yaw)
        robot_y -= 0.05 * np.sin(yaw)

        # Quaternion from yaw
        qz = float(np.sin(yaw / 2))
        qw = float(np.cos(yaw / 2))

        msg = PoseWithCovarianceStamped()
        msg.header.stamp = stamp
        msg.header.frame_id = 'map'
        msg.pose.pose.position.x = robot_x
        msg.pose.pose.position.y = robot_y
        msg.pose.pose.position.z = 0.0
        msg.pose.pose.orientation.x = 0.0
        msg.pose.pose.orientation.y = 0.0
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw

        cov = [0.0] * 36
        cov[0] = 0.2
        cov[7] = 0.2
        cov[14] = 999.0
        cov[21] = 999.0
        cov[28] = 999.0
        cov[35] = yaw_cov
        msg.pose.covariance = cov

        self.pose_pub.publish(msg)
        return yaw

    def _select_heading_pair(self, detections):
        """Select the best marker pair for heading computation.

        Priority: anchor-anchor > anchor-learned > learned-learned.
        Returns tuple of ((id1, tvec1), (id2, tvec2)) or None.
        """
        anchors = [(mid, tv) for mid, tv in detections if mid in self._anchor_ids]
        learned = [(mid, tv) for mid, tv in detections if mid not in self._anchor_ids]

        if len(anchors) >= 2:
            return (anchors[0], anchors[1])
        if len(anchors) == 1 and len(learned) >= 1:
            return (anchors[0], learned[0])
        if len(learned) >= 2:
            return (learned[0], learned[1])
        return None

    def _observe_unknown_markers(self, yaw, known_detections, unknown_tvecs):
        """Back-project unknown markers into map frame and learn their positions.

        Uses the camera position derived from known markers + yaw to compute
        where unknown markers must be in the map.
        """
        if not unknown_tvecs:
            return

        # Recompute camera map position from known detections
        positions = []
        for mid, tvec in known_detections:
            marker_pos = self.marker_map[mid]
            cx = marker_pos[0] - (tvec[0] * np.sin(yaw) + tvec[2] * np.cos(yaw))
            cy = marker_pos[1] + (tvec[0] * np.cos(yaw) - tvec[2] * np.sin(yaw))
            positions.append((cx, cy))

        cam_x = float(np.mean([p[0] for p in positions]))
        cam_y = float(np.mean([p[1] for p in positions]))

        for mid, tvec in unknown_tvecs:
            # Project marker from camera frame to map frame
            mx = cam_x + tvec[0] * np.sin(yaw) + tvec[2] * np.cos(yaw)
            my = cam_y - tvec[0] * np.cos(yaw) + tvec[2] * np.sin(yaw)
            mz = 0.385  # assume same height as anchor markers

            if mid not in self._observations:
                self._observations[mid] = []
            self._observations[mid].append((mx, my, mz))

            obs = self._observations[mid]
            if len(obs) % 5 == 1:
                self.get_logger().info(
                    f'Marker {mid}: {len(obs)}/{self._min_obs} observations'
                )
            if len(obs) >= self._min_obs:
                xs = np.array([o[0] for o in obs])
                ys = np.array([o[1] for o in obs])
                if np.std(xs) < self._max_std and np.std(ys) < self._max_std:
                    avg_x = float(np.mean(xs))
                    avg_y = float(np.mean(ys))
                    avg_z = float(np.mean([o[2] for o in obs]))
                    pos = np.array([avg_x, avg_y, avg_z], dtype=np.float64)
                    self.marker_map[mid] = pos
                    self._save_marker_to_file(mid, pos)
                    self.get_logger().info(
                        f'Learned marker {mid} at ({avg_x:.3f}, {avg_y:.3f}, {avg_z:.3f}) '
                        f'from {len(obs)} observations '
                        f'(std x={np.std(xs):.4f}, y={np.std(ys):.4f})'
                    )
                    del self._observations[mid]
                elif len(obs) > self._min_obs * 3:
                    # Too noisy, discard oldest half and keep trying
                    self._observations[mid] = obs[len(obs) // 2:]

    @staticmethod
    def _rotation_matrix_to_quaternion(R):
        """Convert 3x3 rotation matrix to quaternion [x, y, z, w]."""
        trace = R[0, 0] + R[1, 1] + R[2, 2]
        if trace > 0:
            s = 0.5 / np.sqrt(trace + 1.0)
            w = 0.25 / s
            x = (R[2, 1] - R[1, 2]) * s
            y = (R[0, 2] - R[2, 0]) * s
            z = (R[1, 0] - R[0, 1]) * s
        elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
            s = 2.0 * np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
            w = (R[2, 1] - R[1, 2]) / s
            x = 0.25 * s
            y = (R[0, 1] + R[1, 0]) / s
            z = (R[0, 2] + R[2, 0]) / s
        elif R[1, 1] > R[2, 2]:
            s = 2.0 * np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
            w = (R[0, 2] - R[2, 0]) / s
            x = (R[0, 1] + R[1, 0]) / s
            y = 0.25 * s
            z = (R[1, 2] + R[2, 1]) / s
        else:
            s = 2.0 * np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
            w = (R[1, 0] - R[0, 1]) / s
            x = (R[0, 2] + R[2, 0]) / s
            y = (R[1, 2] + R[2, 1]) / s
            z = 0.25 * s
        return [float(x), float(y), float(z), float(w)]


def main(args=None):
    rclpy.init(args=args)
    node = ArucoDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
