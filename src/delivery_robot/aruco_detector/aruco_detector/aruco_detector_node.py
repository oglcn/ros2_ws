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

        self.marker_map = self._load_marker_map(marker_map_file)
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
            return

        self._publish_detections(msg.header.stamp, ids, corners)

        for i, marker_id in enumerate(ids.flatten()):
            rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(
                corners[i:i+1], self.marker_size, self.camera_matrix, self.dist_coeffs
            )
            rvec = rvec[0][0]
            tvec = tvec[0][0]

            self._publish_marker_tf(msg.header.stamp, marker_id, rvec, tvec)

            if int(marker_id) in self.marker_map:
                self._publish_robot_pose(msg.header.stamp, marker_id, rvec, tvec)

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

    def _publish_robot_pose(self, stamp, marker_id, rvec, tvec):
        """Compute robot pose in map frame from a known marker and publish."""
        marker_world_pos = self.marker_map[int(marker_id)]

        rot_cam_to_marker, _ = cv2.Rodrigues(rvec)
        t_cam_to_marker = tvec.reshape(3, 1)

        rot_marker_to_cam = rot_cam_to_marker.T
        t_marker_to_cam = -rot_marker_to_cam @ t_cam_to_marker

        robot_x = marker_world_pos[0] + t_marker_to_cam[0, 0]
        robot_y = marker_world_pos[1] + t_marker_to_cam[1, 0]
        robot_z = marker_world_pos[2] + t_marker_to_cam[2, 0]

        quat = self._rotation_matrix_to_quaternion(rot_marker_to_cam)

        msg = PoseWithCovarianceStamped()
        msg.header.stamp = stamp
        msg.header.frame_id = 'map'
        msg.pose.pose.position.x = float(robot_x)
        msg.pose.pose.position.y = float(robot_y)
        msg.pose.pose.position.z = float(robot_z)
        msg.pose.pose.orientation.x = quat[0]
        msg.pose.pose.orientation.y = quat[1]
        msg.pose.pose.orientation.z = quat[2]
        msg.pose.pose.orientation.w = quat[3]

        cov = [0.0] * 36
        cov[0] = 0.05   # x variance
        cov[7] = 0.05   # y variance
        cov[14] = 0.05  # z variance
        cov[21] = 0.1   # roll variance
        cov[28] = 0.1   # pitch variance
        cov[35] = 0.1   # yaw variance
        msg.pose.covariance = cov

        self.pose_pub.publish(msg)

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
