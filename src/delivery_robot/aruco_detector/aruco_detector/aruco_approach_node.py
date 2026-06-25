#!/usr/bin/env python3
"""
Drives the robot toward a target ArUco marker using closed-loop visual
servoing on pixel position and apparent marker size -- no camera
calibration required, just /aruco/detections.

Armed/disarmed: the node never drives unless armed via the
'aruco_approach/armed' topic (std_msgs/Bool). It starts disarmed, so
detecting the marker alone never moves the robot -- a Start command is
required. While disarmed it still tracks and publishes status (so the
UI can show live distance feedback) but always commands zero velocity.

Control strategy:
  - Lateral centering blends from rotation (far away, small marker) to
    pure mecanum strafe (close up, large marker). Strafing doesn't swing
    the camera, so it keeps the marker in frame during the final approach
    when a body rotation would easily push it out of the FOV.
  - Forward speed scales down as the marker grows (apparent size) and as
    centering error grows, so it naturally decelerates into the target
    instead of overshooting.

Marker-loss handling (no wheel encoders on this robot, so blind driving
is kept short and bounded):
  - Lost right after being seen large/close -> assume arrival, stop.
  - Lost while still small/far -> brief (lost_recovery_timeout) blind
    correction continuing the last known turn/strafe direction, then
    abort if the marker doesn't reappear.
"""

import time

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool

from delivery_robot_msgs.msg import ArucoDetections, ApproachStatus


class ArucoApproachNode(Node):
    def __init__(self):
        super().__init__('aruco_approach')

        self.declare_parameter('target_marker_id', 0)
        self.declare_parameter('image_width', 640)
        self.declare_parameter('image_height', 480)
        self.declare_parameter('arrival_size_px', 220.0)
        self.declare_parameter('max_forward_speed', 0.8)
        self.declare_parameter('max_strafe_speed', 0.8)
        self.declare_parameter('max_rotate_speed', 0.7)
        self.declare_parameter('lost_recovery_timeout', 1.5)
        self.declare_parameter('lost_arrival_size_ratio', 0.85)
        self.declare_parameter('invert_lateral', False)
        self.declare_parameter('invert_rotation', False)
        self.declare_parameter('control_rate_hz', 10.0)

        self._load_params()
        self.add_on_set_parameters_callback(self._on_param_change)

        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.status_pub = self.create_publisher(ApproachStatus, 'aruco_approach/status', 10)
        self.create_subscription(
            ArucoDetections, 'aruco/detections', self._detections_cb, 10
        )
        self.create_subscription(Bool, 'aruco_approach/armed', self._armed_cb, 10)

        self._armed = False
        self._visible = False
        self._last_seen_time = 0.0
        self._last_ex = 0.0
        self._last_size_norm = 0.0
        self._drive_state = 'IDLE'  # IDLE -> TRACKING -> RECOVERING -> ARRIVED / FAILED

        self.create_timer(1.0 / self.control_rate_hz, self._control_step)

        self.get_logger().info(
            f'ArUco approach ready (disarmed): target_id={self.target_marker_id}, '
            f'arrival_size_px={self.arrival_size_px}'
        )

    def _load_params(self):
        # Force int: launch may pass this as a string ("2"), which would make
        # `int(mid) == self.target_marker_id` always False and the target
        # would never be matched.
        self.target_marker_id = int(self.get_parameter('target_marker_id').value)
        self.image_width = self.get_parameter('image_width').value
        self.image_height = self.get_parameter('image_height').value
        self.arrival_size_px = self.get_parameter('arrival_size_px').value
        self.max_forward_speed = self.get_parameter('max_forward_speed').value
        self.max_strafe_speed = self.get_parameter('max_strafe_speed').value
        self.max_rotate_speed = self.get_parameter('max_rotate_speed').value
        self.lost_recovery_timeout = self.get_parameter('lost_recovery_timeout').value
        self.lost_arrival_size_ratio = self.get_parameter('lost_arrival_size_ratio').value
        self.invert_lateral = self.get_parameter('invert_lateral').value
        self.invert_rotation = self.get_parameter('invert_rotation').value
        self.control_rate_hz = self.get_parameter('control_rate_hz').value

    def _on_param_change(self, params):
        for p in params:
            if p.name == 'target_marker_id' and p.value != self.target_marker_id:
                self.get_logger().info(
                    f'Target marker changed to {p.value}, resetting approach state'
                )
                self._drive_state = 'IDLE'
                self._last_seen_time = 0.0
        self._load_params()
        return SetParametersResult(successful=True)

    def _armed_cb(self, msg: Bool):
        was_armed = self._armed
        self._armed = bool(msg.data)
        if self._armed != was_armed:
            self.get_logger().info('Armed -- approach will drive' if self._armed
                                    else 'Disarmed -- holding position')
            self._drive_state = 'IDLE'

    def _detections_cb(self, msg: ArucoDetections):
        """Always update live visibility/size, regardless of armed state,
        so the UI has fresh distance feedback even before Start is pressed."""
        target = None
        for i, mid in enumerate(msg.marker_ids):
            if int(mid) == self.target_marker_id:
                base = i * 8
                if base + 7 < len(msg.corners):
                    target = [
                        (msg.corners[base], msg.corners[base + 1]),
                        (msg.corners[base + 2], msg.corners[base + 3]),
                        (msg.corners[base + 4], msg.corners[base + 5]),
                        (msg.corners[base + 6], msg.corners[base + 7]),
                    ]
                break

        if target is None:
            return

        cx = sum(c[0] for c in target) / 4.0
        img_cx = self.image_width / 2.0

        self._last_ex = (cx - img_cx) / img_cx
        self._last_size_norm = self._apparent_size(target) / self.arrival_size_px
        self._last_seen_time = time.time()

    @staticmethod
    def _apparent_size(corners):
        def dist(a, b):
            return ((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2) ** 0.5
        sides = [dist(corners[i], corners[(i + 1) % 4]) for i in range(4)]
        return sum(sides) / 4.0

    def _control_step(self):
        self._visible = (time.time() - self._last_seen_time) < 0.3

        if not self._armed:
            self._drive_state = 'IDLE'
            self._publish_stop()
            self._publish_status()
            return

        if self._drive_state in ('ARRIVED', 'FAILED'):
            self._publish_stop()
            self._publish_status()
            return

        if self._visible:
            self._track_step()
        else:
            age = time.time() - self._last_seen_time
            if self._drive_state not in ('TRACKING', 'RECOVERING'):
                self._publish_stop()  # never seen the target yet -- nothing to do
            elif self._last_size_norm >= self.lost_arrival_size_ratio:
                self.get_logger().info('Marker lost at close range -- assuming arrival.')
                self._drive_state = 'ARRIVED'
                self._publish_stop()
            elif age < self.lost_recovery_timeout:
                self._drive_state = 'RECOVERING'
                self._recovery_step()
            else:
                self.get_logger().warn('Marker lost and not recovered -- aborting approach.')
                self._drive_state = 'FAILED'
                self._publish_stop()

        self._publish_status()

    def _track_step(self):
        ex = self._last_ex
        size_norm = self._last_size_norm
        self._drive_state = 'TRACKING'

        self.get_logger().info(
            f'tracking ex={ex:.2f} size_norm={size_norm:.2f}',
            throttle_duration_sec=1.0,
        )

        if size_norm >= 1.0:
            self.get_logger().info('Arrived at target marker.')
            self._drive_state = 'ARRIVED'
            self._publish_stop()
            return

        close_factor = max(0.0, min(1.0, size_norm))
        rot_sign = -1.0 if self.invert_rotation else 1.0
        lat_sign = -1.0 if self.invert_lateral else 1.0

        # Far away -> mostly rotate to center. Close up -> mostly strafe,
        # so the camera doesn't swing the marker out of frame.
        az = rot_sign * (-self.max_rotate_speed * ex * (1.0 - close_factor))
        ly = lat_sign * (-self.max_strafe_speed * ex * close_factor)

        forward_scale = max(0.0, 1.0 - abs(ex))
        lx = max(0.0, self.max_forward_speed * (1.0 - size_norm) * forward_scale)

        self.get_logger().info(
            f'cmd lx={lx:.2f} ly={ly:.2f} az={az:.2f}',
            throttle_duration_sec=1.0,
        )
        self._publish(lx, ly, az)

    def _recovery_step(self):
        """Marker dropped out of frame mid-approach. Keep correcting in the
        direction it was last drifting, but don't push forward blindly."""
        ex = self._last_ex
        close_factor = max(0.0, min(1.0, self._last_size_norm))
        rot_sign = -1.0 if self.invert_rotation else 1.0
        lat_sign = -1.0 if self.invert_lateral else 1.0
        direction = 1.0 if ex > 0 else -1.0

        az = rot_sign * (-self.max_rotate_speed * 0.5 * direction * (1.0 - close_factor))
        ly = lat_sign * (-self.max_strafe_speed * 0.5 * direction * close_factor)

        self._publish(0.0, ly, az)

    def _publish(self, lx, ly, az):
        msg = Twist()
        msg.linear.x = float(lx)
        msg.linear.y = float(ly)
        msg.angular.z = float(az)
        self.cmd_pub.publish(msg)

    def _publish_stop(self):
        self._publish(0.0, 0.0, 0.0)

    def _publish_status(self):
        msg = ApproachStatus()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.target_marker_id = int(self.target_marker_id)
        msg.armed = bool(self._armed)
        msg.marker_visible = bool(self._visible)
        msg.size_norm = float(self._last_size_norm) if self._visible else 0.0
        msg.state = self._drive_state
        self.status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ArucoApproachNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
