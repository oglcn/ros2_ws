#!/usr/bin/env python3
"""
Closed-loop visual servoing toward a target ArUco marker.

Consumes /aruco/detections (pixel corners) and drives the robot via
/cmd_vel.  No camera calibration needed -- all control is based on
apparent marker position and size in the image.

Safety: the node starts DISARMED and will never move the motors until
explicitly armed via the /aruco_approach/armed topic.  It still tracks
and publishes status while disarmed so the UI can show live feedback.

Speed contract: all power parameters are dimensionless [0, 1] values
that map directly to motor duty-cycle proportions.  There are no m/s
or rad/s quantities here because the robot has no wheel encoders.

Control strategy:
  Far from marker  -> rotate to center the marker horizontally
  Close to marker  -> mecanum strafe instead (avoids swinging the
                      camera and losing the marker from the FOV)
  Forward speed    -> proportional to remaining distance, scaled
                      down when the marker is off-center

State machine:
  IDLE       -- not armed
  SEARCHING  -- armed, waiting to see the target marker
  TRACKING   -- marker visible, driving toward it
  RECOVERING -- marker lost mid-approach, brief blind correction
  ARRIVED    -- close enough, stopped
  FAILED     -- lost marker and recovery timed out
"""

import time

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, Int32

from delivery_robot_msgs.msg import ArucoDetections, ApproachStatus

STATES = ('IDLE', 'SEARCHING', 'TRACKING', 'RECOVERING', 'ARRIVED', 'FAILED')


class ArucoApproachNode(Node):
    def __init__(self):
        super().__init__('aruco_approach')

        self.declare_parameter('target_marker_id', 0)
        self.declare_parameter('image_width', 640)
        self.declare_parameter('arrival_size_px', 200.0)
        self.declare_parameter('forward_power', 0.15)
        self.declare_parameter('strafe_power', 0.15)
        self.declare_parameter('rotation_power', 0.12)
        self.declare_parameter('lost_timeout', 2.0)
        self.declare_parameter('lost_close_ratio', 0.85)
        self.declare_parameter('visibility_timeout', 0.5)
        self.declare_parameter('control_rate_hz', 10.0)

        self._load_params()
        self.add_on_set_parameters_callback(self._on_param_change)

        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.status_pub = self.create_publisher(
            ApproachStatus, 'aruco_approach/status', 10)

        self.create_subscription(
            ArucoDetections, 'aruco/detections', self._detections_cb, 10)
        self.create_subscription(
            Bool, 'aruco_approach/armed', self._armed_cb, 10)
        self.create_subscription(
            Int32, 'aruco_approach/set_target', self._set_target_cb, 10)

        self._armed = False
        self._state = 'IDLE'
        self._last_seen_time = 0.0
        self._last_ex = 0.0
        self._last_size_norm = 0.0

        self.create_timer(1.0 / self.control_rate_hz, self._control_step)

        self.get_logger().info(
            f'ArUco approach ready (disarmed): '
            f'target_id={self.target_marker_id}, '
            f'powers=({self.forward_power}, {self.strafe_power}, '
            f'{self.rotation_power})')

    # -- Parameter handling --------------------------------------------------

    def _load_params(self):
        self.target_marker_id = int(
            self.get_parameter('target_marker_id').value)
        self.image_width = self.get_parameter('image_width').value
        self.arrival_size_px = self.get_parameter('arrival_size_px').value
        self.forward_power = self.get_parameter('forward_power').value
        self.strafe_power = self.get_parameter('strafe_power').value
        self.rotation_power = self.get_parameter('rotation_power').value
        self.lost_timeout = self.get_parameter('lost_timeout').value
        self.lost_close_ratio = self.get_parameter('lost_close_ratio').value
        self.visibility_timeout = self.get_parameter('visibility_timeout').value
        self.control_rate_hz = self.get_parameter('control_rate_hz').value

    def _on_param_change(self, params):
        for p in params:
            if p.name == 'target_marker_id' and int(p.value) != self.target_marker_id:
                self.get_logger().info(
                    f'Target changed to {p.value}, resetting state')
                self._reset_tracking()
        self._load_params()
        return SetParametersResult(successful=True)

    # -- Topic callbacks -----------------------------------------------------

    def _armed_cb(self, msg: Bool):
        newly_armed = bool(msg.data)
        if newly_armed == self._armed:
            return

        was_armed = self._armed
        self._armed = newly_armed
        if self._armed:
            self.get_logger().info('Armed')
            self._state = 'SEARCHING'
            self._reset_tracking()
        else:
            self.get_logger().info('Disarmed')
            self._state = 'IDLE'
            if was_armed:
                self._publish_stop()

    def _set_target_cb(self, msg: Int32):
        new_id = int(msg.data)
        if new_id != self.target_marker_id:
            self.get_logger().info(f'Target set to marker {new_id}')
            self.target_marker_id = new_id
            self._reset_tracking()
            if self._armed:
                self._state = 'SEARCHING'

    def _detections_cb(self, msg: ArucoDetections):
        for i, mid in enumerate(msg.marker_ids):
            if int(mid) != self.target_marker_id:
                continue
            base = i * 8
            if base + 7 >= len(msg.corners):
                break
            corners = [
                (float(msg.corners[base]),     float(msg.corners[base + 1])),
                (float(msg.corners[base + 2]), float(msg.corners[base + 3])),
                (float(msg.corners[base + 4]), float(msg.corners[base + 5])),
                (float(msg.corners[base + 6]), float(msg.corners[base + 7])),
            ]

            cx = sum(c[0] for c in corners) / 4.0
            img_cx = self.image_width / 2.0
            self._last_ex = (cx - img_cx) / img_cx
            self._last_size_norm = (
                self._mean_edge_length(corners) / self.arrival_size_px)
            self._last_seen_time = time.time()
            return

    # -- Control loop --------------------------------------------------------

    def _control_step(self):
        visible = (time.time() - self._last_seen_time) < self.visibility_timeout

        if not self._armed:
            self._state = 'IDLE'
            self._publish_status(visible)
            return

        if self._state in ('ARRIVED', 'FAILED'):
            self._publish_status(visible)
            return

        if visible:
            self._step_tracking()
        else:
            self._step_not_visible()

        self._publish_status(visible)

    def _step_tracking(self):
        ex = self._last_ex
        size_norm = self._last_size_norm
        self._state = 'TRACKING'

        if size_norm >= 1.0:
            self.get_logger().info('Arrived at target marker')
            self._state = 'ARRIVED'
            self._publish_stop()
            return

        blend = max(0.0, min(1.0, size_norm))

        az = -self.rotation_power * ex * (1.0 - blend)
        ly = -self.strafe_power * ex * blend
        forward_scale = max(0.0, 1.0 - abs(ex))
        lx = self.forward_power * (1.0 - size_norm) * forward_scale

        self.get_logger().info(
            f'TRACKING ex={ex:.2f} size={size_norm:.2f} '
            f'-> lx={lx:.3f} ly={ly:.3f} az={az:.3f}',
            throttle_duration_sec=1.0)
        self._publish_cmd(lx, ly, az)

    def _step_not_visible(self):
        age = time.time() - self._last_seen_time

        if self._state not in ('TRACKING', 'RECOVERING'):
            self._state = 'SEARCHING'
            self._publish_stop()
            return

        if self._last_size_norm >= self.lost_close_ratio:
            self.get_logger().info(
                'Marker lost at close range -- assuming arrived')
            self._state = 'ARRIVED'
            self._publish_stop()
            return

        if age < self.lost_timeout:
            self._state = 'RECOVERING'
            direction = 1.0 if self._last_ex > 0 else -1.0
            blend = max(0.0, min(1.0, self._last_size_norm))
            az = -self.rotation_power * 0.5 * direction * (1.0 - blend)
            ly = -self.strafe_power * 0.5 * direction * blend
            self._publish_cmd(0.0, ly, az)
        else:
            self.get_logger().warn('Recovery timed out -- approach failed')
            self._state = 'FAILED'
            self._publish_stop()

    # -- Helpers -------------------------------------------------------------

    def _reset_tracking(self):
        self._last_seen_time = 0.0
        self._last_ex = 0.0
        self._last_size_norm = 0.0

    @staticmethod
    def _mean_edge_length(corners):
        def dist(a, b):
            return ((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2) ** 0.5
        return sum(
            dist(corners[i], corners[(i + 1) % 4]) for i in range(4)) / 4.0

    def _publish_cmd(self, lx, ly, az):
        msg = Twist()
        msg.linear.x = float(lx)
        msg.linear.y = float(ly)
        msg.angular.z = float(az)
        self.cmd_pub.publish(msg)

    def _publish_stop(self):
        self._publish_cmd(0.0, 0.0, 0.0)

    def _publish_status(self, visible):
        msg = ApproachStatus()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.target_marker_id = int(self.target_marker_id)
        msg.armed = bool(self._armed)
        msg.marker_visible = bool(visible)
        msg.size_norm = float(self._last_size_norm) if visible else 0.0
        msg.state = self._state
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
