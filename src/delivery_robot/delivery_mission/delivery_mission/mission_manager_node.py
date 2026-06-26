"""Delivery mission manager with waypoint navigation.

Orchestrates point-to-point delivery missions: navigates through
intermediate waypoints using EKF pose feedback (tank-drive style),
then hands off to aruco_approach_node for final marker docking.
"""

import math
import os
import time

import rclpy
from rclpy.node import Node
import yaml

from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Empty, Int32

from delivery_robot_msgs.msg import ApproachStatus, DeliveryGoal, DeliveryStatus


class MissionManagerNode(Node):

    def __init__(self):
        super().__init__('mission_manager')

        self._declare_params()
        self._load_params()

        self._state = 'IDLE'
        self._message = ''
        self._target_marker_id = 0
        self._waypoints = []       # list of (x, y) tuples
        self._wp_idx = 0
        self._waypoint_start_time = 0.0
        self._terminal_state_time = None
        self._paused_from = None   # state to resume to: 'NAVIGATING' or 'APPROACHING'
        self._use_marker = True    # False for arbitrary point destinations

        self._current_x = 0.0
        self._current_y = 0.0
        self._current_yaw = 0.0    # radians
        self._last_ekf_time = None
        self._last_approach_status = None

        self._markers = {}         # {int: (float, float)}
        self._marker_map_path = ''

        self._load_marker_map()

        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.status_pub = self.create_publisher(DeliveryStatus, 'delivery/status', 10)
        self.approach_armed_pub = self.create_publisher(Bool, 'aruco_approach/armed', 10)
        self.approach_target_pub = self.create_publisher(Int32, 'aruco_approach/set_target', 10)

        self.create_subscription(Odometry, 'odometry/filtered', self._ekf_cb, 10)
        self.create_subscription(ApproachStatus, 'aruco_approach/status', self._approach_status_cb, 10)
        self.create_subscription(DeliveryGoal, 'delivery/goal', self._goal_cb, 10)
        self.create_subscription(Empty, 'delivery/cancel', self._cancel_cb, 10)
        self.create_subscription(Bool, 'delivery/pause', self._pause_cb, 10)

        period = 1.0 / self._control_rate
        self.create_timer(period, self._control_loop)

        self.get_logger().info(
            f'Mission manager ready: {len(self._markers)} markers loaded, '
            f'control at {self._control_rate} Hz')

    # ------------------------------------------------------------------
    # Parameters
    # ------------------------------------------------------------------

    def _declare_params(self):
        self.declare_parameter('linear_speed', 0.15)
        self.declare_parameter('angular_speed', 0.12)
        self.declare_parameter('heading_tolerance', 15.0)
        self.declare_parameter('correction_gain', 0.5)
        self.declare_parameter('arrival_radius', 0.5)
        self.declare_parameter('standoff_distance', 0.8)
        self.declare_parameter('waypoint_tolerance', 0.15)
        self.declare_parameter('slowdown_radius', 0.3)
        self.declare_parameter('navigation_timeout', 120.0)
        self.declare_parameter('ekf_timeout', 2.0)
        self.declare_parameter('control_rate', 10.0)
        self.declare_parameter('marker_map_file', '')

    def _load_params(self):
        self._linear_speed = self.get_parameter('linear_speed').value
        self._angular_speed = self.get_parameter('angular_speed').value
        self._heading_tolerance = self.get_parameter('heading_tolerance').value
        self._correction_gain = self.get_parameter('correction_gain').value
        self._arrival_radius = self.get_parameter('arrival_radius').value
        self._standoff_distance = self.get_parameter('standoff_distance').value
        self._waypoint_tolerance = self.get_parameter('waypoint_tolerance').value
        self._slowdown_radius = self.get_parameter('slowdown_radius').value
        self._navigation_timeout = self.get_parameter('navigation_timeout').value
        self._ekf_timeout = self.get_parameter('ekf_timeout').value
        self._control_rate = self.get_parameter('control_rate').value
        self._marker_map_path = self.get_parameter('marker_map_file').value

    # ------------------------------------------------------------------
    # Marker map
    # ------------------------------------------------------------------

    def _load_marker_map(self):
        path = self._marker_map_path
        if not path:
            active_file = os.path.expanduser('~/ros2_ws/aruco_maps/active_map.txt')
            try:
                with open(active_file) as f:
                    filename = f.read().strip()
                path = os.path.expanduser(f'~/ros2_ws/aruco_maps/{filename}')
            except FileNotFoundError:
                self.get_logger().warn(f'Active map pointer not found: {active_file}')
                return

        try:
            with open(path) as f:
                data = yaml.safe_load(f)
        except FileNotFoundError:
            self.get_logger().error(f'Marker map not found: {path}')
            return

        self._markers = {}
        for mid, pos in data.get('markers', {}).items():
            self._markers[int(mid)] = (float(pos['x']), float(pos['y']))

        self.get_logger().info(
            f'Loaded marker map: {path} ({len(self._markers)} markers: '
            f'{list(self._markers.keys())})')

    # ------------------------------------------------------------------
    # ROS callbacks
    # ------------------------------------------------------------------

    def _ekf_cb(self, msg: Odometry):
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self._current_yaw = math.atan2(siny_cosp, cosy_cosp)
        self._current_x = float(msg.pose.pose.position.x)
        self._current_y = float(msg.pose.pose.position.y)
        self._last_ekf_time = time.monotonic()

    def _approach_status_cb(self, msg: ApproachStatus):
        self._last_approach_status = msg
        if self._state != 'APPROACHING':
            return
        if msg.state == 'ARRIVED':
            self.approach_armed_pub.publish(Bool(data=False))
            self._transition_to('ARRIVED',
                                f'Delivered to marker {self._target_marker_id}')
        elif msg.state == 'FAILED':
            self.approach_armed_pub.publish(Bool(data=False))
            self._transition_to('FAILED', 'ArUco approach failed -- marker lost')

    def _goal_cb(self, msg: DeliveryGoal):
        if self._state in ('NAVIGATING', 'APPROACHING', 'PAUSED'):
            self.get_logger().info('New goal received -- cancelling current mission')
            self._cancel_current()

        self._load_marker_map()

        target_id = int(msg.target_marker_id)
        self._use_marker = (target_id != 255)

        if self._use_marker and target_id not in self._markers:
            self._message = f'Unknown marker ID {target_id}'
            self.get_logger().error(self._message)
            self._publish_status()
            return

        if self._last_ekf_time is None or \
                (time.monotonic() - self._last_ekf_time) > self._ekf_timeout:
            self._message = 'No localization -- start localization first'
            self.get_logger().error(self._message)
            self._publish_status()
            return

        self._target_marker_id = target_id

        self._waypoints = []
        for pt in msg.waypoints:
            self._waypoints.append((float(pt.x), float(pt.y)))

        if self._use_marker:
            # Marker destination: final waypoint offset by standoff_distance
            marker_pos = self._markers[target_id]
            prev = self._waypoints[-1] if self._waypoints else \
                (self._current_x, self._current_y)
            mx, my = marker_pos
            dx = prev[0] - mx
            dy = prev[1] - my
            dist = math.sqrt(dx * dx + dy * dy)
            if dist > 0.01:
                standoff_x = mx + (dx / dist) * self._standoff_distance
                standoff_y = my + (dy / dist) * self._standoff_distance
                self._waypoints.append((standoff_x, standoff_y))
            else:
                self._waypoints.append(marker_pos)
            label = f'marker {target_id}'
        else:
            # Arbitrary point destination: navigate directly to the point
            dest = (float(msg.destination.x), float(msg.destination.y))
            self._waypoints.append(dest)
            label = f'point ({dest[0]:.2f}, {dest[1]:.2f})'

        self._wp_idx = 0
        self._waypoint_start_time = time.monotonic()
        self._terminal_state_time = None
        self._paused_from = None

        self._transition_to(
            'NAVIGATING',
            f'Navigating to {label} via {len(self._waypoints)} waypoints')
        self.get_logger().info(
            f'Mission started: target {label}, '
            f'{len(self._waypoints)} waypoints')

    def _cancel_cb(self, msg: Empty):
        if self._state in ('NAVIGATING', 'APPROACHING', 'PAUSED'):
            self.get_logger().info('Mission cancelled')
            self._cancel_current()

    def _pause_cb(self, msg: Bool):
        if msg.data:
            if self._state in ('NAVIGATING', 'APPROACHING'):
                self._paused_from = self._state
                if self._state == 'APPROACHING':
                    self.approach_armed_pub.publish(Bool(data=False))
                self._publish_zero_cmd()
                self._transition_to('PAUSED', 'Mission paused by user')
                self.get_logger().info('Mission paused')
        else:
            if self._state == 'PAUSED':
                resume_to = self._paused_from or 'NAVIGATING'
                if resume_to == 'APPROACHING':
                    self.approach_armed_pub.publish(Bool(data=True))
                self._waypoint_start_time = time.monotonic()
                self._transition_to(resume_to, f'Resumed ({resume_to.lower()})')
                self.get_logger().info(f'Mission resumed -> {resume_to}')

    # ------------------------------------------------------------------
    # Control loop
    # ------------------------------------------------------------------

    def _control_loop(self):
        if self._state in ('ARRIVED', 'FAILED'):
            if self._terminal_state_time is not None and \
                    (time.monotonic() - self._terminal_state_time) > 3.0:
                self._state = 'IDLE'
                self._message = ''
                self._terminal_state_time = None

        self._publish_status()

        if self._state == 'NAVIGATING':
            self._navigate_step()

    def _navigate_step(self):
        now = time.monotonic()

        if self._last_ekf_time is None or \
                (now - self._last_ekf_time) > self._ekf_timeout:
            self._publish_zero_cmd()
            self._paused_from = 'NAVIGATING'
            self._transition_to('PAUSED', 'Localization lost -- paused')
            self.get_logger().warn('EKF stale -- pausing mission')
            return

        if (now - self._waypoint_start_time) > self._navigation_timeout:
            self._publish_zero_cmd()
            self._transition_to(
                'FAILED',
                f'Navigation timeout at waypoint {self._wp_idx + 1}/'
                f'{len(self._waypoints)}')
            self.get_logger().error(self._message)
            return

        x, y, yaw = self._current_x, self._current_y, self._current_yaw
        tx, ty = self._waypoints[self._wp_idx]

        dx = tx - x
        dy = ty - y
        distance = math.sqrt(dx * dx + dy * dy)
        bearing = math.atan2(dy, dx)
        heading_error = self._normalize_angle(bearing - yaw)

        is_final = (self._wp_idx == len(self._waypoints) - 1)

        if is_final:
            if self._use_marker and distance < self._arrival_radius:
                self._begin_approach()
                return
            if not self._use_marker and distance < self._waypoint_tolerance:
                self._publish_zero_cmd()
                self._transition_to(
                    'ARRIVED', 'Arrived at destination')
                self.get_logger().info('Arrived at arbitrary destination')
                return

        if not is_final and distance < self._waypoint_tolerance:
            self._wp_idx += 1
            self._waypoint_start_time = time.monotonic()
            self.get_logger().info(
                f'Waypoint {self._wp_idx}/{len(self._waypoints)} reached')
            return

        twist = Twist()
        if abs(heading_error) > math.radians(self._heading_tolerance):
            twist.angular.z = self._angular_speed if heading_error > 0 \
                else -self._angular_speed
            twist.linear.x = 0.0
        else:
            speed = self._linear_speed
            if distance < self._slowdown_radius:
                speed *= (distance / self._slowdown_radius)
            speed = max(speed, 0.03)
            twist.angular.z = self._correction_gain * (heading_error / math.pi)
            twist.linear.x = speed

        twist.linear.y = 0.0
        self.cmd_vel_pub.publish(twist)

    def _begin_approach(self):
        self._publish_zero_cmd()
        self.approach_armed_pub.publish(Bool(data=False))
        self.approach_target_pub.publish(
            Int32(data=int(self._target_marker_id)))
        self.approach_armed_pub.publish(Bool(data=True))
        self._transition_to(
            'APPROACHING',
            f'Approaching marker {self._target_marker_id}')
        self.get_logger().info(
            f'Handing off to ArUco approach for marker {self._target_marker_id}')

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def _transition_to(self, new_state, message=''):
        self._state = new_state
        self._message = message
        if new_state in ('ARRIVED', 'FAILED'):
            self._terminal_state_time = time.monotonic()

    def _cancel_current(self):
        if self._state == 'APPROACHING' or self._paused_from == 'APPROACHING':
            self.approach_armed_pub.publish(Bool(data=False))
        self._publish_zero_cmd()
        self._state = 'IDLE'
        self._message = 'Mission cancelled'
        self._waypoints = []
        self._wp_idx = 0
        self._paused_from = None
        self._terminal_state_time = None

    def _publish_zero_cmd(self):
        self.cmd_vel_pub.publish(Twist())

    def _publish_status(self):
        msg = DeliveryStatus()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.state = self._state
        msg.target_marker_id = self._target_marker_id
        msg.message = self._message

        if self._waypoints:
            msg.current_waypoint_index = self._wp_idx
            msg.total_waypoints = len(self._waypoints)

            if self._wp_idx < len(self._waypoints):
                tx, ty = self._waypoints[self._wp_idx]
                dx = tx - self._current_x
                dy = ty - self._current_y
                msg.distance_to_waypoint = float(math.sqrt(dx * dx + dy * dy))
                bearing = math.atan2(dy, dx)
                err = self._normalize_angle(bearing - self._current_yaw)
                msg.heading_error = float(math.degrees(err))
        else:
            msg.current_waypoint_index = 0
            msg.total_waypoints = 0
            msg.distance_to_waypoint = 0.0
            msg.heading_error = 0.0

        self.status_pub.publish(msg)

    @staticmethod
    def _normalize_angle(a):
        """Wrap angle to [-pi, pi]."""
        return (a + math.pi) % (2.0 * math.pi) - math.pi


def main(args=None):
    rclpy.init(args=args)
    node = MissionManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node._publish_zero_cmd()
        except Exception:
            pass
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
