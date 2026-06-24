"""
ROS2 node that captures frames from the Pi Camera Module 3 using rpicam-vid
and publishes them as sensor_msgs/Image and sensor_msgs/CompressedImage.

rpicam-vid is launched in a clean environment to avoid ROS2's LD_LIBRARY_PATH
shadowing the PPA's PiSP-enabled libcamera on Pi 5 + Ubuntu 24.04.
"""

import os
import subprocess
import threading

import yaml
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import CameraInfo, CompressedImage, Image


FAST_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1,
)


class CameraNode(Node):
    def __init__(self):
        super().__init__('pi_camera')

        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)
        self.declare_parameter('framerate', 30)
        self.declare_parameter('quality', 50)
        self.declare_parameter('publish_raw', False)
        self.declare_parameter('calibration_file', '')

        self.width = self.get_parameter('width').value
        self.height = self.get_parameter('height').value
        self.framerate = self.get_parameter('framerate').value
        self.quality = self.get_parameter('quality').value
        self.publish_raw = self.get_parameter('publish_raw').value
        calibration_file = self.get_parameter('calibration_file').value

        self.image_pub = self.create_publisher(Image, 'camera/image_raw', FAST_QOS)
        self.compressed_pub = self.create_publisher(
            CompressedImage, 'camera/image_raw/compressed', FAST_QOS
        )
        self.info_pub = self.create_publisher(CameraInfo, 'camera/camera_info', FAST_QOS)

        self._camera_info_msg = self._load_calibration(calibration_file)
        self._bridge = None
        self._process = None
        self._running = False

        self.get_logger().info(
            f'Starting camera capture: {self.width}x{self.height}@{self.framerate}fps q={self.quality}'
        )
        self._start_capture()

    def _build_clean_env(self):
        """Build a minimal environment so rpicam-vid picks up the system
        PiSP-enabled libcamera instead of the ROS2 one."""
        env = {
            'PATH': '/usr/bin:/usr/sbin:/bin:/sbin',
            'HOME': os.environ.get('HOME', '/home/pi'),
            'USER': os.environ.get('USER', 'pi'),
            'LANG': os.environ.get('LANG', 'C.UTF-8'),
        }
        xdg = os.environ.get('XDG_RUNTIME_DIR')
        if xdg:
            env['XDG_RUNTIME_DIR'] = xdg
        return env

    def _load_calibration(self, calibration_file):
        """Load camera calibration YAML and build a CameraInfo message."""
        msg = CameraInfo()
        msg.header.frame_id = 'camera_link'
        msg.width = self.width
        msg.height = self.height

        if not calibration_file or not os.path.isfile(calibration_file):
            if calibration_file:
                self.get_logger().warn(
                    f'Calibration file not found: {calibration_file}, using defaults'
                )
            else:
                self.get_logger().info('No calibration file specified, CameraInfo will use zeros')
            return msg

        try:
            with open(calibration_file, 'r') as f:
                cal = yaml.safe_load(f)

            msg.width = cal.get('image_width', self.width)
            msg.height = cal.get('image_height', self.height)
            msg.distortion_model = cal.get('distortion_model', 'plumb_bob')

            cm = cal.get('camera_matrix', {})
            if 'data' in cm:
                msg.k = [float(v) for v in cm['data']]

            dc = cal.get('distortion_coefficients', {})
            if 'data' in dc:
                msg.d = [float(v) for v in dc['data']]

            rm = cal.get('rectification_matrix', {})
            if 'data' in rm:
                msg.r = [float(v) for v in rm['data']]

            pm = cal.get('projection_matrix', {})
            if 'data' in pm:
                msg.p = [float(v) for v in pm['data']]

            self.get_logger().info(f'Loaded calibration from: {calibration_file}')
        except Exception as e:
            self.get_logger().error(f'Failed to load calibration: {e}')

        return msg

    def _start_capture(self):
        cmd = [
            'rpicam-vid',
            '-t', '0',
            '--width', str(self.width),
            '--height', str(self.height),
            '--framerate', str(self.framerate),
            '--codec', 'mjpeg',
            '--quality', str(self.quality),
            '--nopreview',
            '--flush',
            '-o', '-',
        ]

        try:
            self._process = subprocess.Popen(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                env=self._build_clean_env(),
                bufsize=0,
            )
        except FileNotFoundError:
            self.get_logger().error(
                'rpicam-vid not found. Install: '
                'sudo add-apt-repository ppa:manajev/pi5-camera && '
                'sudo apt install rpicam-apps-rpi'
            )
            return

        self._running = True
        threading.Thread(target=self._read_mjpeg_stream, daemon=True).start()
        threading.Thread(target=self._log_stderr, daemon=True).start()

    def _log_stderr(self):
        while self._running and self._process and self._process.stderr:
            line = self._process.stderr.readline()
            if not line:
                break
            text = line.decode('utf-8', errors='replace').strip()
            if text:
                self.get_logger().debug(f'rpicam-vid: {text}')

    def _read_mjpeg_stream(self):
        buf = b''
        stdout = self._process.stdout

        while self._running:
            chunk = stdout.read(4096)
            if not chunk:
                if self._running:
                    self.get_logger().warn('rpicam-vid stream ended unexpectedly')
                break

            buf += chunk

            while True:
                soi = buf.find(b'\xff\xd8')
                if soi == -1:
                    buf = b''
                    break
                eoi = buf.find(b'\xff\xd9', soi + 2)
                if eoi == -1:
                    buf = buf[soi:]
                    break
                jpeg_data = buf[soi:eoi + 2]
                buf = buf[eoi + 2:]
                self._publish_frame(jpeg_data)

    def _publish_frame(self, jpeg_data: bytes):
        now = self.get_clock().now().to_msg()

        # Publish CameraInfo in sync with images
        info_msg = self._camera_info_msg
        info_msg.header.stamp = now
        self.info_pub.publish(info_msg)

        comp_msg = CompressedImage()
        comp_msg.header.stamp = now
        comp_msg.header.frame_id = 'camera_link'
        comp_msg.format = 'jpeg'
        comp_msg.data = jpeg_data
        self.compressed_pub.publish(comp_msg)

        if self.publish_raw:
            if self._bridge is None:
                import cv2
                import numpy as np
                from cv_bridge import CvBridge
                self._bridge = CvBridge()
                self._cv2 = cv2
                self._np = np
            np_arr = self._np.frombuffer(jpeg_data, self._np.uint8)
            frame = self._cv2.imdecode(np_arr, self._cv2.IMREAD_COLOR)
            if frame is not None:
                img_msg = self._bridge.cv2_to_imgmsg(frame, encoding='bgr8')
                img_msg.header.stamp = now
                img_msg.header.frame_id = 'camera_link'
                self.image_pub.publish(img_msg)

    def destroy_node(self):
        self._running = False
        if self._process:
            self._process.terminate()
            try:
                self._process.wait(timeout=3)
            except subprocess.TimeoutExpired:
                self._process.kill()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
