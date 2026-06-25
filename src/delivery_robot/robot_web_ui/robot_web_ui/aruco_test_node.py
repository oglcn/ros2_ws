#!/usr/bin/env python3
"""
Minimal ArUco detection test / approach control page.

Just the camera stream, a "target marker ID" box, a live distance
read-out, and a Start/Stop button. No joystick, no manual driving.
Start/Stop only has an effect if aruco_approach_node is also running
(it arms/disarms that node via a topic) -- in plain detection-test mode
there's simply nothing listening, which is harmless.

Provides:
  GET /       -- single-page web app
  GET /stream -- MJPEG proxy from /camera/image_raw/compressed
  WS  /ws     -- pushes ArUco detections + approach status to the browser,
                 accepts {"armed": true/false} to start/stop the approach
"""

import asyncio
import json
import os
import threading

import rclpy
from aiohttp import web
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Bool

from delivery_robot_msgs.msg import ArucoDetections, ApproachStatus

FAST_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1,
)

STATIC_DIR = os.path.join(os.path.dirname(__file__), 'static')


class ArucoTestNode(Node):
    def __init__(self):
        super().__init__('aruco_test_web')

        self.declare_parameter('port', 8081)
        self.port = self.get_parameter('port').value

        self._latest_frame = b''
        self._frame_lock = threading.Lock()

        self._detections = []
        self._det_lock = threading.Lock()

        self._ws_clients: list[web.WebSocketResponse] = []

        self.armed_pub = self.create_publisher(Bool, 'aruco_approach/armed', 10)

        self.create_subscription(
            CompressedImage, 'camera/image_raw/compressed',
            self._image_cb, FAST_QOS
        )
        self.create_subscription(
            ArucoDetections, 'aruco/detections',
            self._detections_cb, 10
        )
        self.create_subscription(
            ApproachStatus, 'aruco_approach/status',
            self._approach_status_cb, 10
        )

        self._aio_thread = threading.Thread(target=self._run_server, daemon=True)
        self._aio_thread.start()

        self.get_logger().info(f'ArUco test page at http://0.0.0.0:{self.port}')

    def _image_cb(self, msg: CompressedImage):
        with self._frame_lock:
            self._latest_frame = bytes(msg.data)

    def _detections_cb(self, msg: ArucoDetections):
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

        with self._det_lock:
            self._detections = detections

        self._push_to_clients({'detections': detections})

    def _approach_status_cb(self, msg: ApproachStatus):
        self._push_to_clients({'approach': {
            'target_id': msg.target_marker_id,
            'armed': msg.armed,
            'visible': msg.marker_visible,
            'size_norm': msg.size_norm,
            'state': msg.state,
        }})

    def _set_armed(self, armed: bool):
        self.armed_pub.publish(Bool(data=bool(armed)))

    def _push_to_clients(self, payload):
        for ws in list(self._ws_clients):
            asyncio.run_coroutine_threadsafe(self._safe_send(ws, payload), self._loop)

    async def _safe_send(self, ws, payload):
        try:
            await ws.send_json(payload)
        except Exception:
            pass

    async def _handle_index(self, request):
        return web.FileResponse(os.path.join(STATIC_DIR, 'aruco_test.html'))

    async def _handle_stream(self, request):
        response = web.StreamResponse(
            status=200,
            headers={'Content-Type': 'multipart/x-mixed-replace; boundary=frame'},
        )
        await response.prepare(request)
        try:
            while True:
                with self._frame_lock:
                    frame = self._latest_frame
                if frame:
                    await response.write(
                        b'--frame\r\nContent-Type: image/jpeg\r\n'
                        + f'Content-Length: {len(frame)}\r\n\r\n'.encode()
                        + frame + b'\r\n'
                    )
                await asyncio.sleep(0.05)
        except (ConnectionResetError, asyncio.CancelledError):
            pass
        return response

    async def _handle_ws(self, request):
        ws = web.WebSocketResponse()
        await ws.prepare(request)
        self._ws_clients.append(ws)
        try:
            async for msg in ws:
                if msg.type == web.WSMsgType.TEXT:
                    try:
                        data = json.loads(msg.data)
                    except ValueError:
                        continue
                    if 'armed' in data:
                        self._set_armed(bool(data['armed']))
        finally:
            if ws in self._ws_clients:
                self._ws_clients.remove(ws)
        return ws

    def _run_server(self):
        self._loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self._loop)

        app = web.Application()
        app.router.add_get('/', self._handle_index)
        app.router.add_get('/stream', self._handle_stream)
        app.router.add_get('/ws', self._handle_ws)

        runner = web.AppRunner(app)
        self._loop.run_until_complete(runner.setup())
        site = web.TCPSite(runner, '0.0.0.0', self.port)
        self._loop.run_until_complete(site.start())
        self._loop.run_forever()


def main(args=None):
    rclpy.init(args=args)
    node = ArucoTestNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
