"""
ROS2 node serving the delivery robot web dashboard via aiohttp.

Provides:
  GET /          -- single-page web app
  GET /stream    -- MJPEG proxy from /camera/image_raw/compressed
  GET /snapshot  -- latest single JPEG frame
  WS  /ws        -- bidirectional bridge (cmd_vel in, status out)
"""

import asyncio
import json
import os
import threading

import rclpy
from aiohttp import web
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import CompressedImage

from delivery_robot_msgs.msg import MotorStatus

FAST_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1,
)

STATIC_DIR = os.path.join(os.path.dirname(__file__), 'static')


class WebUINode(Node):
    def __init__(self):
        super().__init__('web_ui')

        self.declare_parameter('port', 8080)
        self.port = self.get_parameter('port').value

        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        self._latest_frame = b''
        self._frame_lock = threading.Lock()
        self._motor_status = {'duty_cycles': [0, 0, 0, 0], 'active': [False] * 4, 'mode': 'manual'}
        self._status_lock = threading.Lock()
        self._ws_clients: list[web.WebSocketResponse] = []

        self.create_subscription(
            CompressedImage, 'camera/image_raw/compressed',
            self._image_cb, FAST_QOS
        )
        self.create_subscription(
            MotorStatus, 'motor_status',
            self._motor_status_cb, 10
        )

        self._aio_thread = threading.Thread(target=self._run_server, daemon=True)
        self._aio_thread.start()

        self.get_logger().info(f'Web UI at http://0.0.0.0:{self.port}')

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

    def _run_server(self):
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        app = web.Application()
        app.router.add_get('/', self._handle_index)
        app.router.add_get('/stream', self._handle_stream)
        app.router.add_get('/snapshot', self._handle_snapshot)
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
                elif msg_raw.type == web.WSMsgType.ERROR:
                    break
        finally:
            status_task.cancel()
            if ws in self._ws_clients:
                self._ws_clients.remove(ws)
            self.get_logger().info('WebSocket client disconnected')

        return ws

    async def _push_status(self, ws: web.WebSocketResponse):
        try:
            while not ws.closed:
                with self._status_lock:
                    status = dict(self._motor_status)
                status['connected'] = True
                await ws.send_json({'status': status})
                await asyncio.sleep(0.2)
        except (ConnectionResetError, asyncio.CancelledError):
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
