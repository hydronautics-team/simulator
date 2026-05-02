#!/usr/bin/env python3
import json
import time
import threading
import cv2
import numpy as np
from http.server import HTTPServer, BaseHTTPRequestHandler
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import String

latest_frame = None
latest_frame_ts = 0.0
latest_data = {}
latest_data_ts = 0.0
latest_marker = {}
frame_lock = threading.Lock()
data_lock = threading.Lock()

class WsBridge(Node):
    def __init__(self):
        super().__init__('ws_bridge')
        self.bridge = CvBridge()
        self.create_subscription(Image, '/stingray/topics/processed_image', self.image_callback, 10)
        self.create_subscription(String, '/stingray/topics/marker_debug', self.marker_callback, 10)
        from stingray_interfaces.msg import UVState
        self.create_subscription(UVState, '/stingray/topics/uv_state', self.uvstate_callback, 10)
        self.get_logger().info("HTTP Bridge запущен (MJPEG stream)")

    def marker_callback(self, msg):
        global latest_marker
        with data_lock:
            latest_marker = json.loads(msg.data)

    def uvstate_callback(self, msg):
        global latest_data, latest_data_ts
        with data_lock:
            latest_data = {
                'depth': round(msg.depth, 3),
                'yaw': round(msg.yaw, 2),
                'roll': round(msg.roll, 2),
                'pitch': round(msg.pitch, 2),
                'surge_accel': round(msg.surge_accel, 3),
                'sway_accel': round(msg.sway_accel, 3),
            }
            latest_data_ts = time.time()

    def image_callback(self, msg):
        global latest_frame, latest_frame_ts
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            _, jpeg = cv2.imencode('.jpg', cv_image, [cv2.IMWRITE_JPEG_QUALITY, 70])
            with frame_lock:
                latest_frame = jpeg.tobytes()
                latest_frame_ts = time.time()
        except Exception:
            pass

class DataHandler(BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path == '/stream':
            self.send_response(200)
            self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=frame')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.end_headers()
            while True:
                with frame_lock:
                    frame = latest_frame
                if frame:
                    self.wfile.write(b'--frame\r\n')
                    self.wfile.write(b'Content-Type: image/jpeg\r\n\r\n')
                    self.wfile.write(frame)
                    self.wfile.write(b'\r\n')
                time.sleep(1/30)
        elif self.path == '/data':
            with data_lock:
                packet = {'ts': latest_data_ts, 'marker': latest_marker, **latest_data}
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.end_headers()
            self.wfile.write(json.dumps(packet).encode())
        elif self.path == '/video':
            with frame_lock:
                frame = latest_frame
            if frame:
                self.send_response(200)
                self.send_header('Content-Type', 'image/jpeg')
                self.send_header('Access-Control-Allow-Origin', '*')
                self.end_headers()
                self.wfile.write(frame)
            else:
                self.send_response(204)
                self.end_headers()
        else:
            self.send_response(404)
            self.end_headers()

def http_server():
    server = HTTPServer(('0.0.0.0', 9090), DataHandler)
    server.serve_forever()

def main():
    rclpy.init()
    bridge = WsBridge()
    t = threading.Thread(target=http_server, daemon=True)
    t.start()
    rclpy.spin(bridge)

if __name__ == '__main__':
    main()
