#!/usr/bin/env python3
import json
import threading
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import websocket


class MarkerReceiver(Node):
    def __init__(self):
        super().__init__('marker_receiver')
        self.pub = self.create_publisher(String, '/marker_data', 10)
        
        self.ws = websocket.WebSocketApp(
            "ws://localhost:9090",
            on_open=self.on_open,
            on_message=self.on_message,
            on_error=self.on_error,
            on_close=self.on_close
        )
        
        self.ws_thread = threading.Thread(target=self.ws.run_forever, daemon=True)
        self.ws_thread.start()
        self.get_logger().info("Подключаюсь к rosbridge...")

    def on_open(self, ws):
        self.get_logger().info("Подключено!")
        # Подписка на состояние аппарата
        ws.send(json.dumps({
            "op": "subscribe",
            "topic": "/stingray/topics/uv_state",
            "type": "stingray_interfaces/msg/UVState"
        }))
        # Подписка на Bbox с передней камеры
        ws.send(json.dumps({
            "op": "subscribe",
            "topic": "/stingray/topics/camera/front/bbox_array",
            "type": "stingray_interfaces/msg/BboxArray"
        }))
        self.get_logger().info("Подписки отправлены")

    def on_message(self, ws, message):
        data = json.loads(message)
        if data.get("op") == "publish":
            topic = data.get("topic")
            msg_data = data.get("msg", {})
            
            out = String()
            out.data = json.dumps({"topic": topic, "data": msg_data})
            self.pub.publish(out)
            
            # Короткий лог
            if "depth" in msg_data:
                self.get_logger().info(f"UVState: depth={msg_data.get('depth', '?')}")
            elif "bboxes" in msg_data:
                self.get_logger().info(f"BboxArray: {len(msg_data.get('bboxes', []))} объектов")

    def on_error(self, ws, error):
        self.get_logger().error(f"Ошибка: {error}")

    def on_close(self, ws, code, msg):
        self.get_logger().warn("Соединение закрыто")


def main():
    rclpy.init()
    node = MarkerReceiver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()