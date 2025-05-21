import rclpy
from rclpy.node import Node
import threading
import time
import json
from http.server import HTTPServer, SimpleHTTPRequestHandler
from websocket_server import WebSocketServer
from ament_index_python.packages import get_package_share_directory
import os

class WebSocketServerWrapper:
    def __init__(self, host, port):
        self.host = host
        self.port = port
        self.server = WebSocketServer(port, host=host)
        self.server.set_fn_new_client(self.on_new_client)
        self.server.set_fn_client_left(self.on_client_left)
        self.clients = []
        self.lock = threading.Lock()

    def on_new_client(self, client, server):
        with self.lock:
            self.clients.append(client)
        print(f"New client connected: {client['id']}")

    def on_client_left(self, client, server):
        with self.lock:
            try:
                self.clients.remove(client)
            except ValueError:
                pass
        print(f"Client disconnected: {client['id']}")

    def broadcast(self, message):
        with self.lock:
            clients = self.clients.copy()
        for client in clients:
            try:
                self.server.send_message(client, message)
            except Exception as e:
                print(f"Error sending to client {client['id']}: {e}")

    def start(self):
        print(f"Starting WebSocket server on {self.host}:{self.port}")
        self.server.run_forever()

class StaticHTTPRequestHandler(SimpleHTTPRequestHandler):
    def __init__(self, *args, **kwargs):
        static_dir = os.path.join(
            get_package_share_directory('your_package'),
            'static'
        )
        super().__init__(*args, directory=static_dir, **kwargs)

class StaticHTTPServerThread(threading.Thread):
    def __init__(self, host, port):
        super().__init__()
        self.host = host
        self.port = port
        self.server = HTTPServer((self.host, self.port), StaticHTTPRequestHandler)

    def run(self):
        print(f"HTTP server starting on {self.host}:{self.port}")
        self.server.serve_forever()

class SampleNode(Node):
    def __init__(self, ws_server):
        super().__init__('sample_node')
        self.ws_server = ws_server
        self.timer = self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        data = {
            'timestamp': time.time(),
            'message': 'Hello from ROS2 node!'
        }
        json_data = json.dumps(data)
        self.ws_server.broadcast(json_data)
        self.get_logger().info('Broadcasted data via WebSocket')

def main():
    # WebSocketサーバーの起動
    ws_server = WebSocketServerWrapper('0.0.0.0', 8081)
    ws_thread = threading.Thread(target=ws_server.start)
    ws_thread.daemon = True
    ws_thread.start()

    # HTTPサーバーの起動
    http_server = StaticHTTPServerThread('0.0.0.0', 8080)
    http_server.daemon = True
    http_server.start()

    # ROS2ノードの初期化
    rclpy.init()
    node = SampleNode(ws_server)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()