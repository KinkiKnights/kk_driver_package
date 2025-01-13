# マルチキャスト関連
import socket
import time
import threading
import base64
import random as r

import asyncio
import websockets

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from kk_driver_msg.msg import KeyCtrl

"""
IPアドレスマルチキャスト
"""
MULTICAST_GROUP = '226.0.0.1'
PORT = 5004
connect_interface = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
connect_interface.connect(("8.8.8.8", 80))
IP_ADDR = connect_interface.getsockname()[0]
connect_interface.close()
def multicast_sender():
    # UDPソケットを作成
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
    # マルチキャストTTLを設定
    sock.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_TTL, 32)
    while True:
        print(f"Sending: {IP_ADDR}")
        # メッセージを送信
        sock.sendto(IP_ADDR.encode('utf-8'), (MULTICAST_GROUP, PORT))
        # 10秒待機
        time.sleep(10)
ip_cast = threading.Thread(target=multicast_sender)

"""
キーボード取得
"""
KEY_COUNT = 256
class KeyboardPublisher(Node):
    def __init__(self):
        super().__init__('kk_receiver')
        self.publisher_ = self.create_publisher(KeyCtrl, 'keys', 1)
        self.timer = self.create_timer(0.1, self.publish_key_states)
        self.key_states = [False] * KEY_COUNT
        self.mouseX = 0
        self.mouseY = 0
    # メッセージのPublish
    def publish_key_states(self):
        msg = KeyCtrl()
        msg.x = self.mouseX;
        msg.y = self.mouseY;
        for i in range(KEY_COUNT):
            msg.keys[i] = self.key_states[i];
        self.publisher_.publish(msg)
    # マウス座標の格納
    def update_mouse_states(self, x, y):
        self.mouseX = int(x)
        self.mouseY = int(y)
    def update_key_states(self, binary_data):
        bitfield = int.from_bytes(binary_data, byteorder='big')
        self.key_states = [char == '1' for char in input_string]
        # for i in range(KEY_COUNT):
        #     self.key_states[i] = bool(bitfield & (1 << i))
    async def websocket_server(self, host="0.0.0.0", port=8765):
        async def handler(websocket):
            while True:
                # try:
                rcv_msg = await websocket.recv()
                print(rcv_msg)
                rcv_msgs = rcv_msg.split(':')
                self.update_mouse_states(rcv_msgs[0], rcv_msgs[1])
                self.update_key_states(rcv_msgs[2])
                # except Exception as e:
                #     self.get_logger().error(f"Error receiving or processing message: {e}")
        server = await websockets.serve(handler, host, port)
        self.get_logger().info(f"WebSocket server started on {host}:{port}")
        await server.wait_closed()

async def main_async():
    rclpy.init()
    publisher = KeyboardPublisher()

    try:
        await publisher.websocket_server()
    except KeyboardInterrupt:
        pass
    finally:
        publisher.destroy_node()
        rclpy.shutdown()

def main():
    ip_cast.start()
    asyncio.run(main_async())

if __name__ == '__main__':
    main()

