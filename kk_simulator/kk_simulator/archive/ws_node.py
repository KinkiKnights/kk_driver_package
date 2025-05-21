import asyncio
import json
import threading
import time
from collections import deque

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from kk_driver_msg.msg import PwmCmd, PwmStatus, Gm6020Cmd, Gm6020Status

import websockets

class WsNode(Node):
    __slots__ = [
        'clients',      # 追加属性1
        'send_que',     # 追加属性2
        'rcv_que',      # 追加属性3
        'sub_list',     # 追加属性4
        'pub_list'      # 追加属性5
    ]

    def __init__(self):
        # ROS2関連初期化
        rclpy.init()
        super().__init__('websockets_2_ros')
        self.sub_list = {}
        self.pub_list = {}

        # WebScoketsサーバ初期化
        self.clients = set()
        self.send_que = deque()
        self.rcv_que = deque()

        # ROS2 Publisher登録
        self.pub_list["pwm/status"] =  self.create_publisher(
            PwmStatus,
            "pwm/status",
            10
        )
        # ROS2 Publisher登録
        self.pub_list["gm6020/status"] =  self.create_publisher(
            GM6020Status,
            "gm6020/status",
            10
        )

        # ROS2 SubScriber登録

        def subPwmCmd(msg):
            try:
                msg_data = {
                    "topic": "pwm/cmd",
                    "type": "PwmCmd",
                    "field": {
                        "child_id": msg.child_id,
                        "port": msg.port,
                        "pos": msg.pos,
                        "spd": msg.spd
                    }
                }
                self.send_que.append(msg_data)
            except Exception as e:
                print(f"PwmCmd subscriber error: {e}")

        self.sub_list["pwm/cmd"] = self.create_subscription(
            PwmCmd, "pwm/cmd", subPwmCmd, 10
        )

        def subGm6020Status(msg):
            try:
                msg_data = {
                    "topic": "gm6020/status",
                    "type": "Gm6020Status",
                    "field": {
                        "position": msg.position,
                        "speed": msg.speed,
                        "torque": msg.torque
                    }
                }
                self.send_que.append(msg_data)
            except Exception as e:
                print(f"Gm6020Status subscriber error: {e}")

        def subGm6020Cmd(msg):
            try:
                msg_data = {
                    "topic": "gm6020/cmd",
                    "type": "Gm6020Cmd",
                    "field": {
                        "motor_id": msg.motor_id,
                        "duty": msg.duty,
                    }
                }
                self.send_que.append(msg_data)
            except Exception as e:
                print(f"Gm6020Status subscriber error: {e}")

        from kk_driver_msg.msg import Gm6020Status, Gm6020Cmd
        self.sub_list["gm6020/cmd"] = self.create_subscription(
            Gm6020Cmd, "gm6020/cmd", subGm6020Cmd, 10
        )

        # 起動処理
        thread = threading.Thread(target=self.spin)
        thread.start()
        asyncio.run(self.run())

    def spin(self):
        rclpy.spin(self)

    def convertTopic(self, data):
        topic = data["topic"]
        msg_type = data["type"]
        #@自動生成ここから
        if topic in self.pub_list:
            try:
                if msg_type == "PwmCmd":
                    msg = PwmCmd()
                    msg.child_id = data["field"]["child_id"]
                    msg.port = data["field"]["port"]
                    msg.pos = data["field"]["pos"]
                    msg.spd = data["field"]["spd"]
                    self.pub_list[topic].publish(msg)
                    return

                elif msg_type == "Gm6020Cmd":
                    msg = Gm6020Cmd()
                    msg.motor_id = data["field"]["motor_id"]
                    msg.duty = data["field"]["duty"]
                    self.pub_list[topic].publish(msg)
                    return

                print("未実装型のMSGを受信")
            except Exception as e:
                print(f"convertTopic error: {e}")
        #@自動生成ここまで
        else:
            print("未登録のMSGを受信")
    
    async def ws_client_handler(self, websocket, path):
        # クライアントリストの追加
        self.clients.add(websocket)
        try:
            async for message in websocket:
                try:
                    # 受信データをJSONでパースして受信キューに追加
                    data = json.loads(message)
                    self.rcv_que.append(data)
                except json.JSONDecodeError:
                    print(f"Invalid JSON received: {message}")
        finally:
            # リストからクライアント削除
            self.clients.remove(websocket)

    async def ws_send_loop(self):
        while True:
            await asyncio.sleep(0.01)
            # 送信処理
            if self.send_que:
                messages = list(self.send_que)
                self.send_que.clear()

                if self.clients:
                    message_str = json.dumps(messages)
                    await asyncio.gather(
                        *[client.send(message_str) for client in self.clients],
                        return_exceptions=True
                    )
            # 受信処理
            messages = list(self.rcv_que)
            self.rcv_que.clear()
            for data in messages:
                self.convertTopic(data)


    async def run(self):
        async with websockets.serve(self.ws_client_handler, "0.0.0.0", 8001):
            print("Starting server at http://0.0.0.0:8001/")
            await self.ws_send_loop()            


if __name__ == "__main__":
    node = WsNode()