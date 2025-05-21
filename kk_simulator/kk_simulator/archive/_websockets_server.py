import asyncio
import json
import threading
import time
from collections import deque

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from kk_driver_msg.msg import PwmCmd, PwmStatus, IcsCmd

import websockets

class WebSockets2Node(Node):
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

        # ROS2 SubScriber登録
        #@自動生成ここから
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
        
        self.sub_list["pwm/cmd"] = self.create_subscription(
            PwmCmd, "pwm/cmd", subPwmCmd, 10
        )
        #@自動生成ここまで

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
                if (msg_type == "PwmCmd"):
                    msg = PwmCmd()
                    msg.child_id = data["field"]["child_id"]
                    msg.port = data["field"]["port"]
                    msg.pos = data["field"]["pos"]
                    msg.spd = data["field"]["spd"]
                    self.pub_list[topic].publish(msg);
                    return
                print("未実装型のMSGを受信")
        
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
        async with websockets.serve(self.ws_client_handler, "0.0.0.0", 8950):
            await self.ws_send_loop()            


node = WebSockets2Node()