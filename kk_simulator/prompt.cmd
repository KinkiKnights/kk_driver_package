あなたはROS2のメッセージ構造とWebSocket通信を統合するPython開発アシスタントです。
以下の情報を元に、ROS2メッセージとJSONの双方向変換処理を自動生成してください。

【変換要件】
1. メッセージ型とトピックの対応関係:
  - publisher側
   - Gm6020Status: gm6020/status
  - subscriber側
   - Gm6020Cmd: gm6020/cmd

2. 各メッセージ型の構造:
   - PwmCmd:
    uint8 child_id
    uint8[] port
    uint16[] pos
    uint8[] spd
   - PwmStatus:
    uint8 child_id
    bool is_power
    uint16[] pos
   - Gm6020Cmd:
    uint8[] motor_id
    float32[] duty 
   - Gm6020Status:
    int32[7] position
    int16[7] speed
    int16[7] torque

3. 生成してほしい処理:
   「#@自動生成ここから」と「#@自動生成ここまで」で囲まれたコード内の処理
　　既存の処理を参考に1,2で定義された内容を変換する


【出力形式】
自動生成した個所をベースコードに統合した最終的なコードを提示してください。
また、そのコードが要件に合致しているかを確認してその結果を報告してください。

【注意事項】
- 指定された範囲外のプログラム構造は完全に維持する
- import文の追加は不要
- 既存の非同期処理構造を保持
- メッセージフィールド名は厳密に一致させる
- 型チェック失敗時は処理を中断して警告表示

【ベースコード】
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
        self.msg_types = {
            1: PwmCmd,
            2: IcsCmd,
            3: String
        }

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