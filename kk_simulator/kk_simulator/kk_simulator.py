import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from kk_driver_msg.msg import PwmCmd  # カスタムメッセージ
import asyncio
import json

from fastapi import FastAPI
from fastapi.responses import JSONResponse
from fastapi.responses import HTMLResponse
from datetime import datetime

import threading
import uvicorn

app = FastAPI()

# シミュレーションデータ（仮想データ）
servo_state = {
    "boards": [
        {"id": "0", "pos": [90, 90, 90, 90, 90, 90, 90, 90]},
        {"id": "1", "pos": [90, 90, 90, 90, 90, 90, 90, 90]},
        {"id": "2", "pos": [90, 90, 90, 90, 90, 90, 90, 90]},
        {"id": "3", "pos": [90, 90, 90, 90, 90, 90, 90, 90]},
    ]
}
servo_tool = """
<html lang="jp"><head><meta charset="UTF-8"><meta name="viewport" content="width=device-width, initial-scale=1.0"><title>Servo Monitor</title><style>body {font-family: Arial, sans-serif;margin: 0;padding: 0;background-color: #f0f0f0;}header {background-color: #333;color: #fff;padding: 10px 20px;text-align: center;}.board {margin: 20px auto;padding: 0;background: #eee;border-radius: 8px;overflow: hidden;box-shadow: 0 4px 8px rgba(0, 0, 0, 0.1);max-width: 600px;}.board-header {display: block;width: 100%;padding: 10px;box-sizing: border-box;background-color: #777;color: #f0f0f0;align-items: center;margin-bottom: 10px;}.servo-container {display: grid;padding: 10px;grid-template-columns: repeat(4, 1fr);gap: 10px;}.servo {width: 100px;height: 100px;position: relative;margin: 0 auto;}.servo .dial {width: 100%;height: 100%;box-sizing: border-box;border: 2px solid #333;border-radius: 50%;position: relative;background: #334;}.servo .needle {width: 5px;height: 50%;background: #68f;position: absolute;top: 50%;left: 50%;transform-origin: bottom center;transform: translateY(-100%) rotate(0deg);transition: .2s;}.servo .label {position: absolute;display: block;top: 25%;left: 0;width: 100%;text-align: center;margin-top: 8px;font-size: 15px;color: #fffa;}</style></head><body><header><h1>サーボ シミュレータ</h1></header><div id="boards"><div class="board"><div class="board-header"><span><strong>Board ID:</strong>0</span></div><div class="servo-container"><div class="servo" id="s00"><div class="dial"><div class="needle" style="transform: translateY(-100%) rotate(-90deg);"></div></div><div class="label">Servo 0<br><span class="deg">180</span>°</div></div><div class="servo" id="s01"><div class="dial"><div class="needle" style="transform: translateY(-100%) rotate(0deg);"></div></div><div class="label">Servo 1<br><span class="deg">180</span>°</div></div><div class="servo" id="s02"><div class="dial"><div class="needle" style="transform: translateY(-100%) rotate(-30deg);"></div></div><div class="label">Servo 2<br><span class="deg">180</span>°</div></div><div class="servo" id="s03"><div class="dial"><div class="needle" style="transform: translateY(-100%) rotate(90deg);"></div></div><div class="label">Servo 3<br><span class="deg">180</span>°</div></div><div class="servo" id="s04"><div class="dial"><div class="needle" style="transform: translateY(-100%) rotate(30deg);"></div></div><div class="label">Servo 4<br><span class="deg">180</span>°</div></div><div class="servo" id="s05"><div class="dial"><div class="needle" style="transform: translateY(-100%) rotate(40deg);"></div></div><div class="label">Servo 5<br><span class="deg">180</span>°</div></div><div class="servo" id="s06"><div class="dial"><div class="needle" style="transform: translateY(-100%) rotate(-60deg);"></div></div><div class="label">Servo 6<br><span class="deg">180</span>°</div></div><div class="servo" id="s07"><div class="dial"><div class="needle" style="transform: translateY(-100%) rotate(-50deg);"></div></div><div class="label">Servo 7<br><span class="deg">180</span>°</div></div></div></div><div class="board"><div class="board-header"><span><strong>Board ID:</strong>0</span></div><div class="servo-container"><div class="servo" id="s10"><div class="dial"><div class="needle" style="transform: translateY(-100%) rotate(-90deg);"></div></div><div class="label">Servo 0<br><span class="deg">180</span>°</div></div><div class="servo" id="s11"><div class="dial"><div class="needle" style="transform: translateY(-100%) rotate(0deg);"></div></div><div class="label">Servo 1<br><span class="deg">180</span>°</div></div><div class="servo" id="s12"><div class="dial"><div class="needle" style="transform: translateY(-100%) rotate(-30deg);"></div></div><div class="label">Servo 2<br><span class="deg">180</span>°</div></div><div class="servo" id="s13"><div class="dial"><div class="needle" style="transform: translateY(-100%) rotate(90deg);"></div></div><div class="label">Servo 3<br><span class="deg">180</span>°</div></div><div class="servo" id="s14"><div class="dial"><div class="needle" style="transform: translateY(-100%) rotate(30deg);"></div></div><div class="label">Servo 4<br><span class="deg">180</span>°</div></div><div class="servo" id="s15"><div class="dial"><div class="needle" style="transform: translateY(-100%) rotate(40deg);"></div></div><div class="label">Servo 5<br><span class="deg">180</span>°</div></div><div class="servo" id="s16"><div class="dial"><div class="needle" style="transform: translateY(-100%) rotate(-60deg);"></div></div><div class="label">Servo 6<br><span class="deg">180</span>°</div></div><div class="servo" id="s17"><div class="dial"><div class="needle" style="transform: translateY(-100%) rotate(-50deg);"></div></div><div class="label">Servo 7<br><span class="deg">180</span>°</div></div></div></div><div class="board"><div class="board-header"><span><strong>Board ID:</strong>0</span></div><div class="servo-container"><div class="servo" id="s20"><div class="dial"><div class="needle" style="transform: translateY(-100%) rotate(-90deg);"></div></div><div class="label">Servo 0<br><span class="deg">180</span>°</div></div><div class="servo" id="s21"><div class="dial"><div class="needle" style="transform: translateY(-100%) rotate(0deg);"></div></div><div class="label">Servo 1<br><span class="deg">180</span>°</div></div><div class="servo" id="s22"><div class="dial"><div class="needle" style="transform: translateY(-100%) rotate(-30deg);"></div></div><div class="label">Servo 2<br><span class="deg">180</span>°</div></div><div class="servo" id="s23"><div class="dial"><div class="needle" style="transform: translateY(-100%) rotate(90deg);"></div></div><div class="label">Servo 3<br><span class="deg">180</span>°</div></div><div class="servo" id="s24"><div class="dial"><div class="needle" style="transform: translateY(-100%) rotate(30deg);"></div></div><div class="label">Servo 4<br><span class="deg">180</span>°</div></div><div class="servo" id="s25"><div class="dial"><div class="needle" style="transform: translateY(-100%) rotate(40deg);"></div></div><div class="label">Servo 5<br><span class="deg">180</span>°</div></div><div class="servo" id="s26"><div class="dial"><div class="needle" style="transform: translateY(-100%) rotate(-60deg);"></div></div><div class="label">Servo 6<br><span class="deg">180</span>°</div></div><div class="servo" id="s27"><div class="dial"><div class="needle" style="transform: translateY(-100%) rotate(-50deg);"></div></div><div class="label">Servo 7<br><span class="deg">180</span>°</div></div></div></div><div class="board"><div class="board-header"><span><strong>Board ID:</strong>0</span></div><div class="servo-container"><div class="servo" id="s30"><div class="dial"><div class="needle" style="transform: translateY(-100%) rotate(-90deg);"></div></div><div class="label">Servo 0<br><span class="deg">180</span>°</div></div><div class="servo" id="s31"><div class="dial"><div class="needle" style="transform: translateY(-100%) rotate(0deg);"></div></div><div class="label">Servo 1<br><span class="deg">180</span>°</div></div><div class="servo" id="s32"><div class="dial"><div class="needle" style="transform: translateY(-100%) rotate(-30deg);"></div></div><div class="label">Servo 2<br><span class="deg">180</span>°</div></div><div class="servo" id="s33"><div class="dial"><div class="needle" style="transform: translateY(-100%) rotate(90deg);"></div></div><div class="label">Servo 3<br><span class="deg">180</span>°</div></div><div class="servo" id="s34"><div class="dial"><div class="needle" style="transform: translateY(-100%) rotate(30deg);"></div></div><div class="label">Servo 4<br><span class="deg">180</span>°</div></div><div class="servo" id="s35"><div class="dial"><div class="needle" style="transform: translateY(-100%) rotate(40deg);"></div></div><div class="label">Servo 5<br><span class="deg">180</span>°</div></div><div class="servo" id="s36"><div class="dial"><div class="needle" style="transform: translateY(-100%) rotate(-60deg);"></div></div><div class="label">Servo 6<br><span class="deg">180</span>°</div></div><div class="servo" id="s37"><div class="dial"><div class="needle" style="transform: translateY(-100%) rotate(-50deg);"></div></div><div class="label">Servo 7<br><span class="deg">180</span>°</div></div></div></div></div><script>data = {"boards": [{"id": "0","pos": [0,90,60,180,120,130,30,40]},{"id": "1","pos": [0,90,60,180,120,130,30,40]},{"id": "2","pos": [0,90,60,180,120,130,30,40]},{"id": "3","pos": [0,90,60,180,120,130,30,40]}]};async function fetchStatus() {const raw_data = await fetch("/servo/state");const data = await raw_data.json();displayBoards(data.boards);}function displayBoards(data) {document.querySelectorAll(".board").forEach((board,board_index)=>{board.querySelectorAll(".servo").forEach((servo, servo_index) =>{/*data[board_index].pos[servo_index] = data[board_index].pos[servo_index] + 1;*/const needle = servo.querySelector(".needle");const deg = servo.querySelector(".deg");const deg_now = data[board_index].pos[servo_index];needle.style = "transform: translateY(-100%) rotate(-"+deg_now+"deg);";deg.innerHTML = Math.round(deg_now);});});}fetchStatus();setInterval(()=>{fetchStatus();console.log("updatye")},100);</script></body></html>
"""

@app.get("/")
def index():
  return HTMLResponse(content=servo_tool)

@app.get("/servo/state")
def get_servo_state():
    """
    サーボモーターの状態をJSONで返すエンドポイント
    """
    return JSONResponse(content=servo_state)

class ServoSimulator(Node):
    def __init__(self):
        super().__init__('servo_simulator')
        self.subscription = self.create_subscription(
            PwmCmd,
            'pwm_cmd',
            self.cmd_callback,
            10
        )
        self.state = {child_id: {"ports": [{"angle": 0, "speed": 0} for _ in range(8)], "last_message": None} for child_id in range(4)}
        self.speed_table = [i for i in range(201)]  # 0°/sec ～ 200°/sec

    def cmd_callback(self, msg):
        # Update state based on the received message
        child_id = msg.child_id
        for port, pos, spd in zip(msg.port, msg.pos, msg.spd):
            if (spd == 0):
                continue;
            if (pos < 1000):
                continue;
            if (pos > 2000):
                continue;
            angle = (pos - 1000) / 1000 * 180
            servo_state["boards"][child_id]["pos"][port] = angle;
def start_fastapi():
    uvicorn.run(app, host="0.0.0.0", port=8000)

def main(args=None):
    rclpy.init(args=args)
    simulator = ServoSimulator()

    fastapi_thread = threading.Thread(target=start_fastapi, daemon=True)
    fastapi_thread.start()

    try:
        rclpy.spin(simulator)
    finally:
        simulator.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()