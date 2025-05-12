# 導入
ワークスペースの`src`ディレクトリにリポジトリをcloneします。

# 起動
`ros2 run kk_driver_package kk_driver`

# ドキュメント

# DriverNode ドキュメント

## 概要
`DriverNode` は、ROS2 のノードであり、各種制御コマンドを受信し、それに対応するシリアル通信を行いながら、センサーデータをパブリッシュする役割を持ちます。本ノードを利用することで、EPB、PWM、モーター、BLDC、および C610 モジュールの制御が可能です。

## 制御用トピック

| トピック名 | メッセージ型 | 説明 |
|------------|--------------|------------------|
| `epb/cmd` | `kk_driver_msg::msg::EpbCmd` | EPB の制御コマンド |
| `pwm/cmd` | `kk_driver_msg::msg::PwmCmd` | PWM の制御コマンド |
| `mtr/cmd` | `kk_driver_msg::msg::MotorCmd` | モーターの制御コマンド |
| `bldc/cmd` | `kk_driver_msg::msg::BldcCmd` | BLDC の制御コマンド |
| `c610/cmd` | `kk_driver_msg::msg::C610Cmd` | C610 モーターの制御コマンド |
| `gm6020/cmd` | `kk_driver_msg::msg::Gm6020Cmd` | GM6020 モーターの制御コマンド |

## 状態用トピック

| トピック名 | メッセージ型 | 説明 |
|------------|--------------|------------------|
| `epb/status` | `kk_driver_msg::msg::EpbStatus` | EPB の状態情報 |
| `pwm/position` | `kk_driver_msg::msg::PwmStatus` | PWM の位置情報 |
| `mtr/encoder` | `kk_driver_msg::msg::Encoder` | エンコーダの値 |
| `c610/status` | `kk_driver_msg::msg::C610Status` | C610 モーターの状態情報 |
| `gm6020/status` | `kk_driver_msg::msg::Gm6020Status` | GM6020 モーターの状態情報 |

# メッセージ定義と説明
## 複数ポートあるメッセージの使い方
一つのメッセージで複数のポートを制御できます。

なるべくメッセージをまとめることでRaspberryPiとCAN Hat基板間の通信負荷を減らすことができます。

```cpp
kk_driver_msg::msg::PwmCmd msg;

//送信するポート数に合わせて配列長を設定
msg.port.resize(2);
msg.pos.resize(2);
msg.spd.resize(2);

// 複数ポート分の設定
msg.port[0] = 0;// ポート0のサーボ
msg.pos[0] = 200; // ポート0の指令値
msg.spd[0] = 7;

msg.port[1] = 3;// ポート3のサーボ
msg.pos[1] = 500; // ポート3の指令値
msg.spd[1] = 7;

// Publishする！！
pwm_cmd_pub->publish(msg);
```

## 動的なメッセージ格納方法
`push_back`関数を利用すると、`resize`関数を使用せずスマートにメッセージを設定できます。

ただし、各要素のpush_back回数が異なると制御に不具合が生じる場合があるため、充分に注意してください。

```cpp
kk_driver_msg::msg::PwmCmd msg;

// ポート0の設定
msg.port.push_back(0);
msg.pos.push_back(200);
msg.spd.push_back(7);

// ポート3の設定
msg.port.push_back(3);
msg.pos.push_back(500);
msg.spd.push_back(7);

// Publishする！！
pwm_cmd_pub->publish(msg);

```

# 各基板利用方法
## GM6020の制御
### モーターのコントロール
`gm6020/cmd`トピックスで、制御したいモーターの制御指令をSubscriptionしています。

処理落ちによる暴走防止のため、0.3秒以上モーターへの指令を受信しないとモーターは自動停止します。

メッセージ型：`kk_driver_msg::msg::Gm6020Cmd`

`モーターのID番号-1`番目の要素に該当のモータ情報が格納されます。

`motor_id`：1から始まるモーターのIDです。モーター裏面の物理スイッチで設定されます。
`duty`：-1~1の範囲で出力Duty比を指定します。範囲外の値は丸められます。

```
uint8[] motor_id
float[] duty 
```

### モーターの状態を取得
`gm6020/status`トピックで、接続可能なモーター(8個)の状態を定周期でPublishします。

メッセージ型：`kk_driver_msg::msg::Gm6020Status`

`モーターのID番号-1`番目の要素に該当のモータ情報が格納されます。

`position`：GM6020の回転角度を返します。単位系はGM6020の仕様に依存します。一周以上の回転に独自対応しています。
`speed`：GM6020の回転速度を返します。単位系はGM6020の仕様に依存します。
`torque`：GM6020の回転トルク(電流値)を返します。単位系はGM6020の仕様に依存します。

```
int32[8] position
int16[8] speed
int16[8] torque
```

## メッセージ型
`kk_driver_package/kk_driver_msg/msg`内に定義があります。
必要に応じて適切にincludeしてください。

例

```
#include "kk_driver_msg/msg/epb_cmd.hpp"
```
### `kk_driver_msg::msg::BldcCmd`
```cpp
uint8 child_id  // 基板CAN子ID
uint8[] port    // 制御対象のポート番号
uint16[] spd    // 設定する回転速度 0xFFFFを最高速度としたDuty値
```

### `kk_driver_msg::msg::C610Cmd`
モータの子CANID(0~)をポートとして取り扱います。トルク値は1000が最大です。
```cpp
uint8 child_id  // 0固定としてください
uint8[] port    // 制御対象のポート番号　モータの子ＩＤです
uint16[] torque // 設定するトルク値　(モータの仕様を確認してください)
```

### `kk_driver_msg::msg::C610Status`
モータにCAN IDが振られています。子ID0から7までの状態を順に格納しています。
```cpp
uint16[] position // モーターの現在位置　(モータの仕様を確認してください)
uint16[] speed    // モーターの現在速度　(モータの仕様を確認してください)
uint16[] torque   // モーターの現在トルク　(モータの仕様を確認してください)
```

### `kk_driver_msg::msg::Encoder`
```cpp
uint8 child_id   // 基板CAN子ID
uint32[] pos     // エンコーダの位置データ 生パルス数です
uint32[] spd     // エンコーダの速度データ 生パルス数です
```

### `kk_driver_msg::msg::EpbCmd`
```cpp
bool is_safety  // 安全状態の設定
```

### `kk_driver_msg::msg::EpbStatus`
```cpp
bool is_safety        // 安全状態のステータス
bool ope_epb_enable   // EPB の操作可否
```

### `kk_driver_msg::msg::KeyCtrl`
```cpp
int32 x            // 操作位置の X 座標
int32 y            // 操作位置の Y 座標
bool[256] keys     // キー入力情報（256 個）
```

### `kk_driver_msg::msg::MotorCmd`
```cpp
uint8 child_id    // 基板CAN子ID
uint8[] port      // 制御対象のポート番号
uint8[] ctrl      // 制御モード
int32[] target    // 目標値（位置や速度）10000~-10000の範囲で出力電圧を指定
```

### `kk_driver_msg::msg::PwmCmd`
```cpp
uint8 child_id    // 基板CAN子ID
uint8[] port      // 制御対象のポート番号
uint16[] pos      // PWM のPWM波ON時間 100 = 10ms
uint8[] spd       // PWM の設定速度 0(遅)~16(早)でサーボ速度を指定します。
```

### `kk_driver_msg::msg::PwmStatus`
```cpp
uint8 child_id  // 基板CAN子ID
bool is_power   // 電源状態 サーボの復帰制御などに活用できます。
uint16[] pos    // 現在の位置データ
```

# 新作ドキュメント
## 基板の操作
基板を操作するためには、kk_driverノードがロボット上のラズパイで起動した状態で、
操作したい基板のトピックをPublishします。

kk_driverノードの起動方法は下記の通りです。
```
ros2 run kk_driver kk_driver
```

### モーターの操作
モータドライバ基板に接続されたモータを制御します。
モータドライバには複数のモードが備わっています。
- フリーモード
- 電圧制御(Dury制御)
- 速度制御(Comming Soon)
- 位置制御(Comming Soon)
#### フリーモードで動作させるには
`/mtr/cmd`に、回したい軸の値を設定したメッセージをPublishします。
```

```