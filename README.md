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

## 状態用トピック

| トピック名 | メッセージ型 | 説明 |
|------------|--------------|------------------|
| `epb/status` | `kk_driver_msg::msg::EpbStatus` | EPB の状態情報 |
| `pwm/position` | `kk_driver_msg::msg::PwmStatus` | PWM の位置情報 |
| `mtr/encoder` | `kk_driver_msg::msg::Encoder` | エンコーダの値 |
| `c610/status` | `kk_driver_msg::msg::C610Status` | C610 モーターの状態情報 |

# メッセージ定義と説明
## 複数ポートあるメッセージの使い方
一つのメッセージで複数のポートを制御できます。

なるべくメッセージをまとめることでRaspberryPiとCAN Hat基板間の通信負荷を減らすことができます。

```
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