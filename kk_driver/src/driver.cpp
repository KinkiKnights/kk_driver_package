#include "../include/uart.hpp"
// ROS2関連ライブラリ
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include "../include/serial/serial_msg.hpp"
#include "kk_driver_msg/msg/pwm_cmd.hpp"
#include "kk_driver_msg/msg/pwm_status.hpp"
#include "kk_driver_msg/msg/motor_cmd.hpp"
#include "kk_driver_msg/msg/encoder.hpp"
#include "kk_driver_msg/msg/gm6020_cmd.hpp"
#include "kk_driver_msg/msg/gm6020_status.hpp"
// #include "kk_driver_msg/msg/core.hpp"
// #include "kk_driver_msg/msg/c610_cmd.hpp"
// #include "kk_driver_msg/msg/c610_status.hpp"
// #include "kk_driver_msg/msg/epb_cmd.hpp"
// #include "kk_driver_msg/msg/epb_status.hpp"
// #include "kk_driver_msg/msg/bldc_cmd.hpp"



class DriverNode : public rclcpp::Node{
private:
    // PWMサーボモータ
    rclcpp::Subscription<kk_driver_msg::msg::PwmCmd>::SharedPtr pwm_cmd_sub;
    rclcpp::Publisher<kk_driver_msg::msg::PwmStatus>::SharedPtr pwm_status_pub;
    // GM6020モータ
    rclcpp::Subscription<kk_driver_msg::msg::Gm6020Cmd>::SharedPtr gm6020_cmd_sub;
    rclcpp::Publisher<kk_driver_msg::msg::Gm6020Status>::SharedPtr gm6020_status_pub;
    // DCモーター
    rclcpp::Subscription<kk_driver_msg::msg::MotorCmd>::SharedPtr motor_cmd_sub;
    rclcpp::Publisher<kk_driver_msg::msg::Encoder>::SharedPtr encoder_status_pub;

    // rclcpp::Subscription<kk_driver_msg::msg::C610Cmd>::SharedPtr c610_cmd_sub;
    // rclcpp::Publisher<kk_driver_msg::msg::C610Status>::SharedPtr c610_status_pub;
    // rclcpp::Subscription<kk_driver_msg::msg::BldcCmd>::SharedPtr bldc_cmd_sub;
    // rclcpp::Publisher<kk_driver_msg::msg::Core>::SharedPtr core_pub;

private:
    UART serial;
    
    PwmServo::Serial pwm_encoder;
    IcsServo::Serial ics_encoder;
    GM6020::Serial gm6020_encoder;
    Motor::Serial motor_encoder;
    
    PwmFeedBack::Serial pwm_decoder;
    IcsFeedBack::Serial ics_decoder;
    GM6020FeedBack::Serial gm6020_decoder;
    MotorFeedback::Serial motor_decoder;
    kk_driver_msg::msg::Gm6020Status gm6020_feedback;

    const uint16_t GM6020_MAX  = 8191;
    uint32_t gm6020_ofst[7];

    rclcpp::TimerBase::SharedPtr timer_;

    void update(){
        uint8_t* rcv_array;
        uint8_t msg_head[200];
        uint8_t msg_num = serial.getSerialMsg(msg_head, &rcv_array);
        // printf("\nGet Msg(%d)",msg_num);
        for (uint8_t msg_idx = 0; msg_idx < msg_num; msg_idx++){
            // メッセージ先頭を格納
            uint8_t* frame = &(rcv_array[msg_head[msg_idx]]);
            // IDでの振り分け処理
            uint8_t id = frame[0];
            // printf("\nid %d:",id);

            if (id == GM6020FeedBack::Serial::SERIAL_ID){
                gm6020_decoder.decode(frame);
                for (uint8_t i = 0; i< gm6020_decoder.port_num; i++){
                    uint8_t port = gm6020_decoder.port[i];
                    uint8_t gm6020_ofs = gm6020_decoder.fb_posofs[i];
                    gm6020_feedback.position[port] = gm6020_decoder.fb_position[i] + gm6020_ofs * GM6020_MAX;
                    gm6020_feedback.speed[port] = gm6020_decoder.fb_speed[i];
                    gm6020_feedback.torque[port] = gm6020_decoder.fb_current[i];
                }
            }
            // if (id == EPBFeedBack::Param::SERIAL_ID) {
            //     kk_driver_msg::msg::Core msg;
            //     for (uint8_t i = 0; i < 8; i++){
            //         msg.cmd[i] = frames[2 + i];
            //     }
            //     msg.hp = frames[12];
            //     msg.limit = frames[13];
            //     msg.is_safety = frames[11];
            //     core_pub->publish(msg);
            // }
            // if (id == Encoder::Param::SERIAL_ID) {
            //     enc_uart_encoder.decode(frames);
            //     kk_driver_msg::msg::Encoder msg;
            //     msg.child_id = enc_uart_encoder.child_id;
            //     uint8_t port_num = enc_uart_encoder.port_num;
            //     msg.pos.resize(port_num);
            //     for (uint8_t i = 0; i < port_num; i++){
            //         msg.pos[i] = enc_uart_encoder.position[i];
            //     }
            //     encoder_status_pub->publish(msg);
            // }
            // if (id == C610_FB::Param::SERIAL_ID) {
            //     c610_status_uart_encoder.decode(frames);
            //     kk_driver_msg::msg::C610Status msg;
            //     uint8_t port_num = 4;
            //     msg.position.resize(port_num);
            //     msg.speed.resize(port_num);
            //     msg.torque.resize(port_num);
            //     for (uint8_t i = 0; i < port_num; i++){
            //         msg.position[i] = c610_status_uart_encoder.position[i];
            //         msg.speed[i] = c610_status_uart_encoder.speed[i];
            //         msg.torque[i] = c610_status_uart_encoder.torque[i];
            //     }
            //     c610_status_pub->publish(msg);
            // }
        }
        //基板状態アンサーバック
        gm6020_status_pub->publish(gm6020_feedback);
    }
public:
    DriverNode(const std::string& name_space="", 
    const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
    : Node("driver_node",name_space,options){
        
        // 制御コマンドTopicを受信するSubscriberを初期化
        pwm_cmd_sub = this->create_subscription<kk_driver_msg::msg::PwmCmd>("pwm/cmd", rclcpp::QoS(10),
            [&](const kk_driver_msg::msg::PwmCmd::SharedPtr msg){
                pwm_encoder.port_num = msg->port.size();
                for (uint8_t i = 0; i < pwm_encoder.port_num; i++){
                    pwm_encoder.port[i] = msg->port[i];
                    pwm_encoder.pos[i] = msg->target[i];
                    pwm_encoder.spd[i] = msg->spd[i];
                }
                uint8_t frame[255];
                serial.sendFrame(frame, pwm_encoder.encode(frame));
            }
        );
        motor_cmd_sub = this->create_subscription<kk_driver_msg::msg::MotorCmd>("mtr/cmd", rclcpp::QoS(10),
            [&](const kk_driver_msg::msg::MotorCmd::SharedPtr msg){
                uint8_t port_num = msg->port.size();
                motor_encoder.port_num = port_num;
                for (uint8_t i = 0; i < port_num; i++){
                    float duty = msg->duty[i];
                    if (duty>0.99f) duty = 0.99f;
                    if (duty<-0.99f) duty = -0.99f;
                    motor_encoder.duty[i] = static_cast<int16_t>(duty * 0x7FFF);
                    motor_encoder.port[i] = msg->port[i];
                }
                uint8_t frame[255];
                serial.sendFrame(frame, motor_encoder.encode(frame));
            }
        );
        gm6020_cmd_sub =  this->create_subscription<kk_driver_msg::msg::Gm6020Cmd>("gm6020/cmd", rclcpp::QoS(10),
            [&](const kk_driver_msg::msg::Gm6020Cmd::SharedPtr msg){
                uint8_t port_num = msg->motor_id.size();
                gm6020_encoder.port_num = port_num;
                for (uint8_t i = 0; i < port_num; i++){
                    float duty = msg->duty[i];
                    if (duty>0.99f) duty = 0.99f;
                    if (duty<-0.99f) duty = -0.99f;
                    gm6020_encoder.target[i] = static_cast<int16_t>(duty * 25000);
                    gm6020_encoder.port[i] = msg->motor_id[i];
                }
                uint8_t frame[255];
                serial.sendFrame(frame, gm6020_encoder.encode(frame));
            }        
        );

        // 基板状態を送信するPublisherを初期化
        pwm_status_pub = this->create_publisher<kk_driver_msg::msg::PwmStatus>("pwm/position", rclcpp::QoS(1));
        gm6020_status_pub = this->create_publisher<kk_driver_msg::msg::Gm6020Status>("gm6020/status", rclcpp::QoS(1));
        encoder_status_pub = this->create_publisher<kk_driver_msg::msg::Encoder>("mtr/encoder", rclcpp::QoS(1));
        // core_pub = this->create_publisher<kk_driver_msg::msg::Core>("core", rclcpp::QoS(1));
        // c610_status_pub = this->create_publisher<kk_driver_msg::msg::C610Status>("c610/status", rclcpp::QoS(1));

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(20), // 0.01秒ごとに実行
            std::bind(&DriverNode::update, this));
    }

};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DriverNode>());
    rclcpp::shutdown();
    return 0;
}