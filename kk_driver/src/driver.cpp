#include "../include/uart.hpp"
// ROS2関連ライブラリ
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include "kk_driver_msg/msg/epb_cmd.hpp"
#include "kk_driver_msg/msg/pwm_cmd.hpp"
#include "kk_driver_msg/msg/motor_cmd.hpp"
#include "kk_driver_msg/msg/bldc_cmd.hpp"
#include "kk_driver_msg/msg/epb_status.hpp"
#include "kk_driver_msg/msg/pwm_status.hpp"
#include "kk_driver_msg/msg/encoder.hpp"

#include "../include/protocol/_protocol.hpp"


class DriverNode : public rclcpp::Node{
private:
    rclcpp::Subscription<kk_driver_msg::msg::EpbCmd>::SharedPtr epb_cmd_sub;
    rclcpp::Subscription<kk_driver_msg::msg::PwmCmd>::SharedPtr pwm_cmd_sub;
    rclcpp::Subscription<kk_driver_msg::msg::MotorCmd>::SharedPtr motor_cmd_sub;
    rclcpp::Subscription<kk_driver_msg::msg::BldcCmd>::SharedPtr bldc_cmd_sub;
    rclcpp::Publisher<kk_driver_msg::msg::EpbStatus>::SharedPtr epb_status_pub;
    rclcpp::Publisher<kk_driver_msg::msg::PwmStatus>::SharedPtr pwm_status_pub;
    rclcpp::Publisher<kk_driver_msg::msg::Encoder>::SharedPtr encoder_status_pub;

private:
    UART serial;
    EPB::Serial epb_uart_encoder;
    ServoPwm::Serial pwm_uart_encoder;
    Motor::Serial motor_uart_encoder;
    Bldc::Serial bldc_uart_encoder;
    Encoder::Serial enc_uart_encoder;
    rclcpp::TimerBase::SharedPtr timer_;
    

    void update(){
        uint8_t frames[200];
        bool is_success;
        while(serial.getSerialMsg(frames, is_success)){
            if (!is_success) continue;
            uint8_t id = frames[0];
            // エンコーダ処理
            printf("id :%d\n",id);
            if (id == Encoder::Param::SERIAL_ID) {
                enc_uart_encoder.decode(frames);
                kk_driver_msg::msg::Encoder msg;
                msg.child_id = enc_uart_encoder.child_id;
                uint8_t port_num = enc_uart_encoder.port_num;
                msg.pos.resize(port_num);
                for (uint8_t i = 0; i < port_num; i++){
                    msg.pos[i] = enc_uart_encoder.position[i];
                }
                encoder_status_pub->publish(msg);
            }
        }
    }
public:
    DriverNode(const std::string& name_space="", 
    const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
    : Node("driver_node",name_space,options){
        
        // 制御コマンドTopicを受信するSubscriberを初期化
        epb_cmd_sub = this->create_subscription<kk_driver_msg::msg::EpbCmd>("epb/cmd", rclcpp::QoS(10),
            [&](const kk_driver_msg::msg::EpbCmd::SharedPtr msg){
                epb_uart_encoder.is_safety = msg->is_safety;
                uint8_t frame[3];
                serial.sendFrame(frame, epb_uart_encoder.encode(frame));
            }
        );
        pwm_cmd_sub = this->create_subscription<kk_driver_msg::msg::PwmCmd>("pwm/cmd", rclcpp::QoS(10),
            [&](const kk_driver_msg::msg::PwmCmd::SharedPtr msg){
                pwm_uart_encoder.child_id = msg->child_id;
                pwm_uart_encoder.port_num = msg->port.size();
                for (uint8_t i = 0; i < pwm_uart_encoder.port_num; i++){
                    pwm_uart_encoder.port[i] = msg->port[i];
                    pwm_uart_encoder.pos[i] = msg->pos[i];
                    pwm_uart_encoder.spd[i] = msg->spd[i];
                }
                uint8_t frame[255];
                serial.sendFrame(frame, pwm_uart_encoder.encode(frame));
            }
        );
        motor_cmd_sub = this->create_subscription<kk_driver_msg::msg::MotorCmd>("mtr/cmd", rclcpp::QoS(10),
            [&](const kk_driver_msg::msg::MotorCmd::SharedPtr msg){
                motor_uart_encoder.child_id = msg->child_id;
                uint8_t port_num = msg->port.size();
                bldc_uart_encoder.port_num = port_num;
                motor_uart_encoder.port_num = port_num;
                for (uint8_t i = 0; i < port_num; i++){
                    motor_uart_encoder.port[i] = msg->port[i];
                    motor_uart_encoder.ctrl[i] = msg->ctrl[i];
                    motor_uart_encoder.target[i] = msg->target[i];
                }
                uint8_t frame[255];
                serial.sendFrame(frame, motor_uart_encoder.encode(frame));
            }
        );
        bldc_cmd_sub = this->create_subscription<kk_driver_msg::msg::BldcCmd>("bldc/cmd", rclcpp::QoS(10),
            [&](const kk_driver_msg::msg::BldcCmd::SharedPtr msg){
                bldc_uart_encoder.child_id = msg->child_id;
                uint8_t port_num = msg->port.size();
                bldc_uart_encoder.port_num = port_num;
                printf("port_num = %d\n", port_num);
                for (uint8_t i = 0; i < port_num; i++){
                    bldc_uart_encoder.port[i] = msg->port[i];
                    bldc_uart_encoder.speed[i] = msg->spd[i];
                    printf("BLDC = %d=>%d, \n", bldc_uart_encoder.port[i], bldc_uart_encoder.speed[i]);
                }
                uint8_t frame[255];
                serial.sendFrame(frame, bldc_uart_encoder.encode(frame));
            }
        );

        // 基板状態を送信するPublisherを初期化
        epb_status_pub = this->create_publisher<kk_driver_msg::msg::EpbStatus>("epb/status", rclcpp::QoS(1));
        pwm_status_pub = this->create_publisher<kk_driver_msg::msg::PwmStatus>("pwm/position", rclcpp::QoS(1));
        encoder_status_pub = this->create_publisher<kk_driver_msg::msg::Encoder>("mtr/encoder", rclcpp::QoS(1));
        
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50), // 1秒ごとに実行
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