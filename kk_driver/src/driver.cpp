#include "../include/uart.hpp"
// ROS2関連ライブラリ
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include "kk_driver_msg/msg/epb_cmd.hpp"
#include "kk_driver_msg/msg/epb_status.hpp"
#include "kk_driver_msg/msg/pwm_cmd.hpp"
#include "kk_driver_msg/msg/pwm_status.hpp"

#include "../include/command/epb.hpp"
#include "../include/command/pwm.hpp"

class DriverNode : public rclcpp::Node{
private:
    rclcpp::Subscription<kk_driver_msg::msg::EpbCmd>::SharedPtr epb_cmd_sub;
    rclcpp::Subscription<kk_driver_msg::msg::PwmCmd>::SharedPtr pwm_cmd_sub;
    rclcpp::Publisher<kk_driver_msg::msg::EpbStatus>::SharedPtr epb_status_pub;
    rclcpp::Publisher<kk_driver_msg::msg::PwmStatus>::SharedPtr pwm_status_pub;

private:
    UART serial;
    Command::Emergency epb_uart_encoder;
    Command::PwmServo pwm_uart_encoder;
public:
    DriverNode(const std::string& name_space="", 
    const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
    : Node("driver_node",name_space,options){
        
        // 制御コマンドTopicを受信するSubscriberを初期化
        epb_cmd_sub = this->create_subscription<kk_driver_msg::msg::EpbCmd>("epb_cmd", rclcpp::QoS(10),
            [&](const kk_driver_msg::msg::EpbCmd::SharedPtr msg){
                epb_uart_encoder.is_safety = msg->is_safety;
                uint8_t frame[3];
                serial.sendFrame(frame, epb_uart_encoder.encode(frame));
            }
        );
        pwm_cmd_sub = this->create_subscription<kk_driver_msg::msg::PwmCmd>("pwm_cmd", rclcpp::QoS(10),
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

        // 基板状態を送信するPublisherを初期化
        epb_status_pub = this->create_publisher<kk_driver_msg::msg::EpbStatus>("epb_status", rclcpp::QoS(1));
        pwm_status_pub = this->create_publisher<kk_driver_msg::msg::PwmStatus>("pwm_status", rclcpp::QoS(1));
    }

};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DriverNode>());
    rclcpp::shutdown();
    return 0;
}