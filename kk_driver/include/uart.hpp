#pragma once
// UART関連ライブラリ
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cstring>
#include <string>

class UART
{
private:
    int uart_fd_;
    char header[2] = {0xF0, 0xF0};

    bool initUart(){
        // UARTオープン
        uart_fd_ = open("/dev/serial0", O_RDWR | O_NOCTTY | O_NDELAY);
        if (uart_fd_ == 1) return false;

        // UART設定
        struct termios options;
        options.c_cflag = B115200 | CS8 | CLOCAL | CREAD;
        options.c_iflag = IGNPAR;
        options.c_oflag = 0;
        options.c_lflag = 0;
        tcsetattr(uart_fd_, TCSANOW, &options);

        return true;
    }

public:
    UART(){
        if(initUart())
            printf("Open Serial\n");
        else
            printf("Faild Serial\n");
    }

    ~UART(){
        if (uart_fd_ == -1) return;
        close(uart_fd_);
    }

    void sendTxt(std::string str){
        if (uart_fd_ == -1) return;
        write(uart_fd_, str.c_str(), str.size());
    }
    void sendFrame(uint8_t* frame,uint8_t num){
        if (uart_fd_ == -1) return;
        uint64_t sum = 0;
        for (uint8_t i = 0; i < num; i++){
            sum += frame[i];
            printf("%d,", frame[i]);
        }
        uint8_t checksum = sum % 0x100;
        write(uart_fd_, header, 2);
        write(uart_fd_, frame, num);
        write(uart_fd_, &checksum, 1);
        printf("=> CS=%d\n", checksum);

    }

    void readTxt(uint8_t* buffer, uint8_t &str_len){
        if (uart_fd_ == -1) return;
        memset(buffer, 0, sizeof(buffer));
        str_len = read(uart_fd_, buffer, sizeof(buffer) - 1);
    }

};