#pragma once
// UART関連ライブラリ
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cstring>
#include <string>
#include <thread>
#include <mutex>
#include <vector>
#include <atomic>

#define BUFFER_SIZE 10240
#define RCV_BUFFER_SIZE 500
class UART
{
private:
    uint8_t buff_select = 0;
    uint8_t buff_full = false;
    bool is_writing = false;
    // 受信バッファ
    uint8_t* rcv_buff[2]; 
    uint8_t buff1[RCV_BUFFER_SIZE];
    uint8_t buff2[RCV_BUFFER_SIZE];
    // 書き込みインデックス
    uint8_t write_idx = 0;

    void rcvLoop(){
        uint8_t temp_buffer[500];
        while (running_)
        {
            int len = read(uart_fd_, temp_buffer, sizeof(temp_buffer));

            // バッファ満杯ならしばらく待機
            if (buff_full){
                usleep(1);
                continue;
            }

            // バッファへの投入
            is_writing = true;
            for (int i = 0; i < len; i++) {
                rcv_buff[buff_select][write_idx++] = temp_buffer[i];
                if (write_idx < RCV_BUFFER_SIZE) continue;
                buff_full = true;
                break;
            }
            is_writing = false;
        }
    }

    uint8_t* getNewBuff(uint8_t& len){
        // 読み取り中なら一連の読み取りが終了するまで待機
        while (is_writing)
        {}
        // 返り値保存
        len = write_idx;
        uint8_t last_idx = buff_select;
        // バッファの入れ替え
        buff_select = (buff_select + 1)%2;
        write_idx = 0;
        buff_full = false;
        printf("@");
        
        return rcv_buff[last_idx];
    }

private:
    int uart_fd_;
    char header[2] = {0xF0, 0xF0};
    std::thread recv_thread_;
    std::mutex buffer_mutex_;
    std::atomic<bool> running_;

    bool initUart(){
        // UARTオープン
        uart_fd_ = open("/dev/serial0", O_RDWR | O_NOCTTY | O_NDELAY);
        if (uart_fd_ == -1) return false;

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
    UART(): running_(false)
    {
        // バッファの用意
        rcv_buff[0] = buff1;
        rcv_buff[1] = buff2;

        // UART通信を開始
        if(initUart()){
            printf("Open Serial\n");
            if (uart_fd_ == -1)
                printf("Connect Disabel\n");
            running_ = true;
            recv_thread_ = std::thread(&UART::rcvLoop, this);
        }
        else
            printf("Faild Serial\n");
    }

    ~UART(){
        if (uart_fd_ == -1) return;
        if (recv_thread_.joinable()){
            recv_thread_.join();
        }
        if (uart_fd_ != -1) {
            close(uart_fd_);
        }
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
    
    // コマンドパース
    static const uint8_t TMP_NUM = 100;
    static constexpr uint8_t DLC_IDX = 1;
    uint8_t blank_counter = 0;
    uint8_t tmp_index = 0;
    

    // メッセージ先頭数を返す
    uint8_t getSerialMsg(uint8_t* msg_heads, uint8_t** rcv_array){
        uint8_t msg_num = 0;
        uint8_t tmp_c;
        // 新規メッセージの取得
        uint8_t get_len;
        uint8_t* rcv = getNewBuff(get_len);

        for (uint16_t i = 0; i < get_len; i++){
            // 解析文字取り出し
            tmp_c = rcv[i];
            printf("%d,", (uint8_t)tmp_c);

            // 先頭文字探索
            if (blank_counter < 2){
                if(tmp_c == 0xF0)
                    blank_counter++;
                else
                    blank_counter = 0;
                printf(": blank search:%d\n", blank_counter);
                continue;
            }
            // 初期文字列が続く場合はコマンド先頭探索
            if (tmp_c == 0xF0) continue;
            // メッセージ先頭を記録
            msg_heads[msg_num] = i;
            // 範囲チェック
            if (i + 1 >= get_len) break;; // DLCが受信範囲内か(最低限)
            uint8_t dlc = rcv[i+1];
            if (i + dlc + 2 >= get_len) break; // チェックサムが受信範囲内か
            if (dlc > 80) {
                // 異常なDLCを除外
                blank_counter = 0;
                continue;
            }
            uint8_t check_sum = rcv[i+dlc+2];
            // チェックサムを確認
            uint32_t sum = 0;
            for (uint8_t sum_idx = 0; sum_idx < dlc + 2; sum_idx++){
                sum += rcv[i + sum_idx];
                printf("/%d",rcv[i+sum_idx]);
            }
            // チェックサム判定
            if (sum % 0x100 == check_sum){
                // チェックサムが正しければ受信メッセージ数を更新
                msg_num++;
                printf("=(%d,%d)::success!!\n", check_sum,sum % 0x100);
            } else {
                printf("=(%d,%d)::fails!!\n", check_sum,sum % 0x100);
            }
            // iの更新(forの更新を見据えて1少なく)
            i += (dlc + 2);
            blank_counter = 0;
            continue;
        }
        blank_counter = 0;
        printf("=================================\n");

        // 返り値の用意
        *rcv_array = rcv;
        return msg_num;
    }
    
};
