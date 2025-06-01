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

class UART
{
private:
    int uart_fd_;
    char header[2] = {0xF0, 0xF0};
    std::thread recv_thread_;
    std::mutex buffer_mutex_;
    std::vector<uint8_t> ring_buffer_;
    std::atomic<bool> running_;
    std::atomic<bool> buffer_overflowed_;
    size_t buffer_head_;
    size_t buffer_tail_;

    bool initUart(){
        // UARTオープン
        uart_fd_ = open("/dev/serial0", O_RDWR | O_NOCTTY | O_NDELAY);
        if (uart_fd_ == -1) return false;

        // UART設定
        struct termios options;
        options.c_cflag = B38400 | CS8 | CLOCAL | CREAD;
        options.c_iflag = IGNPAR;
        options.c_oflag = 0;
        options.c_lflag = 0;
        tcsetattr(uart_fd_, TCSANOW, &options);

        return true;
    }

    void receiveLoop(){
        uint8_t temp_buffer[20000];
        while (running_){
            int len = read(uart_fd_, temp_buffer, sizeof(temp_buffer));
            if (len > 0){
                std::lock_guard<std::mutex> lock(buffer_mutex_);
                for (int i = 0; i < len; i++){
                    ring_buffer_[buffer_head_] = temp_buffer[i];
                    buffer_head_ = (buffer_head_ + 1) % BUFFER_SIZE;
                    if (buffer_head_ == buffer_tail_){
                        buffer_tail_ = (buffer_tail_ + 1) % BUFFER_SIZE; // オーバーフロー時に古いデータを削除
                        buffer_overflowed_ = true;
                    }
                }
            }
            usleep(1000); // 1msスリープ
        }
    }

public:
    UART(): ring_buffer_(BUFFER_SIZE), buffer_head_(0), buffer_tail_(0), running_(false), buffer_overflowed_(false)
    {
        if(initUart()){
            printf("Open Serial\n");
            if (uart_fd_ == -1)
                printf("Connect Disabel\n");
            running_ = true;
            recv_thread_ = std::thread(&UART::receiveLoop, this);
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

    size_t readBuffer(uint8_t* buffer, size_t max_len){
        std::lock_guard<std::mutex> lock(buffer_mutex_);
        size_t count = 0;
        while (buffer_tail_ != buffer_head_ && count < max_len){
            buffer[count++] = ring_buffer_[buffer_tail_];
            buffer_tail_ = (buffer_tail_ + 1) % BUFFER_SIZE;
        }
        return count;
    }

    bool isRingBufferEmpty() const {
        return buffer_head_ == buffer_tail_;
    }

    bool hasOverflowed() const {
        return buffer_overflowed_;
    }

    bool getByte(uint8_t &data){
        std::lock_guard<std::mutex> lock(buffer_mutex_);
        if (isRingBufferEmpty()) {
            return false; // バッファが空
        }
        data = ring_buffer_[buffer_tail_];
        buffer_tail_ = (buffer_tail_ + 1) % BUFFER_SIZE;
        return true;
    }

    // コマンドパース
    static const uint8_t TMP_NUM = 300;
    static constexpr uint8_t DLC_IDX = 1;
    uint8_t blank_counter = 0;
    uint8_t tmp_index = 0;
    uint32_t check_sum = 0;
    uint8_t cmd_temporary[TMP_NUM];

    // 返り値trueなら残あり
    bool getSerialMsg(uint8_t* frames, bool& is_success){
        uint8_t tmp_c;
        is_success = false;
        while(getByte(tmp_c)){
            printf("%d,", (uint8_t)tmp_c);
            // 先頭文字列探索
            if (blank_counter < 2){
                if(tmp_c == 0xF0)
                    blank_counter++;
                else
                    blank_counter = 0;
                printf(": blank search:%d\n", blank_counter);

            }
            // コマンド先頭探索
            else if (tmp_index == 0){
                // 初期文字列が続く場合はコマンド先頭探索
                if (tmp_c == 0xF0) continue;
                cmd_temporary[tmp_index++] = tmp_c;
                check_sum = tmp_c;
                // printf(":cmd type\n");
            }
            // コマンド処理
            else {
                if (tmp_index >= TMP_NUM) {
                    // バッファオーバーフロー時の処理
                    printf("cmd_temporary buffer overflow!\n");
                    blank_counter = 0;
                    tmp_index = 0;
                    return false;
                }
                cmd_temporary[tmp_index] = tmp_c;
                tmp_index++;
                // コマンド受信中間処理
                if (tmp_index < (cmd_temporary[DLC_IDX] + 3)){
                    check_sum += tmp_c;
                }
                // コマンド受信完了処理
                else {
                    // チェックサム正常計算時
                    if (check_sum % 0x100 == tmp_c)
                        is_success = true;
                    // チェックサム異常時処理
                    else 
                        printf("\nChecksum calc is failed(%d(calc) != %d(frame), str[%d])...", check_sum, tmp_c, tmp_index );
                    // 初期化処理
                    printf("cmd_cmp\n");
                    if (is_success){
                        for (uint8_t idx = 0; idx < tmp_index; idx++){
                            frames[idx] = cmd_temporary[idx];
                        }
                    }
                    blank_counter = 0;
                    tmp_index = 0;
                    return true;
                }
            }
        }
        return false;
    }
    
};
