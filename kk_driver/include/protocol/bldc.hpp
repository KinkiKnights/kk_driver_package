#pragma once
#include <stdint.h>
#include "../util/byte_conversion.hpp"
#include "../util/param.hpp"
#include "./can_msg.hpp"

namespace Bldc{
namespace DEF
    {
        constexpr uint32_t MAX_TARGET = 0xffff;
    } // namespace name
    

    /**
     * @brief CANプロトコル
     * @note ブラシレスモータは基本2ポート
     */
    struct Can{
    public: // ユーティリティ関数
        /**
         * @brief CANのIDから子ＩＤを算出する
         * @return 子ID。想定外子IDの場合は0xFFを返す。
         **/
        static uint8_t getChildNumber(CanMessage& msg){
            uint8_t id = msg.id - Param::CAN_BASE_ID;
            if (id < Param::BOARD_NUM) return id;
            else return 0xff;
        }

        /**
         * @brief CAＮメッセージのIDが一致するかを確認する
         * @return 各基板側ファームからの利用が想定されている。
         */
        inline static bool isMe(CanMessage& msg, uint16_t& id){
            if (msg.id == id) return true;
            return false;
        }
    public: // メッセージエンコード・デコード
        uint16_t speed[2];

        /**
         * CANメッセージをエンコードする
         */
        CanMessage encode(uint16_t child_id, uint8_t port = 0){
            CanMessage msg;

            // ヘッダ情報設定
            msg.id = Param::CAN_BASE_ID + child_id;
            msg.dlc = 4;
            msg.port = port;

            for(uint8_t port = 0; port < 2; port++){
                // フレームの設定
                uint8_t* section = &msg.data[port*2];
                section[0] = speed[port] & 0xFF;
                section[1] = (speed[port] >> 8) & 0xFF;
                
            }
            return msg;
        }

        void decode(CanMessage& msg){
            for(uint8_t port = 0; port < 2; port++){
                uint8_t* section = &msg.data[port*2];
                speed[port] = section[1];
                speed[port] = (speed[port]<<8) + section[0];
            }
        }
    public: // 応用関数
        void setDuty(float duty, uint8_t port){
            speed[port] = uint16_t(duty * DEF::MAX_TARGET);
        }

        float getDuty(uint8_t port){
                return 1.f * speed[port] / DEF::MAX_TARGET;
        }
    };

    /**
     * @brief Serialプロトコル
     */
    struct Serial{
    public: // 設定値
        // 対称のボードのＩＤ
        uint8_t child_id = 0;
        // メッセージに含むポート数
        uint8_t port_num = 0;
        // 現在位置
        uint16_t speed[Param::PORT_NUM];
        uint8_t port[Param::PORT_NUM];

    public:
        /**
         * @brief シリアルコマンドのエンコード
         * @note farme1のDLCは、フレーム2以降のフレーム数、帰り値は総フレーム数
         */
        uint8_t encode(uint8_t* frame){
            // ヘッダ情報の設定
            uint8_t dlc = 3 * port_num + 1; 
            frame[0] = Param::SERIAL_ID;
            frame[1] = dlc;
            frame[2] = child_id;
            // 送信ポート数のバリデーション
            if (port_num > Param::PORT_NUM) return 0;

            
            for(uint8_t idx = 0; idx < 2; idx++){
                // フレームの設定
                uint8_t* section = &frame[idx*3 + 3];
                section[0] = port[idx];
                section[1] = speed[idx] & 0xFF;
                section[2] = (speed[idx] >> 8) & 0xFF;
                
            }
            return dlc + 2;
        }
        
        /**
         * @brief シリアルコマンドのデコード
         */
        void decode(uint8_t* frame){
            // ヘッダ情報の解析
            port_num = (frame[1] -1) / 3; // dlc抽出
            child_id = frame[2];

            // 送信ポート数のバリデーション
            if (port_num > Param::PORT_NUM){
                port_num = 0;
                return;
            }

            // 制御情報の取得
            for(uint8_t idx = 0; idx < 2; idx++){
                uint8_t* section = &frame[idx*3 + 3];
                port[idx] = section[0];
                speed[idx] = section[2];
                speed[idx] = (speed[idx]<<8) + section[1];
            }
        }
    };
};