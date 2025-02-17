#pragma once
#include <stdint.h>
#include "../util/byte_conversion.hpp"
#include "../util/param.hpp"
#include "./can_msg.hpp"


namespace Encoder{
    
    /**
     * @brief CANプロトコル
     * @note エンコーダは基本2ポート.4ポート基板はCANIDが8オフセットしたID構成になっている
     */
    struct Can{
        static const uint16_t OFFSET_ID = 0x08;

    public: // ユーティリティ関数
        /**
         * @brief CANのIDから子ＩＤを算出する
         * @return 子ID。想定外子IDの場合は0xFFを返す。
         **/
        static uint8_t getChildNumber(CanMessage& msg){
            uint8_t id = (msg.id|(~OFFSET_ID)) - Param::CAN_BASE_ID;
            if (id < Param::BOARD_NUM) return id;
            else return 0xff;
        }

        /**
         * @brief CAＮメッセージのIDが一致するかを確認する
         * @return 各基板側ファームからの利用が想定されている。
         */
        inline static bool isMe(CanMessage& msg, uint16_t& id){
            if (msg.id == id) return true;
            if (msg.id == (id + OFFSET_ID))return true;
            return false;
        }
    public: // メッセージエンコード・デコード
        bool offset_port;
        int32_t position[2];

        /**
         * CANメッセージをエンコードする
         */
        CanMessage encode(uint16_t child_id, uint8_t port = 0){
            CanMessage msg;

            // ヘッダ情報設定
            msg.id = Param::CAN_BASE_ID + child_id;
            if (offset_port) msg.id += OFFSET_ID;
            msg.dlc = 8;
            msg.port = port;

            for(uint8_t port = 0; port < 2; port++){
                // フレームの設定
                Command::int32_to_array(&msg.data[port*4], position[port]);
            }
            return msg;
        }

        void decode(CanMessage& msg){
            offset_port = ((msg.id & OFFSET_ID) > 0);
            for(uint8_t port = 0; port < 2; port++){
                Command::array_to_int32(&msg.data[port*4], position[port]);
            }
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
        int32_t position[Param::PORT_NUM];

    public:
        /**
         * @brief シリアルコマンドのエンコード
         * @note farme1のDLCは、フレーム2以降のフレーム数、帰り値は総フレーム数
         */
        uint8_t encode(uint8_t* frame){
            // ヘッダ情報の設定
            uint8_t dlc = 4 * port_num + 1; 
            frame[0] = Param::SERIAL_ID;
            frame[1] = dlc;
            frame[2] = child_id;
            // 送信ポート数のバリデーション
            if (port_num > Param::PORT_NUM) return 0;

            // 現在位置情報の設定
            for (uint8_t idx = 0; idx < port_num; idx++){
                Command::int32_to_array(&frame[idx * 4 + 3], position[idx]);
            }
            return dlc + 2;
        }
        
        /**
         * @brief シリアルコマンドのデコード
         */
        void decode(uint8_t* frame){
            // ヘッダ情報の解析
            port_num = (frame[1] -1) / 4; // dlc抽出
            child_id = frame[2];

            // 送信ポート数のバリデーション
            if (port_num > Param::PORT_NUM){
                port_num = 0;
                return;
            }

            // 制御情報の取得
            for (uint8_t idx = 0; idx < port_num; idx++){
                // ポートごとの情報の先頭を計算
                Command::array_to_int32(&frame[idx * 4 + 3], position[idx]);
            }
        }
    };
};