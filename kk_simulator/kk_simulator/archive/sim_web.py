import sys
import os
sys.path.append(os.path.dirname(__file__))  # 現在のディレクトリをパスに追加

import kk_http
import ws_node


def main():    
    http_server = kk_http.Server()
    ws = ws_node.WsNode() # コンストラクタでasyncio開始


if __name__ == '__main__':
    main()