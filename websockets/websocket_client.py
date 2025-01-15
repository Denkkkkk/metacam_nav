import websocket

# 定义回调函数
def on_message(ws, message):
    print(f"接收到消息: {message}")

def on_error(ws, error):
    print(f"发生错误: {error}")

def on_close(ws, close_status_code, close_msg):
    print("WebSocket 连接关闭")

def on_open(ws):
    print("WebSocket 连接已打开")
    ws.send("你好，服务器！")  # 向服务器发送消息

# 创建 WebSocket 客户端并连接到服务器
def connect_to_server():
    ws = websocket.WebSocketApp("ws://0.0.0.0:36895/",  # 服务器地址
                                on_message=on_message,
                                on_error=on_error,
                                on_close=on_close,
                                on_open=on_open)
    ws.run_forever()

if __name__ == "__main__":
    # 启动客户端连接
    connect_to_server()
