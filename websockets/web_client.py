import websocket

# 处理来自服务器的消息
def on_message(ws, message):
    print(f"Received message from server: {message}")

# 处理连接事件
def on_open(ws):
    print("Client: Connection opened")
    ws.send("Hello, Server!")  # 向服务器发送消息

# 处理关闭事件
def on_close(ws, close_status_code, close_msg):
    print("Client: Connection closed")

def run_client():
    ws = websocket.WebSocketApp(
        "ws://0.0.0.0:12345",  # 服务器地址
        on_message=on_message,
        on_open=on_open,
        on_close=on_close
    )
    ws.run_forever()

if __name__ == "__main__":
    run_client()
