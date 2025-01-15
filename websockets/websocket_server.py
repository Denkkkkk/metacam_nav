import websocket
import threading

# 处理来自客户端的消息
def on_message(ws, message):
    print(f"Received message from client: {message}")
    ws.send("Hello, Client! I received your message.")

# 处理连接事件
def on_open(ws):
    print("Server: Connection opened")

# 启动WebSocket服务器
def run_server():
        
    server = websocket.WebSocketApp(
        "ws://localhost:8080",  # 服务端地址
        on_message=on_message,
        on_open=on_open
    )

    # 启动 WebSocket 服务器
    server.run_forever()

if __name__ == "__main__":
    # 使用线程启动服务器，确保它在主线程之外运行
    server_thread = threading.Thread(target=run_server)
    server_thread.start()
    server_thread.join()
