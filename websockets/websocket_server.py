import websocket
import threading
import time

# 服务器接收到消息后的回调函数
def on_message(ws, message):
    print(f"收到消息: {message}")
    ws.send("消息已接收，感谢连接！")  # 服务器响应客户端

# 服务器关闭后的回调函数
def on_close(ws, close_status_code, close_msg):
    print("连接关闭")

# 启动WebSocket服务器
def start_server():
    # WebSocket 服务器地址
    server_address = "ws://0.0.0.0:38976"
    
    # 创建 WebSocket 服务器实例
    server = websocket.WebSocketServer(server_address, on_message=on_message, on_close=on_close)
    print("WebSocket 服务器已启动，等待客户端连接...")
    server.run_forever()

if __name__ == "__main__":
    # 启动 WebSocket 服务器
    start_server()
