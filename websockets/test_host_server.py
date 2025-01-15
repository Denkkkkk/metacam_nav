import asyncio
import websockets
import json

async def echo(websocket):
    while True:
        message = await websocket.recv()
        
        try:
            # 尝试将接收到的消息解析为 JSON 格式
            # data = json.loads(message)
            print(f"Received JSON data: {message}")
        except json.JSONDecodeError:
            print("Received non-JSON message:")
            print(f"Message: {message}")
        
        # 响应消息
        await websocket.send(f"Host Server Echo: {message}")

async def main():
    async with websockets.serve(echo, "0.0.0.0", 12345):
        print("WebSocket server started on port 12345...")
        # Keep the server running indefinitely
        await asyncio.Future()

if __name__ == "__main__":
    asyncio.run(main())
