import asyncio
import websockets

async def send_forever():
    uri = "ws://0.0.0.0:38976"
    async with websockets.connect(uri) as websocket:
        # while True:
            # Send a message to the server
            await websocket.send("Hello from Docker!")
            print("Sent: Hello from Docker!")

            # Wait for the server’s response
            response = await websocket.recv()
            print(f"Received from server: {response}")

            # Wait for 1 second before sending the next message
            # await asyncio.sleep(0.01)

if __name__ == "__main__":
    loop = asyncio.get_event_loop()
    try:
        loop.run_until_complete(send_forever())
    finally:
        loop.close()