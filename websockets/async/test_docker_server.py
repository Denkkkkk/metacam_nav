import asyncio
import websockets

async def echo(websocket, path):
    while True:
        message = await websocket.recv()
        print(f"Received message: {message}")
        await websocket.send(f"Echo: {message}")

if __name__ == "__main__":
    # Get or create the event loop
    loop = asyncio.get_event_loop()
    
    # Create the server (returns a "serve" object)
    server = websockets.serve(echo, "0.0.0.0", 38976)

    # Start the server
    loop.run_until_complete(server)
    print("WebSocket server started on port 38976...")

    try:
        # Keep the server running indefinitely
        loop.run_forever()
    except KeyboardInterrupt:
        pass
    finally:
        loop.close()