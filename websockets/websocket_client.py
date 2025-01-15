import time
import websocket

def send_forever():
    uri = "ws://0.0.0.0:38976"
    # Establish the WebSocket connection
    ws = websocket.create_connection(uri)

    while True:
        # Send a message to the server
        ws.send("Hello from Docker!")
        print("Sent: Hello from Docker!")

        # Wait for the server’s response
        response = ws.recv()
        print(f"Received from server: {response}")

        # Wait for 1 second before sending the next message
        time.sleep(1)

if __name__ == "__main__":
    try:
        send_forever()
    except KeyboardInterrupt:
        print("Connection closed.")
