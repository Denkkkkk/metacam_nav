import socket

def receive_data():
    # 创建一个 TCP/IP socket
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as server_sock:
        # 绑定到特定的地址和端口
        server_sock.bind(('0.0.0.0', 38976))  # 监听端口38976
        server_sock.listen(1)  # 最大连接数为1

        print("Server is listening on port 38976...")

        # 等待连接
        connection, client_address = server_sock.accept()

        with connection:
            print(f"Connection established with {client_address}")
            
            while True:
                # 持续接收数据（最大1024字节）
                data = connection.recv(1024)
                
                if data:
                    # 输出接收到的数据
                    print(f"Received data: {data.decode('utf-8')}")
                else:
                    # 如果没有接收到数据，等待新的连接或数据
                    print("No data received. Waiting for new data...")

if __name__ == '__main__':
    receive_data()
