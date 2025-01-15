import websocket

def subscribe_once_sync(host='0.0.0.0', port=38976, topic='/cmd_vel'):
    """
    连接到 rosbridge WebSocket, 订阅指定话题, 只获取一帧后返回(OpenCV 格式).
    不需要使用 asyncio.
    
    :param host: rosbridge 所在的主机 IP 或域名
    :param port: rosbridge 对外暴露的端口
    :param topic: 需要订阅的 ROS2 话题（本例中消息类型为 sensor_msgs/msg/CompressedImage）
    :return: 如果成功获取到图片，则返回 OpenCV 格式的图像 (numpy 数组);
             如果获取失败则返回 None.
    """

    uri = f"ws://{host}:{port}"
    print(f"[subscribe_once_sync] Connecting to {uri} ...")

    # 使用 websocket-client 进行同步连接
    ws = websocket.WebSocket()
    ws.connect(uri)
    print(f"[subscribe_once_sync] Connected!")
    
    try:
        # 1. 发送订阅请求
        subscribe_msg = {
            "op": "subscribe",
            "topic": topic,
            "type": "geometry_msgs/Twist",  # ROS1 消息类型写法
            "queue_length": 1
        }
        ws.send(json.dumps(subscribe_msg))
        print(f"[subscribe_once_sync] Subscribed to {topic}, waiting for a message...")

        # 2. 不断读取服务端发来的消息，直到拿到第一帧图像
        while True:
            response_str = ws.recv()  # 同步阻塞调用，直到收到数据为止
            response = json.loads(response_str)
            print(response_str)

            # # 根据 rosbridge 协议，只有 "op" == "publish" 才是真正的话题数据
            # if response.get("op") == "publish" and response.get("topic") == topic:
            #     msg = response.get("msg", {})
            #     if "data" in msg:
            #         # base64 解码
            #         img_b64 = msg["data"]
            #         img_binary = base64.b64decode(img_b64)

            #         # 转为 numpy 数组
            #         np_arr = np.frombuffer(img_binary, np.uint8)
            #         # OpenCV 解码为 BGR 图像
            #         image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

            #         # 拿到就返回
            #         if image_np is not None:
            #             print("[subscribe_once_sync] Received one frame!")
            #             return image_np
            #         else:
            #             print("[subscribe_once_sync] Failed to decode image data.")
            #             return None
            #     else:
            #         print("[subscribe_once_sync] Received publish message, but no 'data' field found.")
            #         return None
    finally:
        # 不管有没有收到帧，都要关闭连接
        ws.close()