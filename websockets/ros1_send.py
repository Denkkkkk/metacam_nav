import asyncio
import websockets
import rospy
from geometry_msgs.msg import Twist
import json

# 定义将 ROS `/cmd_vel` 消息格式化为 WebSocket 消息的函数
def format_cmd_vel_message(cmd_vel_msg):
    # 你可以选择任何格式，这里选择将数据转为JSON
    return json.dumps({
        "linear": {
            "x": cmd_vel_msg.linear.x,
            "y": cmd_vel_msg.linear.y,
            "z": cmd_vel_msg.linear.z
        },
        "angular": {
            "x": cmd_vel_msg.angular.x,
            "y": cmd_vel_msg.angular.y,
            "z": cmd_vel_msg.angular.z
        }
    })
    
class Ros1Send:
    def __init__(self,websocket):
        # 初始化 ROS 节点
        rospy.init_node('cmd_vel_subscriber_node', anonymous=True)
        # 订阅 /cmd_vel 话题，设置消息类型为 Twist
        rospy.Subscriber("/cmd_vel", Twist, self.callback)
        # 打印订阅者启动信息
        rospy.loginfo("Subscribed to /cmd_vel topic")
        self.websocket = websocket
        
    async def websocket_send(self, msg):
        # 处理消息的异步操作
        await self.websocket.send(str(msg))  # 示例，发送 WebSocket 消息

    def callback(self, msg):
        # 处理接收到的 /cmd_vel 消息
        # 调度异步任务
        formatted_msg = format_cmd_vel_message(msg)
        asyncio.run(websocket_send(msg))
            
        print(f"Sent: {formatted_msg}")
        
    def spin(self):
        # 保持节点的活动
        rospy.spin()
    
# def callback(msg):
#     # 获取当前线程的事件循环
#     loop = asyncio.new_event_loop()  # 创建一个新的事件循环
#     asyncio.set_event_loop(loop)  # 设置当前线程的事件循环为新创建的循环
#     # 当收到 /cmd_vel 消息时，将其格式化并发送到 WebSocket
#     formatted_msg = format_cmd_vel_message(msg)
#     websocket.send("Hello from ros1_send2!")
#     print(f"Sent: {formatted_msg}")
    
#     response = websocket.recv()
#     print(f"Received from server: {response}")

# # 定义订阅 /cmd_vel 话题并将消息发送到 WebSocket 的函数
async def send_forever():
    uri = "ws://0.0.0.0:38976"  # WebSocket服务器地址
    async with websockets.connect(uri) as websocket:
        # 订阅 ROS1 /cmd_vel 话题
        await websocket.send("Hello from ros1_send1!")
        
        cmd_vel_subscriber = Ros1Send(websocket)
        cmd_vel_subscriber.spin()  # 启动 ROS 循环，等待回调
        
        
# 启动异步任务
if __name__ == "__main__":
    #ROS节点运行在事件循环中
    loop = asyncio.get_event_loop()
    try:
        loop.run_until_complete(send_forever())
    finally:
        loop.close()
    
    # try:
    #     uri = "ws://0.0.0.0:38976"  # WebSocket服务器地址
    #     with websockets.connect(uri) as websocket:
    #         # 实例化 CmdVelSubscriber 类并启动 ROS 循环
    #         cmd_vel_subscriber = Ros1Send(websocket)
    #         cmd_vel_subscriber.spin()  # 启动 ROS 循环，等待回调
    # except rospy.ROSInterruptException:
    #     rospy.loginfo("ROS Node interrupted.")
