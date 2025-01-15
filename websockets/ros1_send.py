import asyncio
import websocket
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
    def __init__(self):
        # 初始化 ROS 节点
        rospy.init_node('cmd_vel_subscriber_node', anonymous=True)
        # 订阅 /cmd_vel 话题，设置消息类型为 Twist
        rospy.Subscriber("/cmd_vel", Twist, self.callback)
        # 打印订阅者启动信息
        rospy.loginfo("Subscribed to /cmd_vel topic")
        self.ws = websocket.WebSocketApp("ws://0.0.0.0:38976", on_open=self.on_open)
        
    # WebSocket客户端的发送函数
    def on_open(self,ws):
        rospy.loginfo("WebSocket 连接已打开")

    def callback(self, msg):
        # 处理接收到的 /cmd_vel 消息
        formatted_msg = format_cmd_vel_message(msg)
        # self.ws.send(formatted_msg)
        print(f"Sent: {formatted_msg}")
        
    def spin(self):
        # 保持节点的活动
        self.ws.run_forever()
        rospy.spin()
        
        
if __name__ == "__main__":
    # #ROS节点运行在事件循环中
    # loop = asyncio.get_event_loop()
    # try:
    #     loop.run_until_complete(send_forever())
    # finally:
    #     loop.close()
    
    try:
        cmd_vel_subscriber = Ros1Send()
        cmd_vel_subscriber.spin()  # 启动 ROS 循环，等待回调
    except rospy.ROSInterruptException:
        rospy.loginfo("ROS Node interrupted.")
