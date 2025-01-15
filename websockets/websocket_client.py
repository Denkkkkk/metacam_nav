import rospy
import json
import websocket
from geometry_msgs.msg import Twist
from functools import partial

# ROS回调函数处理/cmd_vel话题消息
def cmd_vel_callback(msg,ws):
    send_message(msg,ws)
    
def send_message(msg,ws):
    try:
        # 将 ROS /cmd_vel 消息转换为 JSON 格式
        msg_json = json.dumps({
            "linear_x": msg.linear.x,
            "linear_y": msg.linear.y,
            "linear_z": msg.linear.z,
            "angular_x": msg.angular.x,
            "angular_y": msg.angular.y,
            "angular_z": msg.angular.z
        })
        # 发送 JSON 数据
        ws.send(msg_json)
    except Exception as e:
        rospy.logerr("发送消息失败: %s", str(e))

class CmdVelSend:
    def __init__(self):
         # 初始化 WebSocket
        self.ws = websocket.WebSocketApp("ws://0.0.0.0:12345", on_open=self.on_open)
        
    # WebSocket客户端的发送函数
    def on_open(self,ws):
        rospy.loginfo("WebSocket 连接已打开")

    # 设置 ROS 节点和 WebSocket
    def client_node(self):
        rospy.init_node('cmd_vel_websocket_client')
        
        # 订阅 /cmd_vel 话题
        rospy.Subscriber("/cmd_vel", Twist,  partial(cmd_vel_callback,self.ws))
        
        # 运行 WebSocket 客户端
        rospy.loginfo("开始连接 WebSocket 服务器")
        self.ws.run_forever()

if __name__ == '__main__':
    try:
       cmd_vel_send = CmdVelSend()
       cmd_vel_send.client_node()
    except rospy.ROSInterruptException:
        pass
