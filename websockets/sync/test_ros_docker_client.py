import rospy
from geometry_msgs.msg import Twist
import socket

def callback(msg):    
    rospy.loginfo("Received on testa: %s", msg.linear.x)    
    rospy.loginfo("Received on testa: %s", msg.angular.z)
    data = str(msg.linear.x) + "#" + str(msg.angular.z)
    try:        
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:            
            sock.connect(('0.0.0.0', 38960))            
            rospy.loginfo("Sending message to System B")            
            sock.sendall(data.encode('utf-8'))    
    except socket.error as exc:        
        rospy.logerr("Caught exception socket.error : %s", exc)
        
def listener():    
    rospy.init_node('testa_listener', anonymous=True)    
    rospy.Subscriber('/cmd_vel', Twist, callback)    
    rospy.spin()
    

if __name__ == '__main__':    
    listener()