#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
import socket

def callback(msg):    
    # rospy.logwarn("Received on testa,linear.x: %s", msg.linear.x)    
    # rospy.logwarn("Received on testa,angular.z: %s", msg.angular.z)
    data = str(msg.linear.x) + "#" + str(msg.angular.z)
    try:        
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:            
            sock.connect(('0.0.0.0', 38960))            
            rospy.logwarn("Sending message to System B")            
            sock.sendall(data.encode('utf-8'))    
    except socket.error as exc:        
        rospy.logerr("Caught exception socket.error : %s", exc)
        
def listener():    
    rospy.init_node('docker_send', anonymous=True)    
    rospy.Subscriber('/cmd_vel', Twist, callback)    
    rospy.spin()
    

if __name__ == '__main__':    
    listener()