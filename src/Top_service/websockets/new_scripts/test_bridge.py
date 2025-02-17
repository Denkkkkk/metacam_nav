
import roslibpy

ros = roslibpy.Ros(host='0.0.0.0', port=9090)

def on_ready():
    print('Connected to rosbridge in ROS2!')

    odom_topic = roslibpy.Topic(
        ros,
        '/tower/mapping/odometry',
        'nav_msgs/msg/Odometry'
    )

    def odom_callback(message):
        print('Odometry:', message)

    odom_topic.subscribe(odom_callback)

ros.on_ready(on_ready)
ros.run_forever()