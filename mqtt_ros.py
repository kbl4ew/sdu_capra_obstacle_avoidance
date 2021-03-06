import paho.mqtt.client as mqtt
import json
import rospy

from geometry_msgs.msg import Twist, TwistStamped, Vector3Stamped
from nav_msgs.msg import Odometry

class ros_capra_interface():

    def __init__(self, client):
        self.command_sub = rospy.Subscriber('command/velocity', Twist, self.command_callback)
        self.odom_pub = rospy.Publisher('odometry', Odometry)

    def command_callback(self, msg):
        command_velocity_mqtt(self, msg.linear.x, msg.angular.z)


def on_connect(client, userdata, flags, rc):
    print("Connected with result code "+str(rc))
    # client.subscribe([("capra/odometry",0), ("capra/gnss",0)])
    client.subscribe("capra/odometry", 0)

def on_message(client, userdata, msg):
    # print(msg.topic+" "+str(msg.payload))
    if msg.topic == "capra/odometry":
        client.ros_interface.odom_pub.publish(parse_odometry(msg.payload))
    # if msg.topic == "capra/gnss":
    #     parse_gnss(msg.payload)

def parse_odometry(odom_mqtt):
    odom = json.loads(odom_mqtt)

def command_velocity_mqtt(client, linear_velocity, angular_velocity):
    payload = {"header":{"frame_id":"hello_id"},"twist":{"linear":{"x":linear_velocity,"y":0.0,"z":0.0},"angular":{"x":0.0,"y":0.0,"z":angular_velocity}}}
    client.publish("capra/direct_velocity", payload)



def initialize_connection():
    client = mqtt.Client()
    client.ros_interface = ros_capra_interface()
    client.on_connect = on_connect
    client.on_message = on_message

    client.connect("10.46.28.1", 1883, 60)
    return client


def run_interface():
    client = initialize_connection()
    rate = rospy.Rate(100)

    while not rospy.is_shutdown():
        client.loop()
        rate.sleep()



    
if __name__ == '__main__':
    try:
        run_interface()
    except rospy.ROSInterruptException:
        pass

