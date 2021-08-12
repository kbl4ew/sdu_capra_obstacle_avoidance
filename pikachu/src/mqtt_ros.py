#!/usr/bin/env python
# license by pikachu team

import paho.mqtt.client as mqtt
import json
import rospy

from geometry_msgs.msg import Twist, TwistStamped, Vector3Stamped
from nav_msgs.msg import Odometry

linear_vel_cmd = 0.0
angular_vel_cmd = 0.0

class interface():
     def __init__(self, rosClient, mqttClient):
        self.rosClient = rosClient
        self.mqttClient = mqttClient

class ros_capra_interface():

    def __init__(self):
        self.command_sub = rospy.Subscriber('/command/velocity', Twist, self.command_callback)
        self.odom_pub = rospy.Publisher('odometry', Odometry, queue_size = 5)


    def command_callback(self, msg):
        global linear_vel_cmd 
        linear_vel_cmd = msg.linear.x
        global angular_vel_cmd 
        angular_vel_cmd = msg.angular.z
#        command_velocity_mqtt(self, msg.linear.x, msg.angular.z)


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
    msg = Odometry()
    msg.child_frame_id = odom["child_frame_id"]
    msg.pose.pose.position.x = odom["pose"]["pose"]["position"]["x"]
    msg.pose.pose.position.y = odom["pose"]["pose"]["position"]["y"]
    msg.pose.pose.position.z = odom["pose"]["pose"]["position"]["z"]

    msg.pose.pose.orientation.x = odom["pose"]["pose"]["orientation"]["x"]
    msg.pose.pose.orientation.y = odom["pose"]["pose"]["orientation"]["y"]
    msg.pose.pose.orientation.z = odom["pose"]["pose"]["orientation"]["z"]
    msg.pose.pose.orientation.w = odom["pose"]["pose"]["orientation"]["w"]
    return msg
      
def command_velocity_mqtt(client, linear_velocity, angular_velocity):
    payload = {"header":{"frame_id":"hello_id"},"twist":{"linear":{"x":linear_velocity,"y":0.0,"z":0.0},"angular":{"x":0.0,"y":0.0,"z":angular_velocity}}}
#    print(payload)
    client.publish("capra/direct_velocity", json.dumps(payload))



def initialize_connection():
    client = mqtt.Client()
    client.ros_interface = ros_capra_interface()
    client.on_connect = on_connect
    client.on_message = on_message

    client.connect("10.46.28.1", 1883, 60)
    return client


def run_interface():
    client = initialize_connection()
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        command_velocity_mqtt(client, linear_vel_cmd, angular_vel_cmd)
        client.loop()
        rate.sleep()



    
if __name__ == '__main__':
    try:
	rospy.init_node('pikachumqtt',anonymous=True)
        run_interface()
    except rospy.ROSInterruptException:
        pass

