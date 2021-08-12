import paho.mqtt.client as mqtt
import json
import rospy

from geometry_msgs.msg import Twist, TwistStamped, Vector3Stamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Illuminance, Image, Imu, Joy, LaserScan, NavSatFix, NavSatStatus
from std_msgs.msg import Bool, Float32, Float64MultiArray

def on_connect(client, userdata, flags, rc):
    print("Connected with result code "+str(rc))
    # Subscribing in on_connect() means that if we lose the connection and
    # reconnect then subscriptions will be renewed.
    client.subscribe([("capra/odometry",0), ("capra/gnss",0)])

def on_message(client, userdata, msg):
    # print(msg.topic+" "+str(msg.payload))
    if msg.topic == "capra/odometry":
        parse_odometry(msg.payload)
    if msg.topic == "capra/gnss":
        parse_gnss(msg.payload)

def publish_velocity_mqtt(client, linear_velocity, angular_velocity):
    payload = {"header":{"frame_id":"frame_id"},"twist":{"linear":{"x":linear_velocity,"y":0.0,"z":0.0},"angular":{"x":0.0,"y":0.0,"z":angular_velocity}}}
    client.publish("capra/direct_velocity", payload)

def parse_odometry(odom_mqtt):
    odom = json.loads(odom_mqtt)


class ros_capra_interface():

    def __init__(self, client):

        self.client = client

        rospy.Subscriber('model/output', Twist, self.command_callback)

        # self.jackal_pos_pub = rospy.Publisher('jackal/position', Vector3Stamped)
        # self.jackal_yaw_pub = rospy.Publisher('jackal/yaw', )
        # self.jackal_ang_vel_pub = rospy.Publisher('jackal/angular_velocity', )
        # self.jackal_lin_vel_pub = rospy.Publisher('jackal/linear_velocity', )

        # self.jackal_imu_ang_vel_pub = rospy.Publisher('jackal/imu/angular_velocity', )
        # self.jackal_imu_lin_accel_pub = rospy.Publisher('jackal/imu/linear_acceleration', )        
        # self.imu_ang_vel_pub = rospy.Publisher('imu/angular_velocity', Vector3Stamped)
        # self.imu_lin_accel_pub = rospy.Publisher('imu/linear_acceleration', Vector3Stamped)

        # self.imu_comp_bear_pub = rospy.Publisher('imu/compass_bearing', )
        # self.gps_latlong_pub = rospy.Publisher('gps/latlong', )

        # self.coll_close_pub = rospy.Publisher('collision/close', )
        # self.coll_stuck_pub = rospy.Publisher('collision/stuck', )

        self.coll_any_pub = rospy.Publisher('/collision/any', Bool)
        self.coll_phys_pub = rospy.Publisher('/collision/physical', Bool)
        self.coll_close_pub = rospy.Publisher('/collision/close', Bool)
        self.coll_flip_pub = rospy.Publisher('/collision/flipped', Bool)
        self.coll_stuck_pub = rospy.Publisher('/collision/stuck', Bool)
        self.coll_out_geo_pub = rospy.Publisher('/collision/outside_geofence', Bool)

        self.lidar_pub = rospy.Publisher('/rplidar/scan', LaserScan)

        self.imu_comp_bear_pub = rospy.Publisher('/imu_um7/compass_bearing', Float32)
        self.imu_mag_pub = rospy.Publisher('/imu_um7/mag', Vector3Stamped)
        self.imu_data_pub = rospy.Publisher('/imu_um7/data', Imu)

        self.navsat_fix_pub = rospy.Publisher('/navsat/fix', NavSatFix)
        self.navsat_vel_pub = rospy.Publisher('/navsat/vel', TwistStamped)

        self.odom_pub = rospy.Publisher('/odometry/filtered', Odometry)
        self.imu_raw_pub = rospy.Publisher('/imu/data_raw', Imu)

        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist)

        self.and_ill_pub = rospy.Publisher('/android/illuminance', Illuminance)

    def command_callback(self, msg):
        publish_velocity_mqtt(self.client, msg.linear.x, msg.angular.z)

def initialize_connection():
    client = mqtt.Client()
    client.on_connect = on_connect
    client.on_message = on_message

    client.connect("10.46.28.1", 1883, 60)
    return client


def run_interface():
    client = initialize_connection()
    pikachu = ros_capra_interface(client)
    rate = rospy.Rate(100)

    while not rospy.is_shutdown():
        client.loop_start()
        client.loop_stop()
        rate.sleep()
    




# Blocking call that processes network traffic, dispatches callbacks and
# handles reconnecting.
# Other loop*() functions are available that give a threaded interface and a
# manual interface.
client.loop_forever()


# show_camera()

