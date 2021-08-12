#!/usr/bin/env python
# license by pikachu team

import rospy
from cv_bridge import CvBridge
import numpy as np

from geometry_msgs.msg import Twist, TwistStamped, Vector3Stamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image

class roomba():
    def __init__(self):
        self.img_sub = rospy.Subscriber('/camera/depth/image_rect_raw', Image, self.depthimg_callback)
        self.vel_pub = rospy.Publisher('/command/velocity', Twist, queue_size = 5)

    def depthimg_callback(self,msg):
        bridge = CvBridge()
        cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding = 'passthrough')
        cv_image=cv_image[cv_image!=0]
        min_dist = np.min(cv_image)

        twistmsg = Twist()


	if min_dist < 400:
            twistmsg.angular.z = 0.1
            twistmsg.linear.x = 0
        else:
            twistmsg.angular.z = 0
            twistmsg.linear.x = 0.1

        self.vel_pub.publish(twistmsg)
        




if __name__ == '__main__':
    try:
	rospy.init_node('pikachumqtt',anonymous=True)
        roomba()
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            rate.sleep()
    except rospy.ROSInterruptException:
        pass
