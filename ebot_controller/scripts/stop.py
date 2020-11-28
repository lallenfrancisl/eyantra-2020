#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry

from tf.transformations import euler_from_quaternion
import math


def control_loop():
    rospy.init_node('ebot_stopper')
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    
    rate = rospy.Rate(10)
    
    velocity_msg = Twist()
    velocity_msg.linear.x = 0
    velocity_msg.angular.z = 0
    
    n = 0
    while n < 10:
        pub.publish(velocity_msg)

        print 'bot stopped'
        rate.sleep()

        n += 1


if __name__ == '__main__':
    control_loop()