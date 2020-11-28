#!/usr/bin/env python
from os import system

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry

from tf.transformations import euler_from_quaternion
import math


class Waypoint:
    '''
        Holds spatial data of goal locations
    '''
    def __init__(self, x=0, y=0, theta=0):
        self.x = x
        self.y = y
        self.theta = theta
    

    def __str__(self):
        return str({
            'x': self.x,
            'y': self.y,
            'theta': self.theta
        })
    

class Error:
    '''
        Error data model
    '''
    def __init__(self, x_error=0, y_error=0, angular_error=0):
        self.x_error = x_error
        self.y_error = y_error
        self.angular_error = angular_error
    
    def __str__(self):
        return str({
            'x_error: ': self.x_error,
            'y_error: ': self.y_error,
            'angular_error: ': self.angular_error
        })

class Bot:
    '''
        Geospatial data of the robot
    '''
    def __init__(self, x=0, y=0, theta=0, twist=Twist()):
        self.x = x
        self.y = y
        self.theta = theta
        self.twist = twist


    def __str__(self):
        return str({
            'x': self.x,
            'y': self.y,
            'theta': self.theta,
            'twist': self.twist
        })


# current position information of the robot 
pose = Bot()

# limits for dividing the laser scan readings into sectors
regions = {
    'bright': 0,
    'fright': 0,
    'front': 0,
    'fleft': 0,
    'bleft': 0,
}

# step value for sampling the objective path
waypoint_angle_step = (float(2) / 100) * math.pi

# final waypoint for tracing the path
trace_path_final_waypoint = Waypoint(2 * math.pi, 2 * (math.sin(2 * math.pi)) * (math.sin(math.pi)), 2 * math.pi)

# final waypoint of the task
final_waypoint = Waypoint(12.5, 0, 0)

# proximity threshold for the robot to redirect before hitting an obstacle
proximity_threshold = 1


def get_waypoint(current_waypoint):
    '''
        Return waypoint(intermediate goals) for the robot
    '''

    global trace_path_final_waypoint
    global waypoint_angle_step

    # initialise with current waypoint
    goal_waypoint = current_waypoint

    if pose.x <= 2 * math.pi:
        if pose.x > current_waypoint.x:
            print '*****************************************************waypoint covered*******************************************************'
            updated_waypoint_angle = current_waypoint.x + waypoint_angle_step
            if updated_waypoint_angle > trace_path_final_waypoint.theta:
                updated_waypoint_angle = trace_path_final_waypoint.theta

            print 'step: {}, current angle: {}, updated angle: {}'.format(waypoint_angle_step, current_waypoint.theta, updated_waypoint_angle)
            goal_waypoint = Waypoint(
                updated_waypoint_angle,
                2 * (math.sin(updated_waypoint_angle) * math.sin(updated_waypoint_angle / 2)),
                updated_waypoint_angle
            )
            print 'updated waypoint: {}'.format(goal_waypoint)
    else:
        goal_waypoint = Waypoint(12.5, 0, 0)

    return goal_waypoint


def laser_callback(msg):
    '''
        Callback for laser scanner subscriber
    '''
    global regions
    regions = {
        'bright': min(msg.ranges[0:120]),
        'fright': min(msg.ranges[120:240]),
        'front': min(msg.ranges[240:480]),
        'fleft': min(msg.ranges[480:600]),
        'bleft': min(msg.ranges[600:720]),
    }


def odom_callback(data):
    '''
        Callback function for the odometer subscriber of the robot
    '''
    global pose

    x  = data.pose.pose.orientation.x
    y  = data.pose.pose.orientation.y
    z = data.pose.pose.orientation.z
    w = data.pose.pose.orientation.w

    pose = Bot(
        data.pose.pose.position.x, 
        data.pose.pose.position.y, 
        euler_from_quaternion([x,y,z,w])[2],
        Twist()
    )

    pose.twist.linear.x = data.twist.twist.linear.x
    pose.twist.angular.z = data.twist.twist.angular.z


def calculate_error(waypoint):
    '''
        Calculate the error of the position of the robot
    '''
    error = Error()
    error.y_error = round(waypoint.y - pose.y, 2)
    error.x_error = round(waypoint.x - pose.x, 2)
    theta_goal = math.atan2(error.y_error, error.x_error)
    error.angular_error = pose.theta - theta_goal

    return error


def is_waypoint_reached(error):
    if(error.x_error <= 0.05 and error.y_error <= 0.05):
        return True
    else:
        return False


def move_to_waypoint(waypoint, publisher, linear_speed, proportional):
    '''
        Move to a waypoint from the current position of the robot
    '''
    global pose 
    global waypoint_angle_step

    # calculate the error
    error = calculate_error(waypoint)

    # construct the velocity messaege to publish
    velocity_msg = Twist()
    velocity_msg.linear.x = linear_speed
    velocity_msg.angular.z = -1 * proportional * error.angular_error

    # publish the message
    publisher.publish(velocity_msg)
    print 'Command {}'.format(velocity_msg)
    print 'Current waypoint: {} counter: {}'.format(waypoint, waypoint.theta / waypoint_angle_step)
    print 'pose data: {}'.format(pose)
    print 'x_error: {}, y_error: {}, angular_error: {}'.format(error.x_error, error.y_error, error.angular_error)
    print("Controller message pushed at {}".format(rospy.get_time()))


def trace_path():
    '''
        Trace the path 2(sin(x))(sin(x/2)) from x = 0 to 2pi
    '''
    # use global variables 
    global pose
    global trace_path_final_waypoint
    
    current_waypoint = Waypoint()
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    

    rate = rospy.Rate(10) 

    velocity_msg = Twist()
    velocity_msg.linear.x = 0
    velocity_msg.angular.z = 0
    pub.publish(velocity_msg)

    while not rospy.is_shutdown() and (pose.x <= trace_path_final_waypoint.x):
        current_waypoint = get_waypoint(current_waypoint)
        move_to_waypoint(current_waypoint, pub, 0.2, 0.85)
    	rate.sleep()

    while round(pose.twist.linear.x, 2) != 0 and round(pose.twist.angular.z, 2) != 0:
        print 'stopping the robot after tracing the path'
        velocity_msg.linear.x = 0
        velocity_msg.angular.z = 0
        pub.publish(velocity_msg)


def go_to_goal():
    '''
        Go to a final goal waypoint
    '''
    # use global variables 
    global pose
    global regions
    global final_waypoint
    
    current_waypoint = Waypoint(pose.x, pose.y, pose.theta)

    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    rate = rospy.Rate(10) 

    velocity_msg = Twist()
    velocity_msg.linear.x = 0
    velocity_msg.angular.z = 0
    pub.publish(velocity_msg)

    while not rospy.is_shutdown() and ((pose.x <= final_waypoint.x) or (pose.y <= final_waypoint.y)):
        if regions['front'] <= proximity_threshold:
            print 'obstacle front: {}'.format(regions['front'])
            current_waypoint.x = pose.x - regions['front'] - proximity_threshold
            current_waypoint.y = pose.y + proximity_threshold
            proportional = 1.2
        elif regions['bright'] <= proximity_threshold:
            print 'obstacle on right back: {}'.format(regions['bright'])
            current_waypoint.x = pose.x - proximity_threshold - regions['bright']
            current_waypoint.y = pose.y + proximity_threshold
            proportional = 1.2
        elif regions['fright'] <= proximity_threshold:
            print 'obstacle right front: {}'.format(regions['fright'])
            current_waypoint.x = pose.x - proximity_threshold - regions['fright']
            current_waypoint.y = pose.y + proximity_threshold
            proportional = 1.2
        else:
            print('navigating to final goal')
            current_waypoint = get_waypoint(current_waypoint)
            proportional = 0.85

        move_to_waypoint(current_waypoint, pub, 0.2, proportional)
    	rate.sleep()
    
    while round(pose.twist.linear.x, 2) != 0 and round(pose.twist.angular.z, 2) != 0:
        print 'stopping the robot after reaching the goal'
        print 'pose data: {}'.format(pose)
        velocity_msg.linear.x = 0
        velocity_msg.angular.z = 0
        pub.publish(velocity_msg)


def control_loop():
    '''
        Full control loop of the robot
    '''

    rospy.init_node('ebot_controller')
    rospy.Subscriber('/ebot/laser/scan', LaserScan, laser_callback)
    rospy.Subscriber('/odom', Odometry, odom_callback)

    trace_path()
    go_to_goal()


if __name__ == '__main__':
    control_loop()