#!/usr/bin/env python

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseWithCovarianceStamped, Pose
from tf.transformations import quaternion_from_euler
import dynamic_reconfigure.client
import math


pose = PoseWithCovarianceStamped()
min_obstacle_dist = float('inf')


class Waypoint:
    def __init__(self, x = 0, y = 0, theta = 0):
        self.x = x
        self.y = y
        self.theta = theta
    
    def __str__(self):
        return str({
            'x': self.x,
            'y': self.y,
            'theta': self.theta
        })


def laser_callback(data):
    global min_obstacle_dist

    min_obstacle_dist = min(data.ranges)


config_client = dynamic_reconfigure.client.Client('/move_base/DWAPlannerROS')


def set_reverse_vel(value = 0):
    global config_client

    config_client.update_configuration({
        'min_vel_x': value
    })


def fb_callback(data):
    global min_obstacle_dist

    distance_to_goal = math.sqrt(
        ((data.base_position.pose.position.x - curr_target.x) ** 2) +
        ((data.base_position.pose.position.y - curr_target.y) ** 2)
    )

    if min_obstacle_dist <= 1 or distance_to_goal <= 1:
        set_reverse_vel(-0.6)
    else:
        set_reverse_vel(0)



def go_to_waypoint(goal_x = 0, goal_y = 0, goal_theta = 0):
    # Create an action client called "move_base" with action definition file "MoveBaseAction"
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)

   # Waits until the action server has started up and started listening for goals.
    client.wait_for_server()

   # Creates a new goal with the MoveBaseGoal constructor
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
   # Move 0.5 meters forward along the x axis of the "map" coordinate frame
    goal.target_pose.pose.position.x = goal_x
    goal.target_pose.pose.position.y = goal_y
    goal.target_pose.pose.position.z = 0
   # No rotation of the mobile base frame w.r.t. map frame
    angle_in_quat = quaternion_from_euler(0, 0, goal_theta)
    goal.target_pose.pose.orientation.x = angle_in_quat[0]
    goal.target_pose.pose.orientation.y = angle_in_quat[1]
    goal.target_pose.pose.orientation.z = angle_in_quat[2]
    goal.target_pose.pose.orientation.w = angle_in_quat[3]

   # Sends the goal to the action server.
    client.send_goal(goal, feedback_cb=fb_callback)
   # Waits for the server to finish performing the action.
    wait = client.wait_for_result()
   # If the result doesn't arrive, assume the Server is not available
    if not wait:
        rospy.logerr("Action server not available!")
        rospy.signal_shutdown("Action server not available!")
    else:
        # Result of executing the action
        return client.get_result()


def amcl_pose_callback(data):
    global pose
    pose = data


target_list = [
        Waypoint(-2, 4, 0),
        Waypoint(18.2, -1.4, 3.14),
        Waypoint(12.6, -1.7, 1.5),
        Waypoint(10.7, 10.5, 0),
        Waypoint(-9.1, -1.2, 0)
    ]

curr_target = None


if __name__ == '__main__':
    global pose
    global target_list
    global curr_target

    try:
        amcl_pose = rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, amcl_pose_callback)
        laser_sensor = rospy.Subscriber('/ebot/laser/scan', LaserScan, laser_callback)
        rospy.init_node('ebot_nav')
        
        while len(target_list) > 0:
            curr_target = target_list.pop()
            print 'sending the target: ', curr_target
            result = go_to_waypoint(curr_target.x, curr_target.y, curr_target.theta)

            if result:
                rospy.loginfo("Goal execution done!")
                print result
                print 'CURRENT LOCATION: ', pose
                rospy.sleep(1)

    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")