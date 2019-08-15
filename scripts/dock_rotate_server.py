#!/usr/bin/env python

'''
Server for rotating robot while under cart 
'''

import rospy
from geometry_msgs.msg import TransformStamped, Pose
from math import cos, sin
import tf_conversions
from robotnik_msgs.srv import dockRotate
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist


'''
#######################################################################################
'''

ROBOT_ID = 'rb1_base_b'

'''
#######################################################################################
'''


odom_coor = Odometry()

def initiate_dock_rotate(req):
    rospy.Subscriber('/rb1_base_b/dummy_odom', Odometry, get_odom)
    vel_pub = rospy.Publisher('/'+ROBOT_ID+'/move_base/cmd_vel', Twist, queue_size=10)
    rospy.sleep(1)
    angle = req.angle
    vel_msg = Twist()
    speed = 0.5 
    tolerance = 0.03
    print('Rotating Cart')
    while (odom_coor.x <= angle):
        vel_msg.linear.x = 0
        vel_msg.angular.z = speed
        vel_pub.publish(vel_msg)
    vel_msg.linear.x = 0
    vel_pub.publish(vel_msg)
    if ((odom_coor.x <= angle + tolerance) and (odom_coor.x >= angle - tolerance)):
        return True
    else:
        return False

def dock_rotate_server():
    s = rospy.Service('dock_rotate', dockRotate, initiate_dock_rotate)
    print ('Dock Rotate Server Ready')

def get_odom(data):
    global odom_coor
    odom_coor = data.pose.pose.position
 

if __name__ == "__main__":
    rospy.init_node('dock_rotate_server')
    dock_rotate_server()
    rospy.spin()
