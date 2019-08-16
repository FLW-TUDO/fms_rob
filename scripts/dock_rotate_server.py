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
import tf_conversions


'''
#######################################################################################
'''

ROBOT_ID = 'rb1_base_b'

'''
#######################################################################################
'''


odom_coor = Odometry()

def initiate_dock_rotate(req):
    rospy.Subscriber('/'+ROBOT_ID+'/dummy_odom', Odometry, get_odom)
    vel_pub = rospy.Publisher('/'+ROBOT_ID+'/move_base/cmd_vel', Twist, queue_size=10)
    rospy.sleep(1)
    angle = req.angle
    angle_quat = tf_conversions.transformations.quaternion_from_euler(0, 0, angle)
    print(angle_quat)
    vel_msg = Twist()
    speed = 0.6 
    tolerance = 0.001
    print('Rotating Cart')
    while not ((odom_coor.z <=  angle_quat[2] + tolerance) and (odom_coor.z >=  angle_quat[2] - tolerance)):
        vel_msg.linear.x = 0
        vel_msg.angular.z = speed
        vel_pub.publish(vel_msg)
    vel_msg.angular.z = 0
    vel_pub.publish(vel_msg)
    return True


def dock_rotate_server():
    s = rospy.Service('/'+ROBOT_ID+'/dock_rotate', dockRotate, initiate_dock_rotate)
    print ('Dock Rotate Server Ready')

def get_odom(data):
    global odom_coor
    odom_coor = data.pose.pose.orientation
 

if __name__ == "__main__":
    rospy.init_node('dock_rotate_server')
    dock_rotate_server()
    rospy.spin()
