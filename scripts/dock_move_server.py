#!/usr/bin/env python

'''
Server for moving robot under Cart 
'''

import rospy
from geometry_msgs.msg import TransformStamped, Pose
from math import cos, sin
import tf_conversions
from robotnik_msgs.srv import dockMove
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

def initiate_dock_move(req):
    rospy.Subscriber('/'+ROBOT_ID+'/dummy_odom', Odometry, get_odom)
    vel_pub = rospy.Publisher('/'+ROBOT_ID+'/move_base/cmd_vel', Twist, queue_size=10)
    rospy.sleep(1)
    distance = req.distance
    vel_msg = Twist()
    speed = 0.2
    tolerance = 0.03
    print('Moving under Cart')
    while (odom_coor.x <= distance):
        vel_msg.linear.x = speed
        vel_msg.angular.z = 0
        vel_pub.publish(vel_msg)
    vel_msg.linear.x = 0
    vel_pub.publish(vel_msg)
    if ((odom_coor.x <= distance + tolerance) and (odom_coor.x >= distance - tolerance)):
        return True
    else:
        return False

def dock_move_server():
    s = rospy.Service('/'+ROBOT_ID+'/dock_move', dockMove, initiate_dock_move)
    print ('Dock Move Server Ready')

def get_odom(data):
    global odom_coor
    odom_coor = data.pose.pose.position
 

if __name__ == "__main__":
    rospy.init_node('dock_move_server')
    dock_move_server()
    rospy.spin()
