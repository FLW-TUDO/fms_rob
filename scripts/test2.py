#!/usr/bin/env python

import rospy
from geometry_msgs.msg import TransformStamped
from math import cos, sin, pi
import tf_conversions


class test:
    def __init__(self):  
        rospy.init_node('test2')
        ROBOT_ID = 'rb1_base_b'
        cart_id = 'KLT_3_neu'
        #rospy.Subscriber('/vicon/'+ROBOT_ID+'/'+ROBOT_ID, TransformStamped, self.robot_pose)
        rospy.Subscriber('/vicon/'+cart_id+'/'+cart_id, TransformStamped, self.cart_pose)
        rospy.sleep(2)

    def robot_pose(self, data):
        """ Robot vicon pose update. """
        #robot_theta_pose_trans_x = data.transform.translation.x
        #robot_theta_pose_trans_y = data.transform.translation.y
        rot=[data.transform.rotation.x, data.transform.rotation.y, data.transform.rotation.z, data.transform.rotation.w]
        rot_euler = tf_conversions.transformations.euler_from_quaternion(rot)
        robot_theta = rot_euler[2]
        #robot_theta = (self.mapping(robot_theta)) / (2*pi)
        print('Robot Theta: {}'.format(robot_theta))
        #rospy.sleep(0.2)

    def cart_pose(self, data):
        """ Robot vicon pose update. """
        #cart_theta_pose_trans_x = data.transform.translation.x
        #cart_theta_pose_trans_y = data.transform.translation.y
        rot=[data.transform.rotation.x, data.transform.rotation.y, data.transform.rotation.z, data.transform.rotation.w]
        rot_euler = tf_conversions.transformations.euler_from_quaternion(rot)
        cart_theta = rot_euler[2]
        #cart_theta = (self.mapping(cart_theta))
        print('Cart Theta: {}'.format(cart_theta))
        #rospy.sleep(0.2)

    def mapping(self, value, leftMin=-pi, leftMax=pi, rightMin=0, rightMax=2*pi):
        # Figure out how 'wide' each range is
        leftSpan = leftMax - leftMin
        rightSpan = rightMax - rightMin
        # Convert the left range into a 0-1 range (float)
        valueScaled = float(value - leftMin) / float(leftSpan)
        # Convert the 0-1 range into a value in the right range.
        return rightMin + (valueScaled * rightSpan) 


if __name__ == '__main__':
    test()
    rospy.spin()