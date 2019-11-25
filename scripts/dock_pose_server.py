#!/usr/bin/env python
"""
Server for calculating offset position from cart for docking operation.
Please note that the calculation of the cart position is reliant on 
the pose published from vicon.
"""

import rospy
from geometry_msgs.msg import TransformStamped, Pose
from std_msgs.msg import Bool
from math import cos, sin
import tf_conversions
from fms_rob.srv import dockPose


'''
#######################################################################################
'''

ROBOT_ID = rospy.get_param('/ROBOT_ID') # by default the robot id is set in the package's launch file

'''
#######################################################################################
'''

goal = TransformStamped()
#pose_updated = Bool()
pose_sub = None

def get_docking_pose(req):
    global goal, pose_sub
    #pose_updated = False
    goal_result = Pose()
    cart_id = req.cart_id
    distance = req.distance
    rospy.on_shutdown(shutdown_hook)
    topic = ['/vicon/'+cart_id+'/'+cart_id, 'geometry_msgs/TransformStamped']
    if(topic not in rospy.get_published_topics('/vicon/')):
        rospy.logerr('[ {} ]: Cart Topic Not Found!'.format(rospy.get_name()))
        return
    #print('Cart id pose being calculated for: {}'.format(cart_id)) ###
    while (True):
        pose_sub = rospy.Subscriber('/vicon/'+cart_id+'/'+cart_id, TransformStamped, get_vicon_pose)
        #print('Topic to be subscribed to is: {}'.format('/vicon/'+cart_id+'/'+cart_id))
        #print('Update Flag Status is: {}'.format(pose_updated))
        #rospy.sleep(0.4)
        #if (pose_updated == True):
        #rospy.loginfo('Cart vicon topic updated')
        rospy.sleep(1) # wait for cart topic subscribtion
        rospy.loginfo('[ {} ]: Cart id Goal is {}'.format(rospy.get_name(), cart_id))
        goal_rot = [goal.transform.rotation.x, goal.transform.rotation.y, goal.transform.rotation.z, goal.transform.rotation.w]
        goal_euler = tf_conversions.transformations.euler_from_quaternion(goal_rot)
        # offset from goal to gurantee proper docking
        goal_result.position.x = goal.transform.translation.x - (distance * cos(goal_euler[2])) # distance offset from cart
        goal_result.position.y = goal.transform.translation.y - (distance * sin(goal_euler[2]))
        # get cart orientation
        goal_result.orientation.x = goal_rot[0] # same orientation as cart
        goal_result.orientation.y = goal_rot[1]
        goal_result.orientation.z = goal_rot[2]
        goal_result.orientation.w = goal_rot[3]
        rospy.loginfo('[ {} ]: Docking Pose Calculated'.format(rospy.get_name()))
        return goal_result
        #else:
        #    rospy.loginfo('Cart vicon topic Not updated!')
        #    continue

def get_vicon_pose(data):
    """ Returns the location of the cart in Vicon. """
    global goal, pose_sub
    goal = data
    #pose_updated = True
    pose_sub.unregister() #avoids previous cart id persistence

def dock_pose_server():
    s = rospy.Service('/'+ROBOT_ID+'/get_docking_pose', dockPose, get_docking_pose)
    rospy.loginfo('[ {} ]: Ready'.format(rospy.get_name()))

def shutdown_hook():
    rospy.logwarn('[ {} ]: node shutdown by user'.format(rospy.get_name()))

if __name__ == "__main__":
    rospy.init_node('dock_pose_server')
    rospy.on_shutdown(shutdown_hook)
    dock_pose_server()
    rospy.spin()
