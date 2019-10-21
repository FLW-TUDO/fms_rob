#!/usr/bin/env python
'''
Server for calculating offset position from cart for docking operation.
Please note that the calculation of the cart position is reliant on 
the pose published from vicon.
'''

import rospy
from geometry_msgs.msg import TransformStamped, Pose
from math import cos, sin
import tf_conversions
from fms_rob.srv import dockPose


'''
#######################################################################################
'''

ROBOT_ID = rospy.get_param('/ROBOT_ID', 'rb1_base_b') # by default the robot id is set in the package's launch file

'''
#######################################################################################
'''

goal = TransformStamped()

def get_docking_pose(req):
    global goal
    goal_result = Pose()
    cart_id = req.cart_id
    distance = req.distance
    rospy.on_shutdown(shutdown_hook)
    topic = ['/vicon/'+cart_id+'/'+cart_id, 'geometry_msgs/TransformStamped']
    if(topic not in rospy.get_published_topics('/vicon/')):
        rospy.logerr('Cart Topic Not Found!')
        return
    rospy.Subscriber('/vicon/'+cart_id+'/'+cart_id, TransformStamped, get_vicon_pose)
    rospy.sleep(1)
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
    rospy.loginfo('Docking Pose Calculated')
    return goal_result

def get_vicon_pose(data):
    '''
    Returns the location of the cart in Vicon
    '''
    global goal
    goal = data

def dock_pose_server():
    s = rospy.Service('/'+ROBOT_ID+'/get_docking_pose', dockPose, get_docking_pose)
    rospy.loginfo('Docking Pose Calculation Ready')
 
def shutdown_hook():
    rospy.logwarn('Dock Pose Server node shutdown by user')

if __name__ == "__main__":
    rospy.init_node('dock_pose_server')
    rospy.on_shutdown(shutdown_hook)
    dock_pose_server()
    rospy.spin()
