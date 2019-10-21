#!/usr/bin/env python

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseStamped


ROBOT_ID = 'rb1_base_b'

'''
#######################################################################################
'''

def action_test():
    client = actionlib.SimpleActionClient('rb1_base_b/move_base', MoveBaseAction)
    client.wait_for_server()
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "vicon_world" # Always send goals in reference to vicon_world when using ros_mocap package
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = 0
    goal.target_pose.pose.position.y = 0
    goal.target_pose.pose.orientation.w = 1.0
    client.send_goal(goal)
    wait = client.wait_for_result()
    if not wait:
        rospy.logerr("Action server not available!")
        rospy.signal_shutdown("Action server not available!")
    else:
        return client.get_result()
    

if __name__ == '__main__':
    try:
        rospy.init_node('action_test_client')
        result = action_test()
        if result:
            rospy.loginfo("Goal execution done!")
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")
