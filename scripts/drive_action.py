#!/usr/bin/env python

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseStamped
from robotnik_msgs.msg import RobActionSelect


'''
#######################################################################################
'''

ROBOT_ID = 'rb1_base_b'

'''
#######################################################################################
'''

class drive_action:
    def __init__(self):
        rospy.init_node('drive_action')
        self.client = actionlib.SimpleActionClient('rb1_base_b/move_base', MoveBaseAction) 
        self.client.wait_for_server() # wait for server for each goal?
        self.action_sub = rospy.Subscriber('/'+ROBOT_ID+'/rob_action', RobActionSelect, self.drive)

    def drive(self, data):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "vicon_world" # Always send goals in reference to vicon_world when using ros_mocap package
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = data.goal.position.x
        goal.target_pose.pose.position.y = data.goal.position.y
        goal.target_pose.pose.orientation.x = data.goal.orientation.x
        goal.target_pose.pose.orientation.y = data.goal.orientation.y
        goal.target_pose.pose.orientation.z = data.goal.orientation.z
        goal.target_pose.pose.orientation.w = data.goal.orientation.w
        print('Sending goal to action server: ') 
        print(goal)
        self.client.send_goal(goal)
        wait = self.client.wait_for_result()
        if not wait:
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available!")
        else:
            return self.client.get_result()
    

if __name__ == '__main__':
    try:
        dr = drive_action()
    except KeyboardInterrupt:
        sys.exit()
        print('Interrupted!')
    rospy.spin()
