#!/usr/bin/env python

import rospy
import actionlib
import sys
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseStamped
from fms_rob.msg import RobActionSelect, RobActionStatus
from actionlib_msgs.msg import GoalStatusArray
from std_srvs.srv import Empty


'''
#######################################################################################
'''

ROBOT_ID = 'rb1_base_b'

'''
#######################################################################################
'''

class drive_action:
    def __init__(self):
        rospy.init_node('drive_action_client')
        self.status_flag = False
        self.client = actionlib.SimpleActionClient('rb1_base_b/move_base', MoveBaseAction) 
        self.client.wait_for_server() # wait for server for each goal?
        self.action_sub = rospy.Subscriber('/'+ROBOT_ID+'/rob_action', RobActionSelect, self.drive)
        self.status_update_sub = rospy.Subscriber('/'+ROBOT_ID+'/move_base/status', GoalStatusArray, self.status_update) # status from action server - use feedback instead ?
        self.action_status_pub = rospy.Publisher('/'+ROBOT_ID+'/rob_action_status', RobActionStatus, queue_size=10)
        print('Ready for Driving')

    def drive(self, data):
        self.command_id = data.command_id # to be removed after msg modification
        self.action = data.action # to be removed after msg modification
        if (data.action == 'drive'):
            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = "vicon_world" # Always send goals in reference to vicon_world when using ros_mocap package
            goal.target_pose.header.stamp = rospy.Time.now()
            goal.target_pose.pose.position.x = data.goal.position.x
            goal.target_pose.pose.position.y = data.goal.position.y
            goal.target_pose.pose.orientation.x = data.goal.orientation.x
            goal.target_pose.pose.orientation.y = data.goal.orientation.y
            goal.target_pose.pose.orientation.z = data.goal.orientation.z
            goal.target_pose.pose.orientation.w = data.goal.orientation.w
            print('Sending Drive goal to action server: ') 
            print(goal)
            rospy.wait_for_service('/'+ROBOT_ID+'/move_base/clear_costmaps')
            reset_costmaps = rospy.ServiceProxy('/'+ROBOT_ID+'/move_base/clear_costmaps', Empty)
            reset_costmaps()
            #self.client.send_goal_and_wait(goal) # blocking
            self.client.send_goal(goal) # non-blocking
            self.status_flag = True
            '''
            wait = self.client.wait_for_result() # blocking - for this callback
            if not wait:
                rospy.logerr("Action server not available!")
                rospy.signal_shutdown("Action server not available!")
                return
            else:
                self.status_flag = False
                #self.client.stop_tracking_goal()
                return
                #return self.client.get_result()
            '''
        else:
            if (data.action == 'cancelCurrent'):
                self.client.cancel_goal()
                print('Cancelling Current Goal')
            if (data.action == 'cancelAll'):
                self.client.cancel_all_goals()
                print('cancelling All Goals')
            if (data.action == 'cancelAtAndBefore'):
                self.client.cancel_goals_at_and_before_time(data.cancellation_stamp)
                s = 'Cancelling all Goals at and before {}'.format(data.cancellation_stamp)
                print(s)
            self.client.stop_tracking_goal()
            self.goal_flstatus_flagag = False
            return

    def status_update(self, data):
        if (self.status_flag == True):
            #print(data.status_list[1].status) # All status list info are at indices 0 and 1
            status = self.client.get_state()
            print(status)
            msg = RobActionStatus()
            #self.client.stop_tracking_goal()
            msg.status = status
            msg.command_id = self.command_id # to be removed after msg modification
            msg.action = self.action # to be removed after msg modification
            self.action_status_pub.publish(msg)
            if (status == 3):
                self.client.stop_tracking_goal()
                self.status_flag = False
                return
    

if __name__ == '__main__':
    try:
        da = drive_action()
    except KeyboardInterrupt:
        sys.exit()
        print('Interrupted!')
    rospy.spin()
