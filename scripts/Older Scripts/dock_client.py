#!/usr/bin/env python

import rospy
import actionlib
import sys
from fms_rob.msg import dockAction, dockGoal
from geometry_msgs.msg import PoseStamped
from fms_rob.msg import RobActionSelect, RobActionStatus
from actionlib_msgs.msg import GoalStatusArray
from std_srvs.srv import Empty
from math import pi


'''
#######################################################################################
'''

ROBOT_ID = 'rb1_base_b'

'''
#######################################################################################
'''

class dock_action_client:
    def __init__(self):
        self.status_flag = False
        self.client = actionlib.SimpleActionClient('do_dock', dockAction) 
        self.client.wait_for_server() # wait for server for each goal?
        self.action_sub = rospy.Subscriber('/'+ROBOT_ID+'/rob_action', RobActionSelect, self.drive)
        self.status_update_sub = rospy.Subscriber('/'+ROBOT_ID+'/move_base/status', GoalStatusArray, self.status_update) # status from action server - use feedback instead ?
        self.action_status_pub = rospy.Publisher('/'+ROBOT_ID+'/rob_action_status', RobActionStatus, queue_size=10)
        print('Ready for Docking')

    def drive(self, data):
        self.command_id = data.command_id # to be removed after msg modification
        self.action = data.action # to be removed after msg modification
        if (data.action == 'dock'):
            goal = dockGoal()
            print('Sending Dock goal to action server: ') 
            goal.distance = 0.5
            goal.angle = pi
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
        rospy.init_node('dock_client')
        dc = dock_action_client()    
    except KeyboardInterrupt:
        sys.exit()
        print('Interrupted!')
    rospy.spin()
