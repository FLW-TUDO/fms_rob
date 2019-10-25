#!/usr/bin/env python
"""
A client to requests both docking and undocking operation. 
Please note that the status message architecture follows the Goal Status Array 
type specified in ROS actions by default.
"""

import rospy
import actionlib
import sys
from fms_rob.msg import dockUndockAction, dockUndockGoal
from geometry_msgs.msg import PoseStamped
from fms_rob.msg import RobActionSelect, RobActionStatus
from actionlib_msgs.msg import GoalStatusArray
from std_srvs.srv import Empty
from math import pi
from std_msgs.msg import String


'''
#######################################################################################
'''

ROBOT_ID = rospy.get_param('/ROBOT_ID', 'rb1_base_b') # by default the robot id is set in the package's launch file

'''
#######################################################################################
'''

class du_action_client:
    
    def __init__(self):
        self.status_flag = False # used to throttle further message sending after action execution
        self.client = actionlib.SimpleActionClient('do_dock_undock', dockUndockAction) 
        rospy.loginfo('Waiting for dock_undock_server')
        self.client.wait_for_server() # wait for server start up
        self.action_sub = rospy.Subscriber('/'+ROBOT_ID+'/rob_action', RobActionSelect, self.dock)
        self.status_update_sub = rospy.Subscriber('/'+ROBOT_ID+'/do_dock_undock/status', GoalStatusArray, self.status_update) # status from dock_undock action server 
        self.action_status_pub = rospy.Publisher('/'+ROBOT_ID+'/rob_action_status', RobActionStatus, queue_size=10) # publishes status msgs upstream
        self.klt_num_pub = rospy.Publisher('/'+ROBOT_ID+'/klt_num', String, queue_size=10) # used for interfacing with the ros_mocap package
        rospy.on_shutdown(self.shutdown_hook) # used to reset the interface with the ros_mocap package
        rospy.loginfo('Ready for Docking')

    def dock(self, data):
        """ Executes the dock or undock action. """
        self.command_id = data.command_id
        self.action = data.action # to be removed after msg modification
        goal = dockUndockGoal()
        if (data.action == 'dock'):
            rospy.loginfo('Sending Dock goal to action server') 
            goal.distance = rospy.get_param(ROBOT_ID+'/fms_rob/dock_distance', '1.0') # get specified dock distance specified during picking. Default: 1.0
            goal.angle = pi
            goal.mode = True # True --> Dock // False --> Undock
            #self.client.send_goal_and_wait(goal) # blocking
            self.client.send_goal(goal) # non-blocking
            self.status_flag = True
        elif(data.action == 'undock'):
            rospy.loginfo('Sending Undock goal to action server') 
            goal.distance = 0.5 # fixed distance when robot moves out from under cart
            goal.angle = pi
            goal.mode = False # True --> Dock // False --> Undock
            #self.client.send_goal_and_wait(goal) # blocking
            self.client.send_goal(goal) # non-blocking
            self.status_flag = True
        else:
            if (data.action == 'cancelCurrent'):
                self.client.cancel_goal()
                rospy.logwarn('Cancelling Current Goal')
            if (data.action == 'cancelAll'):
                self.client.cancel_all_goals()
                rospy.logwarn('cancelling All Goals')
            if (data.action == 'cancelAtAndBefore'):
                self.client.cancel_goals_at_and_before_time(data.cancellation_stamp)
                s = 'Cancelling all Goals at and before {}'.format(data.cancellation_stamp)
                rospy.logwarn(s)
            self.client.stop_tracking_goal()
            self.goal_flstatus_flagag = False
            return

    def status_update(self, data):
        """ Forwarding status messages upstream. """
        if (self.status_flag == True):
            #print(data.status_list[1].status) # All status list info are at indices 0 and 1
            status = self.client.get_state()
            print(status)
            msg = RobActionStatus()
            #self.client.stop_tracking_goal()
            msg.status = status
            msg.command_id = self.command_id
            msg.action = self.action # to be removed after msg modification
            self.action_status_pub.publish(msg)
            if (status == 3): # if action execution is successful
                self.client.stop_tracking_goal()
                self.status_flag = False
                return
            if (status == 4): # if action execution is aborted
                self.client.stop_tracking_goal()
                self.status_flag = False
                rospy.logerr('Execution Aborted by Server!')
    
    def shutdown_hook(self):
        self.klt_num_pub.publish('') # resets the picked up cart number in the ros_mocap package
        rospy.logwarn('Dock Undock Client node shutdown by user')

if __name__ == '__main__':
    try:
        rospy.init_node('dock_client')
        dc = du_action_client()    
    except KeyboardInterrupt:
        sys.exit()
        rospy.logerr('Interrupted!')
    rospy.spin()
