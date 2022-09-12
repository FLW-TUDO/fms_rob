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
from std_msgs.msg import String, Bool
import dynamic_reconfigure.client


'''
#######################################################################################
'''

ROBOT_ID = rospy.get_param('/ROBOT_ID') # by default the robot id is set in the package's launch file

'''
#######################################################################################
'''

class DUActionClient:
    
    def __init__(self):
        rospy.init_node('dock_undock_client')
        self.status_flag = False # used to throttle further message sending after action execution
        self.act_client = actionlib.SimpleActionClient('do_dock_undock', dockUndockAction) 
        err_flag = False
        if (self.act_client.wait_for_server(timeout=rospy.Duration.from_sec(5))): # wait for server start up
            #rospy.loginfo('[ {} ]: Move Base Server Running'.format(rospy.get_name()))
            pass
        else:
            rospy.logerr('[ {} ]: Timedout waiting for Dock Undock Server!'.format(rospy.get_name()))
            err_flag = True        
        self.act_client.wait_for_server() # wait for server start up
        self.action_sub = rospy.Subscriber('/'+ROBOT_ID+'/rob_action', RobActionSelect, self.dock)
        self.status_update_sub = rospy.Subscriber('/'+ROBOT_ID+'/do_dock_undock/status', GoalStatusArray, self.status_update) # status from dock_undock action server 
        self.action_status_pub = rospy.Publisher('/'+ROBOT_ID+'/rob_action_status', RobActionStatus, queue_size=10) # publishes status msgs upstream
        self.klt_num_pub = rospy.Publisher('/'+ROBOT_ID+'/klt_num', String, queue_size=10) # used for interfacing with the ros_mocap package
        rospy.on_shutdown(self.shutdown_hook) # used to reset the interface with the ros_mocap package
        self.reconf_client = dynamic_reconfigure.client.Client("dynamic_reconf_server", timeout=30) # client of fms_rob dynmaic reconfigure server
        #self.pick_flag = Bool()
        #self.return_flag = Bool()
        #self.pick_flag = True
        #self.return_flag = True
        rospy.sleep(1)
        if not err_flag:
            rospy.loginfo('[ {} ]: Ready'.format(rospy.get_name()))
        else:
            rospy.logerr('[ {} ]: Not Ready!'.format(rospy.get_name()))

    def dock(self, data):
        """ Executes the dock or undock action. """
        if (data.action == 'dock'):
            self.command_id = data.command_id # Note: command id is updated only when the action is chosen and not for all sent actions
            self.action = data.action
            self.direction = data.direction
            cart_id = data.cart_id
            print(cart_id)
            goal = dockUndockGoal()
            pick_flag = rospy.get_param('/'+ROBOT_ID+'/dynamic_reconf_server/pick')
            if (pick_flag == True):
                status = self.act_client.get_state()
                if (status != 1):
                    goal.distance = rospy.get_param('/'+ROBOT_ID+'/fms_rob/dock_distance', 1.0) # get specified dock distance specified during picking. Default: 1.0
                    goal.angle = pi
                    goal.mode = True # True --> Dock // False --> Undock
                    goal.direction = self.direction
                    goal.cart_id = cart_id
                    rospy.loginfo('[ {} ]: Sending Dock goal to action server'.format(rospy.get_name())) 
                    print(goal)
                    #self.act_client.send_goal_and_wait(goal) # blocking
                    self.act_client.send_goal(goal) # non-blocking
                    self.status_flag = True
                else:
                    rospy.logerr('[ {} ]: Dock Action Rejected! - Already processing a docking operation'.format(rospy.get_name()))
                    return
            else:
                rospy.logerr('[ {} ]: Action Rejected! - Invalid Dock Action'.format(rospy.get_name()))
                return
        elif (data.action == 'undock'):
            self.command_id = data.command_id
            self.action = data.action
            self.direction = data.direction
            goal = dockUndockGoal()
            return_flag = rospy.get_param('/'+ROBOT_ID+'/dynamic_reconf_server/return')
            if(return_flag == True):
                status = self.act_client.get_state()
                if (status != 1):
                    goal.distance = 0.5 # fixed distance when robot moves out from under cart
                    goal.angle = pi
                    goal.mode = False # True --> Dock // False --> Undock
                    goal.direction = self.direction
                    rospy.loginfo('[ {} ]: Sending Undock goal to action server'.format(rospy.get_name())) 
                    #self.act_client.send_goal_and_wait(goal) # blocking - Cancellations Not possible
                    self.act_client.send_goal(goal) # non-blocking
                    self.status_flag = True
                else:
                    rospy.logerr('[ {} ]: Undock Action Rejected! - Already processing an undocking operation'.format(rospy.get_name()))
                    return
            else:
                rospy.logerr('[ {} ]: Action Rejected! - Invalid Undock Action'.format(rospy.get_name()))
                return 
        else:
            if (data.action == 'cancelCurrent'):
                self.act_client.cancel_goal()
                rospy.logwarn('Cancelling Current Goal')
                self.reconf_client.update_configuration({"pick": False})
                self.reconf_client.update_configuration({"return": False})
            if (data.action == 'cancelAll'):
                self.act_client.cancel_all_goals()
                rospy.logwarn('cancelling All Goals')
                self.reconf_client.update_configuration({"pick": False})
                self.reconf_client.update_configuration({"return": False})
            if (data.action == 'cancelAtAndBefore'):
                self.act_client.cancel_goals_at_and_before_time(data.cancellation_stamp)
                s = 'Cancelling all Goals at and before {}'.format(data.cancellation_stamp)
                rospy.logwarn(s)
                self.reconf_client.update_configuration({"pick": False})
                self.reconf_client.update_configuration({"return": False})
            #self.act_client.stop_tracking_goal()
            #self.status_flag = False
            return
    '''
    def dynamic_params_update(self, config):
        """ Dynamically Obtaining the interlock state. """
        #rospy.loginfo("Config set to {pick}, {dock}, {undock}, {place}, {home}, {return}".format(**config))
        self.pick_flag = config['pick']
        self.return_flag = config['return']
        rospy.loginfo('Parameters updated by dock client') ###
    '''
        
    def status_update(self, data):
        """ Forwarding status messages upstream. """
        if (self.status_flag == True):
            #print(data.status_list[1].status) # All status list info are at indices 0 and 1
            status = self.act_client.get_state()
            #print(status)
            rospy.loginfo('[ {} ] >>> Status: {} '.format(rospy.get_name(), status))
            msg = RobActionStatus()
            #self.act_client.stop_tracking_goal()
            msg.status = status
            msg.command_id = self.command_id
            msg.action = self.action # to be removed after msg modification
            self.action_status_pub.publish(msg)
            if (status == 3): # if action execution is successful
                if(msg.action == 'dock'):
                    rospy.loginfo('[ {} ]: Dock Action Successful'.format(rospy.get_name()))
                    print('--------------------------------')
                    self.reconf_client.update_configuration({"dock": True})
                    self.reconf_client.update_configuration({"pick": False})
                    self.reconf_client.update_configuration({"undock": False})
                    self.reconf_client.update_configuration({"home": False})
                else:
                    rospy.loginfo('[ {} ]: Undock Action Successful'.format(rospy.get_name()))
                    print('--------------------------------')
                    self.reconf_client.update_configuration({"undock": True})
                    self.reconf_client.update_configuration({"return": False})
                self.act_client.stop_tracking_goal()
                self.status_flag = False
                return
            if (status == 4): # if action execution is aborted
                self.reconf_client.update_configuration({"pick": False})
                self.reconf_client.update_configuration({"return": False})
                #self.act_client.stop_tracking_goal()
                self.status_flag = False
                rospy.logerr('[ {} ]: Execution Aborted by Dock-Undock Server!'.format(rospy.get_name()))
            if (status == 2): # if action execution is preempted
                #self.reconf_client.update_configuration({"dock": False})
                #self.act_client.stop_tracking_goal()
                self.status_flag = False
                rospy.logwarn('[ {} ]: Execution Preempted by user!'.format(rospy.get_name())) 
    
    def shutdown_hook(self):
        self.klt_num_pub.publish('') # resets the picked up cart number in the ros_mocap package
        self.act_client.cancel_all_goals()
        rospy.logwarn('[ {} ]: node shutdown by user'.format(rospy.get_name()))

if __name__ == '__main__':
    try:
        dc = DUActionClient()    
    except KeyboardInterrupt:
        sys.exit()
        #rospy.logerr('Interrupted!')
    rospy.spin()
