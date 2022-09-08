#!/usr/bin/env python
"""
A client to request following a provided list of waypoints. 
Please note that the status message architecture follows the Goal Status Array 
type specified in ROS actions by default.
"""

import rospy
import actionlib
import sys
from fms_rob.msg import followWaypointsAction, followWaypointsGoal
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

class FollowWPActionClient:
    
    def __init__(self):
        rospy.init_node('follow_waypoints_client')
        self.status_flag = False # used to throttle further message sending after action execution
        self.act_client = actionlib.SimpleActionClient('do_follow_waypoints', followWaypointsAction) 
        err_flag = False
        if (self.act_client.wait_for_server(timeout=rospy.Duration.from_sec(5))): # wait for server start up
            #rospy.loginfo('[ {} ]: Move Base Server Running'.format(rospy.get_name()))
            pass
        else:
            rospy.logerr('[ {} ]: Timedout waiting for Follow Waypoints Server!'.format(rospy.get_name()))
            err_flag = True        
        self.act_client.wait_for_server() # wait for server start up
        self.action_sub = rospy.Subscriber('/'+ROBOT_ID+'/rob_action', RobActionSelect, self.followWP)
        self.status_update_sub = rospy.Subscriber('/'+ROBOT_ID+'/do_follow_waypoints/status', GoalStatusArray, self.status_update) # status from dock_undock action server 
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

    def followWP(self, data):
        """ Executes the follow waypoints """
        if (data.action == 'pick' or data.action == 'place' or data.action == 'return' or data.action == 'home'):
            self.command_id = data.command_id # Note: command id is updated only when the action is chosen and not for all sent actions
            self.action = data.action
            self.cart_id= data.cart_id
            self.station_id = data.station_id
            waypoints = data.waypoints
            goal = followWaypointsGoal()
            pick_flag = rospy.get_param('/'+ROBOT_ID+'/dynamic_reconf_server/pick')
            place_flag = rospy.get_param('/'+ROBOT_ID+'/dynamic_reconf_server/place')
            dock_flag = rospy.get_param('/'+ROBOT_ID+'/dynamic_reconf_server/dock')
            undock_flag = rospy.get_param('/'+ROBOT_ID+'/dynamic_reconf_server/undock')
            return_flag = rospy.get_param('/'+ROBOT_ID+'/dynamic_reconf_server/return')
            home_flag = rospy.get_param('/'+ROBOT_ID+'/dynamic_reconf_server/home')

            # if (self.action == 'pick' and (home_flag or undock_flag)) or:
                # status = self.act_client.get_state()
                # if (status != 1):
            goal.waypoints = waypoints
            rospy.loginfo('[ {} ]: Sending waypoints list to action server'.format(rospy.get_name())) 
            self.act_client.send_goal(goal) # non-blocking
            self.status_flag = True
                # else:
                #     rospy.logerr('[ {} ]: Dock Action Rejected! - Already processing a docking operation'.format(rospy.get_name()))
                # return
            # else:
            #     rospy.logerr('[ {} ]: Action Rejected! - Invalid Dock Action'.format(rospy.get_name()))
            #     return
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
            msg.cart_id = self.cart_id
            msg.station_id = self.station_id
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
        fwp = FollowWPActionClient()    
    except KeyboardInterrupt:
        sys.exit()
        #rospy.logerr('Interrupted!')
    rospy.spin()
