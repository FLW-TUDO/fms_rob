#!/usr/bin/env python
"""
A client that requests the navigation of the robot to the original position from
which the cart was picked. It acts as a client to ROS's built in move base node,
which is an implementation of an action server.
Please note that the status message architecture follows the Goal Status Array
type specified in ROS actions by default.
"""

import rospy
import actionlib
import sys
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Pose, TransformStamped
from fms_rob.msg import RobActionSelect, RobActionStatus
from fms_rob.srv import  dockPose
from actionlib_msgs.msg import GoalStatusArray
from std_msgs.msg import String, Bool
from math import pi
from std_srvs.srv import Empty
import time
import dynamic_reconfigure.client


'''
#######################################################################################
'''

ROBOT_ID = rospy.get_param('/ROBOT_ID') # by default the robot id is set in the package's launch file

'''
#######################################################################################
'''

class ReturnAction:
    def __init__(self):
        rospy.init_node('return_action_client')
        self.status_flag = False # used to throttle further message sending after action execution
        self.act_client = actionlib.SimpleActionClient('/'+ROBOT_ID+'/move_base', MoveBaseAction) 
        rospy.loginfo('[ {} ]: Waiting for move_base server'.format(rospy.get_name()))
        err_flag = False
        if (self.act_client.wait_for_server(timeout=rospy.Duration.from_sec(5))): # wait for server start up
            #rospy.loginfo('[ {} ]: Move Base Server Running'.format(rospy.get_name()))
            pass
        else:
            rospy.logerr('[ {} ]: Timedout waiting for Move Base Server!'.format(rospy.get_name()))
            err_flag = True
        self.action_sub = rospy.Subscriber('/'+ROBOT_ID+'/rob_action', RobActionSelect, self.returns)
        self.status_update_sub = rospy.Subscriber('/'+ROBOT_ID+'/move_base/status', GoalStatusArray, self.status_update) # status from move base action server 
        self.action_status_pub = rospy.Publisher('/'+ROBOT_ID+'/rob_action_status', RobActionStatus, queue_size=10) # publishes status msgs upstream
        self.klt_num_pub = rospy.Publisher('/'+ROBOT_ID+'/klt_num', String, queue_size=10) # used for interfacing with the ros_mocap package
        self.reconf_client = dynamic_reconfigure.client.Client("dynamic_reconf_server", timeout=30) # client of fms_rob dynmaic reconfigure server
        rospy.on_shutdown(self.shutdown_hook) # used to reset the interface with the ros_mocap package
        #self.place_flag = True
        #self.dock_flag = True
        ###self.dock_flag = Bool()
        ###self.place_flag = Bool()
        rospy.sleep(1)
        if not err_flag:
            rospy.loginfo('[ {} ]: Ready'.format(rospy.get_name()))
        else:
            rospy.logerr('[ {} ]: Not Ready!'.format(rospy.get_name()))
    def returns(self, data):
        """ Executes returning action. """
        self.command_id = data.command_id
        self.action = data.action # to be removed after msg modification 
        if (data.action == 'return'):
            dock_flag = rospy.get_param('/'+ROBOT_ID+'/dynamic_reconf_server/dock')
            place_flag = rospy.get_param('/'+ROBOT_ID+'/dynamic_reconf_server/place')
            if ((place_flag == True) or (dock_flag == True)):
                #if (dock_pose == None):
                #    rospy.logerr('Cart Topic Not Found!')
                #    return
                goal = MoveBaseGoal()
                goal.target_pose.header.frame_id = "vicon_world" # Always send goals in reference to vicon_world when using ros_mocap package
                goal.target_pose.header.stamp = rospy.Time.now()
                '''
                goal.target_pose.pose.position.x = self.return_pose['trans_x']
                goal.target_pose.pose.position.y = self.return_pose['trans_y']
                goal.target_pose.pose.orientation.x = self.return_pose['rot_x']
                goal.target_pose.pose.orientation.y = self.return_pose['rot_y']
                goal.target_pose.pose.orientation.z = self.return_pose['rot_z']
                goal.target_pose.pose.orientation.w = self.return_pose['rot_w']
                '''
                goal.target_pose.pose.position.x = rospy.get_param('/'+ROBOT_ID+'/dynamic_reconf_server/return_pose_trans_x')
                goal.target_pose.pose.position.y = rospy.get_param('/'+ROBOT_ID+'/dynamic_reconf_server/return_pose_trans_y')
                goal.target_pose.pose.orientation.x = rospy.get_param('/'+ROBOT_ID+'/dynamic_reconf_server/return_pose_rot_x')
                goal.target_pose.pose.orientation.y = rospy.get_param('/'+ROBOT_ID+'/dynamic_reconf_server/return_pose_rot_y')
                goal.target_pose.pose.orientation.z = rospy.get_param('/'+ROBOT_ID+'/dynamic_reconf_server/return_pose_rot_z')
                goal.target_pose.pose.orientation.w = rospy.get_param('/'+ROBOT_ID+'/dynamic_reconf_server/return_pose_rot_w')
                rospy.loginfo('[ {} ]: Sending Return goal to action server'.format(rospy.get_name())) 
                #rospy.loginfo('Return goal coordinates: {}'.format(goal))
                try:
                    rospy.wait_for_service('/'+ROBOT_ID+'/move_base/clear_costmaps') # clear cost maps before sending goal to remove false positive obstacles
                    reset_costmaps = rospy.ServiceProxy('/'+ROBOT_ID+'/move_base/clear_costmaps', Empty)
                    reset_costmaps()
                    rospy.loginfo('[ {} ]: Costmaps Cleared Successfully'.format(rospy.get_name())) 
                except:
                    rospy.logwarn('[ {} ]: Costmaps Clearing Service Call Failed!'.format(rospy.get_name())) 
                rospy.sleep(0.5)
                #self.act_client.send_goal_and_wait(goal) # blocking
                self.act_client.send_goal(goal) # non-blocking
                self.status_flag = True
            else:
                #self.act_client.cancel_goal()
                rospy.logerr('[ {} ]: Action Rejected! - Invalid Return Action'.format(rospy.get_name()))
                return
        else:
            if (data.action == 'cancelCurrent'):
                self.act_client.cancel_goal()
                rospy.logwarn('Cancelling Current Goal')
                self.reconf_client.update_configuration({"place": False})
                self.reconf_client.update_configuration({"dock": False})
            if (data.action == 'cancelAll'):
                self.act_client.cancel_all_goals()
                rospy.logwarn('cancelling All Goals')
                self.reconf_client.update_configuration({"place": False})
                self.reconf_client.update_configuration({"dock": False})
            if (data.action == 'cancelAtAndBefore'):
                self.act_client.cancel_goals_at_and_before_time(data.cancellation_stamp)
                s = 'Cancelling all Goals at and before {}'.format(data.cancellation_stamp)
                rospy.logwarn(s)
                self.reconf_client.update_configuration({"place": False})
                self.reconf_client.update_configuration({"dock": False})
            #self.act_client.stop_tracking_goal()
            #self.status_flag = False
            return

    '''
    def dynamic_params_update(self, config):
        """ Dynamically Obtaining the interlock state. """
        #rospy.loginfo("Config set to {pick}, {dock}, {undock}, {place}, {home}, {return}".format(**config))
        self.place_flag = config['home']
        self.dock_flag = config['undock']
        self.return_pose = {'trans_x': config['return_pose_trans_x'], 'trans_y': config['return_pose_trans_y'], \
            'rot_x': config['return_pose_rot_x'], 'rot_y': config['return_pose_rot_y'], \
            'rot_z': config['return_pose_rot_z'], 'rot_w': config['return_pose_rot_w']}
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
            #msg.cart_id = self.cart_id
            self.action_status_pub.publish(msg)
            if (status == 3): # if action execution is successful
                rospy.loginfo('[ {} ]: Return Action Successful'.format(rospy.get_name()))
                print('--------------------------------')
                self.reconf_client.update_configuration({"return": True})
                self.reconf_client.update_configuration({"place": False})
                self.reconf_client.update_configuration({"dock": False})
                self.act_client.stop_tracking_goal()
                self.status_flag = False
                return
            if (status == 4): # if action execution is aborted
                #self.reconf_client.update_configuration({"place": False})
                #self.reconf_client.update_configuration({"dock": False})
                #self.act_client.stop_tracking_goal()
                self.status_flag = False
                rospy.logerr('[ {} ]: Execution Aborted by Move Base Server!'.format(rospy.get_name()))
                try:
                    rospy.wait_for_service('/'+ROBOT_ID+'/move_base/clear_costmaps') # clear cost maps before sending goal to remove false positive obstacles
                    reset_costmaps = rospy.ServiceProxy('/'+ROBOT_ID+'/move_base/clear_costmaps', Empty)
                    reset_costmaps()
                    rospy.loginfo('[ {} ]: Costmaps Cleared Successfully'.format(rospy.get_name())) 
                except:
                    rospy.logwarn('[ {} ]: Costmaps Clearing Service Call Failed!'.format(rospy.get_name())) 
                rospy.sleep(1)
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
        ra = ReturnAction()
    except KeyboardInterrupt:
        sys.exit()
        #rospy.logerr('Interrupted!')
    rospy.spin()
