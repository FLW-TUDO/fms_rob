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
from fms_rob.msg import RobActionSelect, RobActionStatus, followWaypointsAction, followWaypointsGoal
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
WaypointMode = rospy.get_param('/Waypoint_Mode') #follow Waypoints mode selection 
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


class FollowWPReturnActionClient:
    
    def __init__(self):
        rospy.init_node('follow_waypoints_return_client')
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
        #self.wp_sub = rospy.Subscriber('/'+ROBOT_ID+'/rob_wp', Float64MultiArray, self.update_wp)
        self.klt_num_pub = rospy.Publisher('/'+ROBOT_ID+'/klt_num', String, queue_size=10) # used for interfacing with the ros_mocap package
        rospy.on_shutdown(self.shutdown_hook) # used to reset the interface with the ros_mocap package
        # self.reconf_client = dynamic_reconfigure.client.Client("dynamic_reconf_server", timeout=30) # client of fms_rob dynmaic reconfigure server
        # self.pick_flag = rospy.get_param('/'+ROBOT_ID+'/dynamic_reconf_server/pick')
        # self.place_flag = rospy.get_param('/'+ROBOT_ID+'/dynamic_reconf_server/place')
        # self.dock_flag = rospy.get_param('/'+ROBOT_ID+'/dynamic_reconf_server/dock')
        # self.undock_flag = rospy.get_param('/'+ROBOT_ID+'/dynamic_reconf_server/undock')
        # self.return_flag = rospy.get_param('/'+ROBOT_ID+'/dynamic_reconf_server/return')
        # self.home_flag = rospy.get_param('/'+ROBOT_ID+'/dynamic_reconf_server/home')
            
        # #self.pick_flag = Bool()
        # #self.return_flag = Bool()
        # #self.pick_flag = True
        # #self.return_flag = True
        # rospy.sleep(1)
        # if not err_flag:
        #     rospy.loginfo('[ {} ]: Ready'.format(rospy.get_name()))
        # else:
        #     rospy.logerr('[ {} ]: Not Ready!'.format(rospy.get_name()))

        self.dock_rotate_angle = pi
        self.reconf_client = dynamic_reconfigure.client.Client("dynamic_reconf_server", timeout=30) # client of fms_rob dynmaic reconfigure server
        rospy.on_shutdown(self.shutdown_hook) # used to reset the interface with the ros_mocap package
        #self.home_flag = True
        #self.undock_flag = True
        self.reconf_client.update_configuration({'home': True})
        self.reconf_client.update_configuration({'undock': True})
        self.reconf_client.update_configuration({'dock': False})
        self.reconf_client.update_configuration({'pick': True})
        self.reconf_client.update_configuration({'place': False})
        self.reconf_client.update_configuration({'return': False})
        #rospy.set_param('/dynamic_reconf_server/home', True)
        #rospy.set_param('/dynamic_reconf_server/undock', True)
        rospy.sleep(1)
        if not err_flag:
            rospy.loginfo('[ {} ]: Ready'.format(rospy.get_name()))
        else:
            rospy.logerr('[ {} ]: Not Ready!'.format(rospy.get_name()))

    def followWP(self, data):
        """ Executes the follow waypoints """
        # print("pick flag", self.pick_flag)
        if (data.action == 'return'):
            self.command_id = data.command_id # Note: command id is updated only when the action is chosen and not for all sent actions
            self.action = data.action
            self.cart_id= data.cart_id
            self.station_id = data.station_id
            Xwaypoints = data.Xwaypoints
            Ywaypoints = data.Ywaypoints
            #print(Xwaypoints)
            #print(Ywaypoints)
            
            # if self.pick_flag == True and data.action == 'place':
            #     status = self.act_client.get_state()
            #     print("status of action" , status)

            goal = followWaypointsGoal()
            goal.Xwaypoints = Xwaypoints
            goal.Ywaypoints = Ywaypoints
            goal.action = data.action

            rospy.loginfo('[ {} ]: Sending waypoints list to action server'.format(rospy.get_name())) 
            self.act_client.send_goal(goal) # non-blocking
            self.status_flag = True
   
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

    def update_wp(self, data):
        self.waypoints = data
        #print(data)

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
                rospy.logerr('[ {} ]: Execution Aborted by Follow waypoints Server!'.format(rospy.get_name()))
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
        if WaypointMode == True:
            fwp = FollowWPReturnActionClient()
        else:
            ra = ReturnAction()
    except KeyboardInterrupt:
        sys.exit()
        #rospy.logerr('Interrupted!')
    rospy.spin()
