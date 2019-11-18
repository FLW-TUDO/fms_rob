#!/usr/bin/env python
"""
A client that requests the navigation of the robot to a station. The exact location
for parking next to the station is specified by the bound mode sent by user (inbound
- outbound - queue). It acts as a client to ROS's built in move base node, which is 
an implementation of an action server. Please note that the status message architecture
follows the Goal Status Array type specified in ROS actions by default.
"""

import rospy
import actionlib
import sys
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Pose, TransformStamped
from fms_rob.msg import RobActionSelect, RobActionStatus
from fms_rob.srv import  parkPose
from robotnik_msgs.srv import set_odometry, set_digital_output
from actionlib_msgs.msg import GoalStatusArray
from std_msgs.msg import String, Bool
from math import cos, sin, pi
import tf_conversions
from std_srvs.srv import Empty
import dynamic_reconfigure.client


'''
#######################################################################################
'''

ROBOT_ID = rospy.get_param('/ROBOT_ID') # by default the robot id is set in the package's launch file

'''
#######################################################################################
'''

class PlaceAction:
    def __init__(self):
        rospy.init_node('place_action_client')
        self.status_flag = False # used to throttle further message sending after action execution
        self.act_client = actionlib.SimpleActionClient('/'+ROBOT_ID+'/move_base', MoveBaseAction) 
        rospy.loginfo('Waiting for move_base server')
        self.act_client.wait_for_server()  # wait for server start up
        self.action_sub = rospy.Subscriber('/'+ROBOT_ID+'/rob_action', RobActionSelect, self.place)
        self.status_update_sub = rospy.Subscriber('/'+ROBOT_ID+'/move_base/status', GoalStatusArray, self.status_update) # status from move base action server 
        self.action_status_pub = rospy.Publisher('/'+ROBOT_ID+'/rob_action_status', RobActionStatus, queue_size=10)
        self.klt_num_pub = rospy.Publisher('/'+ROBOT_ID+'/klt_num', String, queue_size=10) # used for interfacing with the ros_mocap package
        #self.klt_num_pub = rospy.Publisher('/'+ROBOT_ID+'/klt_num', String, queue_size=10)
        self.park_distance = 1.1 # min: 1.02
        rospy.on_shutdown(self.shutdown_hook) # used to reset the interface with the ros_mocap package
        self.reconf_client = dynamic_reconfigure.client.Client("dynamic_reconf_server", timeout=30) # client of fms_rob dynmaic reconfigure server
        #self.dock_flag = Bool()
        #self.dock_flag = True
        rospy.sleep(1)
        rospy.loginfo('Ready for Placing')

    def place(self, data):
        """ Executes the placing operation. """
        if (data.action == 'place'):
            self.command_id = data.command_id
            self.action = data.action # to be removed after msg modification
            self.station_id = data.station_id
            self.bound_mode = data.bound_mode
            dock_flag = rospy.get_param('/'+ROBOT_ID+'/dynamic_reconf_server/dock')
            if (dock_flag == True):
                parking_spots = self.calc_park_spots(self.station_id, self.park_distance)
                rospy.loginfo('Calculated parking spots for placing: {}'.format(parking_spots))
                goal = MoveBaseGoal()
                goal.target_pose.header.frame_id = "vicon_world" # Always send goals in reference to vicon_world when using ros_mocap package
                goal.target_pose.header.stamp = rospy.Time.now()
                if (parking_spots == None):
                    rospy.logerr('Station Topic Not Found!')
                    return
                if (self.bound_mode == 'inbound'):
                    goal.target_pose.pose.position.x = parking_spots.inbound.pose.position.x
                    goal.target_pose.pose.position.y = parking_spots.inbound.pose.position.y
                    goal.target_pose.pose.orientation.x = parking_spots.inbound.pose.orientation.x
                    goal.target_pose.pose.orientation.y = parking_spots.inbound.pose.orientation.y
                    goal.target_pose.pose.orientation.z = parking_spots.inbound.pose.orientation.z
                    goal.target_pose.pose.orientation.w = parking_spots.inbound.pose.orientation.w
                if (self.bound_mode == 'outbound'):
                    goal.target_pose.pose.position.x = parking_spots.outbound.pose.position.x
                    goal.target_pose.pose.position.y = parking_spots.outbound.pose.position.y
                    goal.target_pose.pose.orientation.x = parking_spots.outbound.pose.orientation.x
                    goal.target_pose.pose.orientation.y = parking_spots.outbound.pose.orientation.y
                    goal.target_pose.pose.orientation.z = parking_spots.outbound.pose.orientation.z
                    goal.target_pose.pose.orientation.w = parking_spots.outbound.pose.orientation.w
                if (self.bound_mode == 'queue'):
                    goal.target_pose.pose.position.x = parking_spots.queue.pose.position.x
                    goal.target_pose.pose.position.y = parking_spots.queue.pose.position.y
                    goal.target_pose.pose.orientation.x = parking_spots.queue.pose.orientation.x
                    goal.target_pose.pose.orientation.y = parking_spots.queue.pose.orientation.y
                    goal.target_pose.pose.orientation.z = parking_spots.queue.pose.orientation.z
                    goal.target_pose.pose.orientation.w = parking_spots.queue.pose.orientation.w
                rospy.loginfo('Sending Place goal to action server') 
                rospy.wait_for_service('/'+ROBOT_ID+'/move_base/clear_costmaps') # clear cost maps before sending goal to remove false positive obstacles
                reset_costmaps = rospy.ServiceProxy('/'+ROBOT_ID+'/move_base/clear_costmaps', Empty)
                reset_costmaps()
                #self.act_client.send_goal_and_wait(goal) # blocking
                self.act_client.send_goal(goal) # non-blocking
                self.status_flag = True
            else:
                rospy.logerr('Action Rejected! - Attempting to place without dock')
                return
        else:
            if (data.action == 'cancelCurrent'):
                self.act_client.cancel_goal()
                rospy.logwarn('Cancelling Current Goal')
                self.reconf_client.update_configuration({"dock": False})
            if (data.action == 'cancelAll'):
                self.act_client.cancel_all_goals()
                rospy.logwarn('cancelling All Goals')
                self.reconf_client.update_configuration({"dock": False})
            if (data.action == 'cancelAtAndBefore'):
                self.act_client.cancel_goals_at_and_before_time(data.cancellation_stamp)
                s = 'Cancelling all Goals at and before {}'.format(data.cancellation_stamp)
                rospy.logwarn(s)
                self.reconf_client.update_configuration({"dock": False})
            self.act_client.stop_tracking_goal()
            self.status_flag = False
            return

    def calc_park_spots(self, station_id, park_distance):
        """
        Calls a service to Calculate the location of 3 parking spots next to the workstation
        (inbound - outbound - queue)
        """
        rospy.loginfo('Calculating Parking Spots')
        rospy.wait_for_service('/'+ROBOT_ID+'/get_parking_spots')
        try:
            get_park_spots = rospy.ServiceProxy('/'+ROBOT_ID+'/get_parking_spots', parkPose)
            resp = get_park_spots(station_id, park_distance)
            return resp
        except rospy.ServiceException:
            rospy.logerr('Calculating Docking Position Service call Failed!')

    '''
    def dynamic_params_update(self, config):
        """ Dynamically Obtaining the interlock state. """
        #rospy.loginfo("Config set to {pick}, {dock}, {undock}, {place}, {home}, {return}".format(**config))
        self.dock_flag = config['dock']
    '''

    def status_update(self, data):
        """ Forwarding status messages upstream. """
        if (self.status_flag == True):
            #print(data.status_list[1].status) # All status list info are at indices 0 and 1
            status = self.act_client.get_state()
            print(status)
            msg = RobActionStatus()
            #self.act_client.stop_tracking_goal()
            msg.status = status
            msg.command_id = self.command_id # to be removed after msg modification
            msg.action = self.action # to be removed after msg modification
            msg.station_id = self.station_id
            msg.bound_mode = self.bound_mode
            self.action_status_pub.publish(msg)
            if (status == 3): # if action execution is successful 
                #self.reconf_client.update_configuration({"dock": False})
                self.reconf_client.update_configuration({"place": True})
                #self.reconf_client.update_configuration({"dock": False})
                self.act_client.stop_tracking_goal()
                self.status_flag = False
                return  
            if (status == 4): # if action execution is aborted
                #self.reconf_client.update_configuration({"dock": False})
                self.act_client.stop_tracking_goal()
                self.status_flag = False
                rospy.logerr('Execution Aborted by Move Base Server!')               

    def shutdown_hook(self):
        self.klt_num_pub.publish('')  # resets the picked up cart number in the ros_mocap package
        self.act_client.cancel_all_goals()
        rospy.logwarn('Place Client node shutdown by user')
    
if __name__ == '__main__':
    try:
        pa = PlaceAction()
    except KeyboardInterrupt:
        sys.exit()
        rospy.logerr('Interrupted!')
    rospy.spin()
