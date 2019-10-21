#!/usr/bin/env python
'''
A client that requests the navigation of the robot to the picking position
infront of the cart in preparation for the docking action. It acts as a client 
to ROS's built in move base node,which is an implementation of an action server.
Please note that the status message architecture follows the Goal Status Array
type specified in ROS actions by default.
'''

import rospy
import actionlib
import sys
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Pose, TransformStamped
from fms_rob.msg import RobActionSelect, RobActionStatus
from fms_rob.srv import  dockPose
from actionlib_msgs.msg import GoalStatusArray
from std_msgs.msg import String
from math import cos, sin, pi
from std_srvs.srv import Empty
import time
import dynamic_reconfigure.client


'''
#######################################################################################
'''

ROBOT_ID = rospy.get_param('/ROBOT_ID', 'rb1_base_b') # by default the robot id is set in the package's launch file

'''
#######################################################################################
'''

class pick_action:
    def __init__(self):
        rospy.init_node('pick_action_client')
        self.status_flag = False # used to throttle further message sending after action execution
        self.client = actionlib.SimpleActionClient('/'+ROBOT_ID+'/move_base', MoveBaseAction) 
        rospy.loginfo('Waiting for move_base server')
        self.client.wait_for_server() # wait for server start up
        self.action_sub = rospy.Subscriber('/'+ROBOT_ID+'/rob_action', RobActionSelect, self.pick)
        self.status_update_sub = rospy.Subscriber('/'+ROBOT_ID+'/move_base/status', GoalStatusArray, self.status_update) # status from move base action server 
        self.action_status_pub = rospy.Publisher('/'+ROBOT_ID+'/rob_action_status', RobActionStatus, queue_size=10) # publishes status msgs upstream
        self.klt_num_pub = rospy.Publisher('/'+ROBOT_ID+'/klt_num', String, queue_size=10) # used for interfacing with the ros_mocap package
        #self.klt_num_pub = rospy.Publisher('/'+ROBOT_ID+'/klt_num', String, queue_size=10)
        self.dock_distance = 1.0 # min: 1.0
        rospy.set_param(ROBOT_ID+'/fms_rob/dock_distance', self.dock_distance) # docking distance infront of cart, before secondary docking motion
        self.dock_rotate_angle = pi
        self.reconf_client = dynamic_reconfigure.client.Client("dynamic_reconf_server", timeout=30)
        rospy.on_shutdown(self.shutdown_hook) # used to reset the interface with the ros_mocap package
        rospy.loginfo('Ready for Picking')

    def pick(self, data):
        '''
        Executes picking action
        '''
        self.command_id = data.command_id
        self.action = data.action # to be removed after msg modification
        self.cart_id = data.cart_id
        self.reconf_client.update_configuration({"cart_id": self.cart_id}) # dynamic parameter to share cart_id in between clients at runtime
        if (data.action == 'pick'):
            dock_pose = self.calc_dock_position(self.cart_id)
            rospy.loginfo('Dock Pose coordinates: {}'.format(dock_pose))
            if (dock_pose == None):
                rospy.logerr('Cart Topic Not Found!')
                return
            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = "vicon_world" # Always send goals in reference to vicon_world when using ros_mocap package
            goal.target_pose.header.stamp = rospy.Time.now()
            goal.target_pose.pose.position.x = dock_pose.position.x
            goal.target_pose.pose.position.y = dock_pose.position.y
            goal.target_pose.pose.orientation.x = dock_pose.orientation.x
            goal.target_pose.pose.orientation.y = dock_pose.orientation.y
            goal.target_pose.pose.orientation.z = dock_pose.orientation.z
            goal.target_pose.pose.orientation.w = dock_pose.orientation.w
            rospy.loginfo('Sending Pick goal to action server') 
            rospy.loginfo('Pick goal coordinates: {}'.format(goal))
            rospy.wait_for_service('/'+ROBOT_ID+'/move_base/clear_costmaps') # clear cost maps before sending goal to remove false positive obstacles
            reset_costmaps = rospy.ServiceProxy('/'+ROBOT_ID+'/move_base/clear_costmaps', Empty)
            reset_costmaps()
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
            self.status_flag = False
            return

    def calc_dock_position(self, cart_id):
        '''
        Calls a service to calculate the pick position infront of the desired cart 
        '''
        rospy.loginfo('Calculating Docking Position')
        rospy.wait_for_service('/'+ROBOT_ID+'/get_docking_pose')
        try:
            get_goal_offset = rospy.ServiceProxy('/'+ROBOT_ID+'/get_docking_pose', dockPose)
            resp = get_goal_offset(cart_id, self.dock_distance)
            return resp.dock_pose
        except rospy.ServiceException:
            rospy.logerr('Calculating Docking Position Service call Failed!')

    def status_update(self, data):
        '''
        Forwarding status messages upstream
        '''
        if (self.status_flag == True):
            #print(data.status_list[1].status) # All status list info are at indices 0 and 1
            status = self.client.get_state()
            print(status)
            msg = RobActionStatus()
            #self.client.stop_tracking_goal()
            msg.status = status
            msg.command_id = self.command_id
            msg.action = self.action # to be removed after msg modification
            msg.cart_id = self.cart_id
            self.action_status_pub.publish(msg)
            if (status == 3): # if action execution is successful 
                self.client.stop_tracking_goal()
                self.status_flag = False
                return
    
    def shutdown_hook(self):
        self.klt_num_pub.publish('') # resets the picked up cart number in the ros_mocap package
        rospy.logwarn('Pick Client node shutdown by user')
    
if __name__ == '__main__':
    try:
        pa = pick_action()
    except KeyboardInterrupt:
        sys.exit()
        rospy.logerr('Interrupted!')
    rospy.spin()
