#!/usr/bin/env python
'''
A client that requests the navigation of the robot to a location specified 
by the user in the MQTT msg
'''

import rospy
import actionlib
import sys
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseStamped
from fms_rob.msg import RobActionSelect, RobActionStatus
from actionlib_msgs.msg import GoalStatusArray
from std_srvs.srv import Empty
from std_msgs.msg import String


'''
#######################################################################################
'''

ROBOT_ID = rospy.get_param('/ROBOT_ID', 'rb1_base_b') # by default the robot id is set in the package's launch file

'''
#######################################################################################
'''

class drive_action:
    def __init__(self):
        rospy.init_node('drive_action_client')
        self.status_flag = False # used to throttle further message sending after action execution
        self.client = actionlib.SimpleActionClient('rb1_base_b/move_base', MoveBaseAction) 
        self.client.wait_for_server() # wait for server start up
        self.action_sub = rospy.Subscriber('/'+ROBOT_ID+'/rob_action', RobActionSelect, self.drive)
        self.status_update_sub = rospy.Subscriber('/'+ROBOT_ID+'/move_base/status', GoalStatusArray, self.status_update) # status from move base action server  
        self.action_status_pub = rospy.Publisher('/'+ROBOT_ID+'/rob_action_status', RobActionStatus, queue_size=10) # publishes status msgs upstream
        self.klt_num_pub = rospy.Publisher('/'+ROBOT_ID+'/klt_num', String, queue_size=10) # used for interfacing with the ros_mocap package
        rospy.on_shutdown(self.shutdown_hook) # used to reset the interface with the ros_mocap package
        rospy.loginfo('Ready for Driving')

    def drive(self, data):
        '''
        Executes the drive action
        '''
        self.command_id = data.command_id
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
            rospy.loginfo('Sending Drive goal to action server') 
            rospy.loginfo('Drive goal coordinates: {}'.format(goal))
            rospy.wait_for_service('/'+ROBOT_ID+'/move_base/clear_costmaps')  # clear cost maps before sending goal to remove false positive obstacles
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
            self.goal_flstatus_flagag = False
            return

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
            self.action_status_pub.publish(msg)
            if (status == 3): # if action execution is successful
                self.client.stop_tracking_goal()
                self.status_flag = False
                return
    
    def shutdown_hook(self):
        self.klt_num_pub.publish('') # resets the picked up cart number in the ros_mocap package
        rospy.logwarn('Drive Client node shutdown by user')

if __name__ == '__main__':
    try:
        da = drive_action()
    except KeyboardInterrupt:
        sys.exit()
        rospy.logerr('Interrupted!')
    rospy.spin()
