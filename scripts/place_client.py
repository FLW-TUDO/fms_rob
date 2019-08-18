#!/usr/bin/env python

import rospy
import actionlib
import sys
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Pose, TransformStamped
from fms_rob.msg import RobActionSelect, RobActionStatus
from fms_rob.srv import  parkPose
from robotnik_msgs.srv import set_odometry, set_digital_output
from actionlib_msgs.msg import GoalStatusArray
from std_msgs.msg import String
from math import cos, sin, pi
import tf_conversions
from std_srvs.srv import Empty
import elevator_test


'''
#######################################################################################
'''

ROBOT_ID = 'rb1_base_b'

'''
#######################################################################################
'''

class place_action:
    def __init__(self):
        rospy.init_node('place_action_client')
        self.status_flag = False
        self.client = actionlib.SimpleActionClient('rb1_base_b/move_base', MoveBaseAction) 
        self.client.wait_for_server() # wait for server for each goal?
        self.action_sub = rospy.Subscriber('/'+ROBOT_ID+'/rob_action', RobActionSelect, self.place)
        self.status_update_sub = rospy.Subscriber('/'+ROBOT_ID+'/move_base/status', GoalStatusArray, self.status_update) # status from move base action server 
        self.action_status_pub = rospy.Publisher('/'+ROBOT_ID+'/rob_action_status', RobActionStatus, queue_size=10)
        self.klt_num_pub = rospy.Publisher('/'+ROBOT_ID+'/klt_num', String, queue_size=10)
        self.dock_distance = 0.500
        self.dock_rotate_angle = pi
        self.park_distance = 1.1
        print('Ready for Placing')

    def place(self, data):
        self.command_id = data.command_id # to be removed after msg modification
        self.action = data.action # to be removed after msg modification
        self.cart_id = data.cart_id
        if (data.action == 'place'):
            parking_spots = self.calc_park_spots(self.cart_id, self.park_distance)
            print(parking_spots)
            if (parking_spots == None):
                print('Cart Topic Not Found!')
                return
            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = "vicon_world" # Always send goals in reference to vicon_world when using ros_mocap package
            goal.target_pose.header.stamp = rospy.Time.now()
            goal.target_pose.pose.position.x = parking_spots.inbound.position.x
            goal.target_pose.pose.position.y = parking_spots.inbound.position.y
            goal.target_pose.pose.orientation.x = parking_spots.inbound.orientation.x
            goal.target_pose.pose.orientation.y = parking_spots.inbound.orientation.y
            goal.target_pose.pose.orientation.z = parking_spots.inbound.orientation.z
            goal.target_pose.pose.orientation.w = parking_spots.inbound.orientation.w
            print('Sending Place goal to action server: ') 
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
            self.status_flag = False
            return

    def calc_park_spots(self, cart_id, park_distance):
        print('Calculating Parking Spots')
        rospy.wait_for_service('/'+ROBOT_ID+'/get_parking_spots')
        try:
            get_park_spots = rospy.ServiceProxy('/'+ROBOT_ID+'/get_parking_spots', parkPose)
            resp = get_park_spots(cart_id, park_distance)
            return resp
        except rospy.ServiceException:
            print ('Calculating Docking Position Service call Failed!')

    def status_update(self, data): # forwarding status messages
        if (self.status_flag == True):
            #print(data.status_list[1].status) # All status list info are at indices 0 and 1
            status = self.client.get_state()
            print(status)
            msg = RobActionStatus()
            #self.client.stop_tracking_goal()
            msg.status = status
            msg.command_id = self.command_id # to be removed after msg modification
            msg.action = self.action # to be removed after msg modification
            msg.cart_id = self.cart_id
            self.action_status_pub.publish(msg)
            if (status == 3):
                #self.dock(status)
                #self.klt_num_pub.publish('/vicon/'+self.cart_id+'/'+self.cart_id) # when robot is under cart publish entire vicon topic of cart for ros_mocap reference
                self.client.stop_tracking_goal()
                self.status_flag = False
                return                 

    
if __name__ == '__main__':
    try:
        pa = place_action()
    except KeyboardInterrupt:
        sys.exit()
        print('Interrupted!')
    rospy.spin()
