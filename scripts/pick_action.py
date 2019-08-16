#!/usr/bin/env python

import rospy
import actionlib
import sys
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Pose, TransformStamped
from robotnik_msgs.msg import RobActionSelect, RobActionStatus
from robotnik_msgs.srv import dockingPose, dockMove, dockRotate, set_odometry, set_digital_output
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

class pick_action:
    def __init__(self):
        rospy.init_node('pick_action')
        self.status_flag = False
        self.client = actionlib.SimpleActionClient('rb1_base_b/move_base', MoveBaseAction) 
        self.client.wait_for_server() # wait for server for each goal?
        self.action_sub = rospy.Subscriber('/'+ROBOT_ID+'/rob_action', RobActionSelect, self.pick)
        self.status_update_sub = rospy.Subscriber('/'+ROBOT_ID+'/move_base/status', GoalStatusArray, self.status_update) # status from action server - use feedback instead ?
        self.action_status_pub = rospy.Publisher('/'+ROBOT_ID+'/rob_action_status', RobActionStatus, queue_size=10)
        self.klt_num_pub = rospy.Publisher('/'+ROBOT_ID+'/klt_num', String, queue_size=10)
        self.dock_distance = 0.500
        self.dock_rotate_angle = pi
        print('Ready for Picking')

    def pick(self, data):
        self.command_id = data.command_id # to be removed after msg modification
        self.action = data.action # to be removed after msg modification
        self.cart_id = data.cart_id
        if (data.action == 'pick'):
            dock_pose = self.calc_dock_position(self.cart_id)
            print(dock_pose)
            if (dock_pose == None):
                print('Cart Topic Not Found!')
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
            print('Sending Pick goal to action server: ') 
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

    def calc_dock_position(self, cart_id):
        print('Calculating Docking Position')
        rospy.wait_for_service('/'+ROBOT_ID+'/get_docking_pose')
        try:
            get_goal_offset = rospy.ServiceProxy('/'+ROBOT_ID+'/get_docking_pose', dockingPose)
            resp = get_goal_offset(cart_id, self.dock_distance)
            return resp.dock_pose
        except rospy.ServiceException:
            print ('Calculating Docking Position Service call Failed!')

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
            msg.cart_id = self.cart_id
            self.action_status_pub.publish(msg)
            if (status == 3):
                self.dock(status)
                #self.klt_num_pub.publish('/vicon/'+self.cart_id+'/'+self.cart_id) # when robot is under cart publish entire vicon topic of cart for ros_mocap reference
                self.client.stop_tracking_goal()
                self.status_flag = False
                return

    def dock(self, status):
        '''
        Performs the actual docking operation. Moving under the cart, raising the elevator, and then rotating.
        Docking operation is blocking and cannot be interrupted.
        '''
        print('Initiating Docking')
        try:
            print('Resetting Odom')
            rospy.wait_for_service('/'+ROBOT_ID+'/set_odometry')
            reset_odom1 = rospy.ServiceProxy('/'+ROBOT_ID+'/set_odometry', set_odometry)
            reset_odom1(0.0,0.0,0.0,0.0)
            print('Odom Reset Successful')
            print('Moving under Cart')
            rospy.wait_for_service('/'+ROBOT_ID+'/dock_move')
            do_dock_move = rospy.ServiceProxy('/'+ROBOT_ID+'/dock_move', dockMove)
            resp_move = do_dock_move(self.dock_distance)
            print('Moving under Cart Successful')
        except rospy.ServiceException:
            print ('Dock Move OR Odom Reset Service call Failed!')
        if (resp_move.ret == True):
            try:
                self.klt_num_pub.publish('/vicon/'+self.cart_id+'/'+self.cart_id) # when robot is under cart publish entire vicon topic of cart for ros_mocap reference
                print('Raising Elevator')
                rospy.wait_for_service('/'+ROBOT_ID+'/robotnik_base_hw/set_digital_output')
                do_raise_elevator = rospy.ServiceProxy('/'+ROBOT_ID+'/robotnik_base_hw/set_digital_output', set_digital_output)
                resp_raise_elevator = do_raise_elevator(3,True) # 3 --> raise elevator // 2 --> lower elevator
                rospy.sleep(7) # service returns immedietly, a wait time is needed#
                #execfile('src/fms_rob/scripts/elevator_test.py')
                #elevator_test.do_elev_test()
                print('Elevator Raise Successful')
            except: 
                print ('Elevator Raise Service call Failed!')
                
            if (resp_raise_elevator.ret == True):
                try:
                    print('Rotating Cart')
                    rospy.wait_for_service('/'+ROBOT_ID+'/dock_rotate')
                    do_dock_rotate = rospy.ServiceProxy('/'+ROBOT_ID+'/dock_rotate', dockRotate)
                    resp_move = do_dock_rotate(self.dock_rotate_angle)
                    print('Rotating Cart Successful')
                except: 
                    print ('Dock Rotate Service call Failed!')
                 

    
if __name__ == '__main__':
    try:
        pa = pick_action()
    except KeyboardInterrupt:
        sys.exit()
        print('Interrupted!')
    rospy.spin()
