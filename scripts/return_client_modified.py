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
import tf_conversions
from math import sqrt, atan2, sin, cos
from geometry_msgs.msg import PoseStamped, TransformStamped, Twist, PointStamped


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
        self.vel_pub = rospy.Publisher('/'+ROBOT_ID+'/move_base/cmd_vel', Twist, queue_size=10)
        self.klt_pose_sub = rospy.Subscriber('/'+ROBOT_ID+'/klt_num', TransformStamped, self.update_pose)
        self.reconf_client = dynamic_reconfigure.client.Client("dynamic_reconf_server", timeout=30) # client of fms_rob dynmaic reconfigure server
        rospy.on_shutdown(self.shutdown_hook) # used to reset the interface with the ros_mocap package
        #self.place_flag = True
        #self.dock_flag = True
        ###self.dock_flag = Bool()
        ###self.place_flag = Bool()
        self.sample_time = 0.0001
        current_time = None
        self.current_time = current_time if current_time is not None else time.time()
        self.last_time = self.current_time
        self.p_term_ang = 0.0
        self.d_term_ang = 0.0
        self.last_error_theta = 0.0
        self.output = 0.0
        self.distance_tolerance = 0.005
        self.kp_trans = 0.8
        self.orientation_tolerance = 0.005
        self.kp_orient = 1.0
        self.kp_ang = 0.7 #0.7 
        self.kd_ang = 0.1 #0.1
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
                rospy.loginfo('[ {} ]: Navigating to secondary return position'.format(rospy.get_name())) 
                se_goal = self.get_secondary_goal()
                try:
                    rospy.wait_for_service('/'+ROBOT_ID+'/move_base/clear_costmaps') # clear cost maps before sending goal to remove false positive obstacles
                    reset_costmaps = rospy.ServiceProxy('/'+ROBOT_ID+'/move_base/clear_costmaps', Empty)
                    reset_costmaps()
                    rospy.loginfo('[ {} ]: Costmaps Cleared Successfully'.format(rospy.get_name())) 
                except:
                    rospy.logwarn('[ {} ]: Costmaps Clearing Service Call Failed!'.format(rospy.get_name())) 
                rospy.sleep(0.5)
                #self.act_client.send_goal_and_wait(goal) # blocking
                self.act_client.send_goal(se_goal) # non-blocking
                self.status_flag = True
                while(self.act_client.get_state() != 3):
                    pass
                # reutrn to original cart position
                self.amend_return_pose()
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

    def get_orginal_cart_pose(self):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "vicon_world" # Always send goals in reference to vicon_world when using ros_mocap package
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = rospy.get_param('/'+ROBOT_ID+'/dynamic_reconf_server/return_pose_trans_x')
        goal.target_pose.pose.position.y = rospy.get_param('/'+ROBOT_ID+'/dynamic_reconf_server/return_pose_trans_y')
        goal.target_pose.pose.orientation.x = rospy.get_param('/'+ROBOT_ID+'/dynamic_reconf_server/return_pose_rot_x')
        goal.target_pose.pose.orientation.y = rospy.get_param('/'+ROBOT_ID+'/dynamic_reconf_server/return_pose_rot_y')
        goal.target_pose.pose.orientation.z = rospy.get_param('/'+ROBOT_ID+'/dynamic_reconf_server/return_pose_rot_z')
        goal.target_pose.pose.orientation.w = rospy.get_param('/'+ROBOT_ID+'/dynamic_reconf_server/return_pose_rot_w')
        # rospy.loginfo('[ {} ]: Sending Return goal to action server'.format(rospy.get_name())) 
        return goal
    
    def get_secondary_goal(self): # secondary return goal
        se_goal = self.get_orginal_cart_pose()
        shift_distance_x = 0.1
        se_goal.target_pose.pose.position.x = se_goal.target_pose.pose.position.x + shift_distance_x  # Secondary cart position always to the right of the original cart position
        return se_goal

    def amend_return_pose(self):
        original_goal = self.get_orginal_cart_pose()    
        goal_trans_x = original_goal.target_pose.pose.position.x
        goal_trans_y = original_goal.target_pose.pose.position.y
        goal_rot_x = original_goal.target_pose.pose.orientation.x
        goal_rot_y = original_goal.target_pose.pose.orientation.y
        goal_rot_z = original_goal.target_pose.pose.orientation.z
        goal_rot_w = original_goal.target_pose.pose.orientation.w

        success = True
        vel_msg = Twist()
        rospy.loginfo('[ {} ]: Amending Return Pose'.format(rospy.get_name()))
        r = rospy.Rate(10)
        while(self.euclidean_distance(goal_trans_x, goal_trans_y) >= self.distance_tolerance):
            vel_msg.linear.x = self.euclidean_distance(goal_trans_x, goal_trans_y) * self.kp_trans
            #print('Euclidean Distance: {}'.format(self.euclidean_distance(goal_x, goal_y)))
            vel_msg.linear.y = 0
            vel_msg.linear.z = 0
            # Angular velocity in the z-axis.
            vel_msg.angular.x = 0
            vel_msg.angular.y = 0
            vel_msg.angular.z = self.angular_vel(goal_trans_x, goal_trans_y)#
            #print('Angular Vel: {}'.format(self.angular_vel(goal_x, goal_y)))
            #print('-----------')
            self.vel_pub.publish(vel_msg)
            r.sleep()
        vel_msg.linear.x = 0
        vel_msg.angular.z = 0
        self.vel_pub.publish(vel_msg)
        rospy.loginfo('[ {} ]: Amended Return Position Reached'.format(rospy.get_name()))
        goal_rot = tf_conversions.transformations.euler_from_quaternion([goal_rot_x, goal_rot_y, goal_rot_z, goal_rot_w])
        orientation_error = goal_rot - self.curr_theta
        orientation_error_mod = atan2(sin(orientation_error),cos(orientation_error))
        while(abs(orientation_error_mod) >= self.orientation_tolerance):
            vel_msg.angular.z = orientation_error * self.kp_orient
            self.vel_pub.publish(vel_msg)
            orientation_error = goal_rot - self.curr_theta
            orientation_error_mod = atan2(sin(orientation_error),cos(orientation_error))
        rospy.loginfo('[ {} ]:  Amended Return Orientation Reached'.format(rospy.get_name()))
        return success
        

    def angular_vel(self, goal_x, goal_y):
        """ PD controller output calculation. """
        current_time = None
        self.error_theta = self.goal_angle(goal_x, goal_y) - self.curr_theta
        self.error_theta = atan2(sin(self.error_theta), cos(self.error_theta)) # angle sign regulation
        #print('Goal angle: {}'.format(self.goal_angle(goal_x, goal_y)))
        #print('Current angle: {}'.format(self.curr_theta))
        #print('Error angle: {}'.format(self.error_theta))
        #self.theta_msg = self.error_theta
        self.current_time = current_time if current_time is not None else time.time()
        delta_time = self.current_time - self.last_time
        delta_error = self.error_theta - self.last_error_theta
        if (delta_time > self.sample_time):
            self.p_term_ang = self.kp_ang * self.error_theta
            self.d_term_ang = 0.0
            if delta_time > 0:
                self.dTerm = delta_error / delta_time
            self.last_time = self.current_time
            self.last_error_theta = self.error_theta
            self.output = self.p_term_ang + (self.kd_ang * self.d_term_ang)
        return self.output

    def goal_angle(self, goal_x, goal_y):
        """ Angle between current orientation and the heading of the next way point. """
        return atan2(goal_y - self.curr_pose_trans_y, goal_x - self.curr_pose_trans_x)
        
    def update_pose(self, data):
        """ Robot vicon pose (under cart) update. """
        self.curr_pose_trans_x = data.transform.translation.x
        self.curr_pose_trans_y = data.transform.translation.y
        rot=[data.transform.rotation.x, data.transform.rotation.y, data.transform.rotation.z, data.transform.rotation.w]
        rot_euler = tf_conversions.transformations.euler_from_quaternion(rot)
        self.curr_theta = rot_euler[2]

    def euclidean_distance(self, goal_x, goal_y):
        """ Euclidean distance between current pose and the next way point."""
        return sqrt(pow((goal_x - self.curr_pose_trans_x), 2) + pow((goal_y - self.curr_pose_trans_y), 2))

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
