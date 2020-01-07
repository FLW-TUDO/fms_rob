#!/usr/bin/env python
"""
An action server to execute both the docking and undocking operations.
The docking operation is composed of moving under cart (in 2 phases), lifting 
the elevator, then rotation the cart. Undocking is composed of lowering the elevator, rotating 
the robot under the cart, then moving out from under the cart. 
Please note that the docking operation is executed after the robot has been 
positioned in the picking location.
"""

import rospy
import actionlib
import sys, time
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseStamped, TransformStamped, Twist, PointStamped
from fms_rob.msg import dockUndockAction, dockUndockGoal, dockUndockFeedback, dockUndockResult
from sensor_msgs.msg import Joy
from nav_msgs.msg import Odometry
from robotnik_msgs.srv import set_odometry, set_digital_output
from rb1_base_msgs.srv import SetElevator
#from actionlib_msgs.msg import GoalStatusArray
from std_msgs.msg import String, Bool, Float32
from math import pow, atan2, sqrt, cos, sin, pi
import tf_conversions
#from std_srvs.srv import Empty
import dynamic_reconfigure.client
#import elevator_test


'''
#######################################################################################
'''

ROBOT_ID = rospy.get_param('/ROBOT_ID') # by default the robot id is set in the package's launch file

'''
#######################################################################################
'''

class DUActionServer:

    def __init__(self):
        rospy.init_node('dock_undock_server')
        try:
            self.du_server = actionlib.SimpleActionServer('do_dock_undock', dockUndockAction, self.execute, False) # create dock-undock action server
        except:
            rospy.logerr('[ {} ]: Error Creating Action Server!'.format(rospy.get_name()))
        self.du_server.start()
        self.odom_sub = rospy.Subscriber('/'+ROBOT_ID+'/dummy_odom', Odometry, self.get_odom) # dummy odom is the remapped odom topic - please check ros_mocap package
        self.vel_pub = rospy.Publisher('/'+ROBOT_ID+'/move_base/cmd_vel', Twist, queue_size=10)
        self.klt_num_pub = rospy.Publisher('/'+ROBOT_ID+'/klt_num', String, queue_size=10) # used for interfacing with the ros_mocap package
        self.pose_sub = rospy.Subscriber('/vicon/'+ROBOT_ID+'/'+ROBOT_ID, TransformStamped, self.update_pose) ####
        self.joystick_sub = rospy.Subscriber('/'+ROBOT_ID+'/joy', Joy, self.joy_update)
        try:
            self.reconf_client = dynamic_reconfigure.client.Client('dynamic_reconf_server', timeout=30) # client of fms_rob dynmaic reconfigure server
        except:
            rospy.logerr('Dynamic Reconf Server is Not running!')
        try:
            self.teb_reconf_client = dynamic_reconfigure.client.Client('/'+ROBOT_ID+'/move_base/TebLocalPlannerROS', timeout=30)
        except:
            rospy.logerr('TEB Planner is Not running!')
        '''collision detector settings'''
        self.col_detector_sub = rospy.Subscriber('/'+ROBOT_ID+'/robotnik_safety_controller/warning_collision_point', PointStamped, self.collision_update)
        #self.collision_point = PointStamped()
        #self.collision_tolerance = PointStamped()
        self.collision_point_x = float('inf')
        self.collision_point_y = float('inf')
        self.collision_tolerance_x = 0.7 # 0.75
        self.collision_tolerance_y = 0.39 # 0.4
        self.curr_col_seq = 0
        self.last_col_seq = 0
        ''' P-Controller settings for primary motion '''
        #self.move_speed = 0.09 #0.14
        self.move_kp = 0.99 #0.99
        #self.rot_kp = 0.99 #0.99
        #self.rot_speed = 0.4 #0.5
        self.move_tolerance = 0.007 #0.005
        #self.ang_tolerance = 0.002 #0.002
        self.feedback = dockUndockFeedback()
        self.result = dockUndockResult()
        ''' PD-Controller settings for secondary move '''
        self.error_theta = 1.0 #1.0
        #self.theta_tolerance = 0.007 #0.007
        #self.theta_tolerance = 0.02
        self.kp_ang = 0.7 #0.7 
        self.kd_ang = 0.1 #0.1
        self.kp_orient = 0.6 #0.3
        self.kp_trans = 0.8 #0.8    ######
        self.distance_tolerance = 0.003 #0.003
        self.orientation_tolerance = 0.01 #0.02
        current_time = None
        self.sample_time = 0.0001
        self.current_time = current_time if current_time is not None else time.time()
        self.last_time = self.current_time
        self.p_term_ang = 0.0
        self.d_term_ang = 0.0
        self.last_error_theta = 0.0
        self.output = 0.0
        self.start_msg = Bool()
        #self.theta_msg = Float32()
        #self.cart_id = String()
        rospy.sleep(1)
        rospy.on_shutdown(self.shutdown_hook) # used to reset the interface with the ros_mocap package
        rospy.loginfo('[ {} ]: Ready'.format(rospy.get_name()))

    def execute(self, goal):
        self.control_flag = False
        self.cart_id_sub = rospy.Subscriber('/'+ROBOT_ID+'/pick_cart_id', String, self.update_cart_id) # obtaining cart id from picking node
        rospy.sleep(1)
        # while (self.control_flag == False):
        #     print('waiting for cart id subscribtion')
        #     rospy.sleep(0.2)
        #print(self.cart_id)
        #print('Cart Vicon Topic: {}'.format('/vicon/'+self.cart_id+'/'+self.cart_id))
        self.cart_pose_sub = rospy.Subscriber('/vicon/'+self.cart_id+'/'+self.cart_id, TransformStamped, self.get_cart_pose) # obtaining picked cart id pose
        rospy.sleep(1)
        #print('Cart Pos: ({}, {})'.format(self.cart_pose_trans[0], self.cart_pose_trans[1]))
        dock_distance = goal.distance # distance to be moved under cart
        dock_angle = goal.angle # rotation angle after picking cart
        elev_mode = goal.mode # docking or undocking
        success_move = False
        success_se_move = False # secondary motion to adjust docking distance
        success_elev = False
        success_rotate = False
        success_odom_reset = False
        self.result.res = False
        if (elev_mode == True): # True --> Dock // False --> Undock
            col_dock_flag = False
            while (self.collision_detected()):
                if (col_dock_flag == False):
                    rospy.logwarn('[ {} ]: Cart Slot Occupied!'.format(rospy.get_name()))
                    col_dock_flag = True
                rospy.sleep(1) # note: >0.2
            else:
                rospy.loginfo('[ {} ]: Cart Slot Free'.format(rospy.get_name()))
            success_se_move = self.do_du_se_move(dock_distance) # pre-motion before cart
            rospy.sleep(0.2) # wait for complete halt of robot
            if (success_se_move):
                success_odom_reset = self.reset_odom()
            if (success_odom_reset):
                success_move = self.do_du_move(dock_distance/2.0) # move under cart
                self.save_cart_pose() 
                rospy.sleep(0.2)
            if (success_move):
                success_elev = self.do_du_elev(elev_mode) # raise/lower elevator
            if (success_elev):
                self.rot_speed = 0.7 #0.5
                self.ang_tolerance = 0.02 #0.002
                success_rotate = self.do_du_rotate(dock_angle) # rotate while picking cart
            if (success_move and success_elev and success_rotate and success_odom_reset and success_se_move):
                self.klt_num_pub.publish('/vicon/'+self.cart_id+'/'+self.cart_id) # when robot is under cart publish entire vicon topic of cart for ros_mocap reference
                try:
                    self.teb_reconf_client.update_configuration({"min_obstacle_dist": 0.25}) # increase obstacle inflation distance after carrying cart
                    rospy.loginfo('[ {} ]: Inflation distance updated successfully'. format(rospy.get_name()))
                except:
                    rospy.logerr('[ {} ]: Inflation distance update Failed!'.format(rospy.get_name))
                self.result.res = True
                self.du_server.set_succeeded(self.result)
            else: 
                self.result.res = False
                self.du_server.set_aborted(self.result)
        else:
            success_elev = self.do_du_elev(elev_mode)
            rospy.sleep(0.2)
            if (success_elev):
                success_odom_reset = self.reset_odom()
            if (success_odom_reset):
                self.rot_speed = 0.3 #0.5
                self.ang_tolerance = 0.002 #0.00
                success_rotate = self.do_du_rotate(dock_angle) 
            if (success_rotate):
                col_undock_flag = False
                while (self.collision_detected()):
                    if (col_undock_flag == False):
                        rospy.logwarn('[ {} ]: Robot Exit Occupied!'.format(rospy.get_name()))
                        col_undock_flag = True
                    rospy.sleep(1) # note: >0.2
                else:
                    rospy.loginfo('[ {} ]: Robot Exit Free'.format(rospy.get_name()))
                success_move = self.do_du_move(dock_distance)
            if (success_move and success_elev and success_rotate and success_odom_reset):
                self.klt_num_pub.publish('') # reset robot vicon location for ros_mocap package
                try:
                    self.teb_reconf_client.update_configuration({"min_obstacle_dist": 0.1}) # original inflation distance: 0.1
                    rospy.loginfo('[ {} ]: Inflation distance updated successfully'. format(rospy.get_name()))
                except:
                    rospy.logerr('[ {} ]: Inflation distance update Failed!'.format(rospy.get_name))
                self.result.res = True
                self.du_server.set_succeeded(self.result)
            else: 
                self.result.res = False
                self.du_server.set_aborted(self.result)

    def reset_odom(self):
        """ Service call to reset odom for motion under cart. """
        success = True
        try:
            rospy.loginfo('[ {} ]: Resetting Odom'.format(rospy.get_name()))
            rospy.wait_for_service('/'+ROBOT_ID+'/set_odometry')
            reset_odom1 = rospy.ServiceProxy('/'+ROBOT_ID+'/set_odometry', set_odometry)
            reset_odom1(0.0,0.0,0.0,0.0)
            rospy.sleep(0.2)
            rospy.loginfo('[ {} ]: Odom Reset Successful'.format(rospy.get_name()))
            return success
        except rospy.ServiceException:
            rospy.logerr('[ {} ]: Odom Reset Service call Failed!'.format(rospy.get_name()))
            success = False
            return success

    def do_du_se_move(self, distance):
        """
        Secondary motion before moving under cart using euclidean distance and a PD controller.
        The aim is to provide accurate docking with the cart and compensate for the errors
        in the target pose reached through the local planner.
        """
        success = True
        rospy.sleep(0.2)
        vel_msg = Twist()
        rospy.loginfo('[ {} ]: Navigating to Secondary Goal'.format(rospy.get_name()))
        se_distance = distance / 2.0
        goal = self.calc_se_dock_position(se_distance)  ### to be updated as user defined ratio
        goal_x = goal[0]
        goal_y = goal[1]
        r = rospy.Rate(10)
        while(self.euclidean_distance(goal_x, goal_y) >= self.distance_tolerance):
            if (self.du_server.is_preempt_requested()):
                self.du_server.set_preempted()
                rospy.logwarn('[ {} ]: Goal preempted'.format(rospy.get_name()))
                success = False
                return success
            vel_msg.linear.x = self.euclidean_distance(goal_x, goal_y) * self.kp_trans
            #print('Euclidean Distance: {}'.format(self.euclidean_distance(goal_x, goal_y)))
            vel_msg.linear.y = 0
            vel_msg.linear.z = 0
            # Angular velocity in the z-axis.
            vel_msg.angular.x = 0
            vel_msg.angular.y = 0
            vel_msg.angular.z = self.angular_vel(goal_x, goal_y)#
            #print('Angular Vel: {}'.format(self.angular_vel(goal_x, goal_y)))
            #print('-----------')
            self.vel_pub.publish(vel_msg)
            r.sleep()
        vel_msg.linear.x = 0
        vel_msg.angular.z = 0
        self.vel_pub.publish(vel_msg)
        rospy.loginfo('[ {} ]: Secondary Docking Goal Position Reached'.format(rospy.get_name()))
        # print('Theta error: {}'.format((self.calc_cart_theta()) - self.curr_theta))
        # print('Cart theta is: {}'.format(self.calc_cart_theta()))
        # print('Robot theta is: {}'.format(self.curr_theta))
        r = rospy.Rate(10) 
        # control_flag = False
        # orientation_error = self.curr_theta - self.calc_cart_theta()
        # while(abs(orientation_error) >= self.orientation_tolerance):
        #     if (self.du_server.is_preempt_requested()):
        #         self.du_server.set_preempted()
        #         rospy.logwarn('[ {} ]: Goal preempted'.format(rospy.get_name()))
        #         success = False
        #         return success
        #     #if ((self.mapping(self.calc_cart_theta()) - self.mapping(self.curr_theta)) > 3.2): #
        #     #print('Orientation error 1: {}'.format(self.mapping(self.calc_cart_theta()) - self.mapping(self.curr_theta)))
        #     #if (self.curr_theta <= self.calc_cart_theta()):
        #     #    orientation_error = self.calc_cart_theta() - self.curr_theta
        #     #else:
        #     cart_theta = self.calc_cart_theta()
        #     robot_theta = self.curr_theta
        #     orientation_error = (robot_theta - cart_theta)
        #     # if (orientation_error > 3.14):
        #     #     orientation_error = 3.14
        #     # elif (orientation_error < 3.14):
        #     #     orientation_error = -3.14
        #     # if (robot_theta > 0 and cart_theta < 0):
        #     #     vel_msg.angular.z = orientation_error * self.kp_orient
        #     # if (robot_theta < 0 and cart_theta > 0):
        #     #     vel_msg.angular.z = -1 * orientation_error * self.kp_orient
        #     # if (robot_theta > 0 and cart_theta > 0):
        #     #     if (orientation_error > 0):
        #     #         vel_msg.angular.z = orientation_error * self.kp_orient
        #     #     else:
        #     #         vel_msg.angular.z = -1 * orientation_error * self.kp_orient
        #     # if (robot_theta < 0 and cart_theta < 0):
        #     #     if (orientation_error < 0):
        #     #         vel_msg.angular.z =  -1 * orientation_error * self.kp_orient
        #     #     else:
        #     #         vel_msg.angular.z = orientation_error * self.kp_orient
        #     # if (orientation_error > 0):
        #     #     vel_msg.angular.z = -1 * orientation_error * self.kp_orient
        #     # elif (orientation_error < 0):
        #     #     vel_msg.angular.z = orientation_error * self.kp_orient
        #     vel_msg.angular.z = -1 * orientation_error * self.kp_orient
        #     # print('Cart theta is: {}'.format(cart_theta))
        #     # print('Robot theta is: {}'.format(robot_theta))
        #     # print('Orientation error: {}'.format(orientation_error))
        #     # print('Angular Vel: {}'.format(vel_msg.angular.z))
        #     # print('-------')
        #     self.vel_pub.publish(vel_msg)
        #     # control_flag = True
        #     r.sleep()
        # # if (not control_flag):
        # #     success = False
        # #     rospy.logerr('[ {} ]: Goal Orientation Not modified!'.format(rospy.get_name()))
        # #     return success
        # vel_msg.linear.x = 0
        # vel_msg.angular.z = 0
        # self.vel_pub.publish(vel_msg)
        orientation_error = self.calc_cart_theta() - self.curr_theta
        orientation_error_mod = atan2(sin(orientation_error),cos(orientation_error))
        while(abs(orientation_error_mod) >= self.orientation_tolerance):
            vel_msg.angular.z = orientation_error * self.kp_orient
            self.vel_pub.publish(vel_msg)
            orientation_error = self.calc_cart_theta() - self.curr_theta
            orientation_error_mod = atan2(sin(orientation_error),cos(orientation_error))
        rospy.loginfo('[ {} ]: Secondary Docking Goal Orientation Reached'.format(rospy.get_name()))
        return success

    def do_du_move(self, distance):
        """ 
        Final (primary) motion under cart.
        Pleae note that motion under the cart is done blindly without the use of vicon or
        on-robot sensors other than the odom.
        """
        success = True
        vel_msg = Twist()
        r = rospy.Rate(10)
        #rospy.loginfo('Current Odom value{}'.format(abs(self.odom_coor.position.x)))
        rospy.loginfo('[ {} ]: Moving under Cart'.format(rospy.get_name())) # periodic logging
        #while(abs(self.odom_coor.position.x) < distance):
        while((distance - abs(self.odom_coor.position.x)) > self.move_tolerance):
            if (self.du_server.is_preempt_requested()):
                self.du_server.set_preempted()
                rospy.logwarn('[ {} ]: Goal preempted'.format(rospy.get_name()))
                success = False
                return success
            vel_msg.linear.x = (distance - abs(self.odom_coor.position.x))*self.move_kp #self.move_speed
            vel_msg.angular.z = 0
            self.vel_pub.publish(vel_msg)
            self.feedback.odom_data = self.odom_data
            self.du_server.publish_feedback(self.feedback)
            r.sleep()
        vel_msg.linear.x = 0
        self.vel_pub.publish(vel_msg)
        rospy.loginfo('[ {} ]: Motion under Cart Successful'.format(rospy.get_name()))
        return success

    def do_du_elev(self, mode):
        """
        Raising or lowering of the elevator. The vicon reference to the robot (i.e: robot id)
        is changed to being that of the cart for further tracking of the robot (through the 
        ros_mocap package) while under the cart.
        """
        success = True
        attempts = 3
        if (self.du_server.is_preempt_requested()):
            self.du_server.set_preempted()
            rospy.logwarn('[ {} ]: Goal preempted'.format(rospy.get_name()))            
            success = False
            return success
        if (mode == True):
            elev_act = 3
        else:
            elev_act = 2 
        counter = 1
        while (counter <= attempts):
            if (counter > 1):
                rospy.loginfo('[ {} ]: Attempting to call Elevator Service again..'.format(rospy.get_name()))
            try:
                rospy.loginfo('[ {} ]: Moving Elevator'.format(rospy.get_name()))
                time_buffer = time.time()
                while (time.time() - time_buffer <= 5.2): # solution for elevator bug
                    if (self.joy_data.buttons[5] == 1 and (self.joy_data.axes[10] == 1.0 or self.joy_data.axes[10] == -1.0)): # Fuse protection
                        rospy.logwarn('[ {} ]: Elevator motion interupted by joystick!'.format(rospy.get_name()))
                        success = False
                        return success
                        #break 
                    rospy.wait_for_service('/'+ROBOT_ID+'/robotnik_base_hw/set_digital_output')
                    move_elevator = rospy.ServiceProxy('/'+ROBOT_ID+'/robotnik_base_hw/set_digital_output', set_digital_output)
                    move_elevator(elev_act,True) # 3 --> raise elevator // 2 --> lower elevator
                rospy.loginfo('[ {} ]: Elevator Service call Successful'.format(rospy.get_name()))
                break
            except rospy.ServiceException: 
                rospy.logwarn('[ {} ]: Elevator Service call Failed!'.format(rospy.get_name()))
                counter += 1
            if (counter > attempts):
                rospy.logerr('[ {} ]: Elevator Service call Failed after {} attempts!'.format(rospy.get_name(), counter-1))
                success = False
                #self.result.res = False
                #self.du_server.set_aborted(self.result)
        return success
    
    def do_du_rotate(self, angle):
        """ Execution of robot rotation around its axis. """
        success = True
        angle_quat = tf_conversions.transformations.quaternion_from_euler(0, 0, angle)
        vel_msg = Twist()
        rospy.loginfo('[ {} ]: Rotating Cart'.format(rospy.get_name()))
        r = rospy.Rate(10)
        while (abs(self.odom_coor.orientation.z) < angle_quat[2] - self.ang_tolerance):
        #while ((angle_quat[2] - abs(self.odom_coor.orientation.z)) > self.ang_tolerance):
            if (self.du_server.is_preempt_requested()):
                self.du_server.set_preempted()
                rospy.logwarn('[ {} ]: Goal preempted'.format(rospy.get_name()))
                success = False
                return success  
            vel_msg.angular.z = self.rot_speed #(angle_quat[2] - abs(self.odom_coor.orientation.z))*self.rot_kp
            self.vel_pub.publish(vel_msg)
            self.feedback.odom_data = self.odom_data
            self.du_server.publish_feedback(self.feedback)
            r.sleep()
        vel_msg.angular.z = 0
        self.vel_pub.publish(vel_msg)
        rospy.loginfo('[ {} ]: Rotation Successful'.format(rospy.get_name()))
        return success
      
    def save_cart_pose(self):
        """ Saves cart pose to enable returning it later during the return action. """
        self.reconf_client.update_configuration({"return_pose_trans_x": self.cart_pose_trans[0]})
        self.reconf_client.update_configuration({"return_pose_trans_y": self.cart_pose_trans[1]})  
        self.reconf_client.update_configuration({"return_pose_rot_x": self.cart_pose_rot[0]})  
        self.reconf_client.update_configuration({"return_pose_rot_y": self.cart_pose_rot[1]})  
        self.reconf_client.update_configuration({"return_pose_rot_z": self.cart_pose_rot[2]})  
        self.reconf_client.update_configuration({"return_pose_rot_w": self.cart_pose_rot[3]})  

    def collision_update(self, data):
        #print(data)
        self.collision_point_x = data.point.x
        self.collision_point_y = data.point.y
        self.curr_col_seq = data.header.seq

    def collision_detected(self):
        #self.last_col_seq = data.header.seq
        if (self.curr_col_seq > self.last_col_seq):
            self.last_col_seq = self.curr_col_seq
            if ((self.collision_point_x <= self.collision_tolerance_x) and (self.collision_point_y <= self.collision_tolerance_y)):
                rospy.logerr('[ {} ]: Collision Detected'.format(rospy.get_name()))
                return True
            else:
                return False

    # def mapping(self, value, leftMin=-pi, leftMax=pi, rightMin=0, rightMax=2*pi):
    #     # Figure out how 'wide' each range is
    #     leftSpan = leftMax - leftMin
    #     rightSpan = rightMax - rightMin
    #     # Convert the left range into a 0-1 range (float)
    #     valueScaled = float(value - leftMin) / float(leftSpan)
    #     # Convert the 0-1 range into a value in the right range.
    #     return rightMin + (valueScaled * rightSpan) 
    
    def get_odom(self, data):
        """ Obtains current odom readings. """
        self.odom_data = data   
        self.odom_coor = data.pose.pose

    def calc_se_dock_position(self, se_distance):
        """
        Calcuation of secondary docking position using the distance between the point calculated 
        by the dock_pose_server and the cart position.
        """
        #goal_x = self.curr_pose_trans_x + (self.cart_pose_x - self.curr_pose_trans_x)/2.0
        #goal_y = self.curr_pose_trans_y + (self.cart_pose_y - self.curr_pose_trans_y)/2.0
        # goal_x = self.curr_pose_trans_x + (self.cart_pose_trans[0] - self.curr_pose_trans_x)/2.0
        # goal_y = self.curr_pose_trans_y + (self.cart_pose_trans[1] - self.curr_pose_trans_y)/2.0
        #print('Cart Pos: ({}, {})'.format(self.cart_pose_trans[0], self.cart_pose_trans[1]))
        goal_x = self.cart_pose_trans[0] - (0.50 * cos(self.calc_cart_theta()))
        goal_y = self.cart_pose_trans[1] - (0.50 * sin(self.calc_cart_theta()))
        #print('(Goal X, Goal Y): ({}, {})'.format(goal_x, goal_y))
        return (goal_x, goal_y)

    def update_pose(self, data):
        """ Robot vicon pose update. """
        self.curr_pose_trans_x = data.transform.translation.x
        self.curr_pose_trans_y = data.transform.translation.y
        rot=[data.transform.rotation.x, data.transform.rotation.y, data.transform.rotation.z, data.transform.rotation.w]
        rot_euler = tf_conversions.transformations.euler_from_quaternion(rot)
        self.curr_theta = rot_euler[2]
    
    def update_cart_id(self, data):
        self.cart_id = data.data
        #self.control_flag = True
        rospy.loginfo_throttle(1, '[ {} ]: Cart id updated to {}'.format(rospy.get_name(), self.cart_id))
        #self.cart_id_sub.unregister()
    
    '''
    def update_cart_pose(self, data):
        """ Cart pose update for usage during the secondary motion. """
        self.cart_pose_trans = [data.transform.translation.x, data.transform.translation.y]
        #self.cart_pose_x = data.transform.translation.x
        #self.cart_pose_y = data.transform.translation.y
        self.cart_pose_rot=[data.transform.rotation.x, data.transform.rotation.y, data.transform.rotation.z, data.transform.rotation.w]
        rot_euler = tf_conversions.transformations.euler_from_quaternion(self.cart_pose_rot)
        self.cart_theta = rot_euler[2]
    '''

    def get_cart_pose(self, data):
        #rospy.loginfo_throttle(1, 'getting cart pose')
        self.cart_pose_trans = [data.transform.translation.x, data.transform.translation.y]
        self.cart_pose_rot = [data.transform.rotation.x, data.transform.rotation.y, data.transform.rotation.z, data.transform.rotation.w]
        self.cart_pose_sub.unregister()
    
    def calc_cart_theta(self):
        rot_euler = tf_conversions.transformations.euler_from_quaternion(self.cart_pose_rot)
        cart_theta = rot_euler[2]
        return cart_theta

    def joy_update(self, data):
        """ Getting joystick data for usage in case of interruption during elevator motion. """
        self.joy_data = data

    '''
    def dynamic_params_update(self, config):
        """ Obtaining of cart id dynamically as set by the previous picking action. """
        rospy.loginfo('Dock server updating parameters') ###
        #self.cart_id = config['cart_id']
        #print('Cart id obtained by dock server is: {}'.format(self.cart_id))
    '''

    def euclidean_distance(self, goal_x, goal_y):
        """ Euclidean distance between current pose and the next way point."""
        return sqrt(pow((goal_x - self.curr_pose_trans_x), 2) + pow((goal_y - self.curr_pose_trans_y), 2))

    def goal_angle(self, goal_x, goal_y):
        """ Angle between current orientation and the heading of the next way point. """
        return atan2(goal_y - self.curr_pose_trans_y, goal_x - self.curr_pose_trans_x)

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

    def shutdown_hook(self):
        self.klt_num_pub.publish('') # resets the picked up cart number in the ros_mocap package
        try:
            self.teb_reconf_client.update_configuration({"min_obstacle_dist": 0.1}) # original inflation distance: 0.1
            rospy.loginfo('[ {} ]: Inflation distance updated successfully'. format(rospy.get_name()))
        except:
            rospy.logerr('[ {} ]: Inflation distance update Failed!'.format(rospy.get_name))
        rospy.logwarn('[ {} ]: node shutdown by user'.format(rospy.get_name()))

if __name__ == '__main__':
    try:
        du = DUActionServer()
    except KeyboardInterrupt:
        sys.exit()
        #rospy.logerr('Interrupted!')
    rospy.spin()
