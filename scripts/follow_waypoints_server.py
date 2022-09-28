#!/usr/bin/env python
"""
An action server to follow a list of provided waypoints without using move base node or
the associated local or global planners.
"""

import rospy
import actionlib
import sys, time
from geometry_msgs.msg import PoseStamped, TransformStamped, Twist, PointStamped
from fms_rob.msg import followWaypointsAction, followWaypointsGoal, followWaypointsFeedback, followWaypointsResult
from sensor_msgs.msg import Joy
from nav_msgs.msg import Odometry
from robotnik_msgs.srv import set_odometry, set_digital_output
from rb1_base_msgs.srv import SetElevator
from std_msgs.msg import String, Bool, Float32
from math import pow, atan2, sqrt, cos, sin, pi
import tf_conversions
import dynamic_reconfigure.client


'''
#######################################################################################
'''

ROBOT_ID = rospy.get_param('/ROBOT_ID') # by default the robot id is set in the package's launch file

'''
#######################################################################################
'''

class FollowWPActionServer:

    def __init__(self):
        rospy.init_node('follow_waypoints_server')
        try:
            self.follow_waypoints_server = actionlib.SimpleActionServer('do_follow_waypoints', followWaypointsAction, self.execute, False) # create dock-undock action server
        except:
            rospy.logerr('[ {} ]: Error Creating Action Server!'.format(rospy.get_name()))
        self.follow_waypoints_server.start()
        # self.odom_sub = rospy.Subscriber('/'+ROBOT_ID+'/dummy_odom', Odometry, self.get_odom) # dummy odom is the remapped odom topic - please check ros_mocap package
        self.vel_pub = rospy.Publisher('/'+ROBOT_ID+'/move_base/cmd_vel', Twist, queue_size=10)
        self.klt_num_pub = rospy.Publisher('/'+ROBOT_ID+'/klt_num', String, queue_size=10) # used for interfacing with the ros_mocap package
        self.klt_num_sub = rospy.Subscriber('/'+ROBOT_ID+'/klt_num', String, self.klt_update)
        self.pose_sub = rospy.Subscriber('/vicon/'+ROBOT_ID+'/'+ROBOT_ID, TransformStamped, self.update_vicon_pose) ####
        # self.joystick_sub = rospy.Subscriber('/'+ROBOT_ID+'/joy', Joy, self.joy_update)
        try:
            self.reconf_client = dynamic_reconfigure.client.Client('dynamic_reconf_server', timeout=30) # client of fms_rob dynmaic reconfigure server
        except:
            rospy.logerr('Dynamic Reconf Server is Not running!')
        # try:
        #     self.teb_reconf_client = dynamic_reconfigure.client.Client('/'+ROBOT_ID+'/move_base/TebLocalPlannerROS', timeout=30)
        # except:
        #     rospy.logerr('TEB Planner is Not running!')
        '''collision detector settings'''
        # self.col_detector_sub = rospy.Subscriber('/'+ROBOT_ID+'/robotnik_safety_controller/warning_collision_point', PointStamped, self.collision_update)
        #self.collision_point = PointStamped()
        #self.collision_tolerance = PointStamped()
        self.collision_point_x = float('inf')
        self.collision_point_y = float('inf')
        self.collision_tolerance_x = 0.7 # 0.75
        self.collision_tolerance_y = 0.02 # 0.4
        self.curr_col_seq = 0
        self.last_col_seq = 0
        ''' P-Controller settings for primary motion '''
        self.robot_speed = 0.9
        self.heading_tolerance = 0.7
        self.distance_tolerance = 0.3

        #self.move_speed = 0.09 #0.14
        self.move_kp = 0.99 #0.99
        #self.rot_kp = 0.99 #0.99
        #self.rot_speed = 0.4 #0.5
        self.move_tolerance = 0.007 #0.005
        #self.ang_tolerance = 0.002 #0.002
        self.feedback = followWaypointsFeedback()
        self.result = followWaypointsResult()
        ''' PD-Controller settings for secondary move '''
        self.error_theta = 1.0 #1.0
        #self.theta_tolerance = 0.007 #0.007
        #self.theta_tolerance = 0.0''2
        self.kp_ang = 0.7 #0.7 
        self.kd_ang = 0.1 #0.1
        self.kp_orient = 0.6 #0.3
        self.kp_trans = 0.8 #0.8    ######
        # self.distance_tolerance = 0.003 #0.003
        self.orientation_tolerance = 0.009 #0.01 #0.02
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

    def execute(self, data):
        self.action = data.action
        print('Action', self.action)
        self.control_flag = False            
        try:
           self.cart_pose_sub = rospy.Subscriber(self.klt_num, TransformStamped, self.update_cart_pose) # obtaining picked cart id pose
           print('klt num', self.klt_num)
        except:
            print('No cart ID')

        rospy.sleep(1.5)
        # self.cart_id_sub = rospy.Subscriber('/'+ROBOT_ID+'/pick_cart_id', String, self.update_cart_id) # obtaining cart id from picking node
        # rospy.sleep(1.5)
        # self.cart_pose_sub = rospy.Subscriber('/vicon/'+self.cart_id+'/'+self.cart_id, TransformStamped, self.update_cart_pose) # obtaining picked cart id pose
        # rospy.sleep(1.5)
        self.result.res = False

        Xwaypoints = data.Xwaypoints
        Ywaypoints = data.Ywaypoints
        success_follow = False
        vel_msg = Twist()

        #print('Current X:' + str(self.curr_pose_trans_x), '\t' 'Current Y: ' + str(self.curr_pose_trans_y), '\t' 'Current Theta: ' + str(self.curr_theta))
            
        for wp in zip(Xwaypoints, Ywaypoints):
            while(self.euclidean_distance(wp[0], wp[1]) >= self.distance_tolerance):
                while((abs(self.error_theta) > self.heading_tolerance)): 
                    #print(wp)
                    # Linear velocity in the x-axis.
                    vel_msg.linear.x = 0
                    vel_msg.linear.y = 0
                    vel_msg.linear.z = 0
                    # Angular velocity in the z-axis.
                    vel_msg.angular.x = 0
                    vel_msg.angular.y = 0
                    vel_msg.angular.z = self.angular_vel(wp[0], wp[1])
                    # Publishing our vel_msg
                    self.vel_pub.publish(vel_msg)
                vel_msg.linear.x = self.robot_speed
                vel_msg.linear.y = 0
                vel_msg.linear.z = 0
                # Angular velocity in the z-axis.
                vel_msg.angular.x = 0
                vel_msg.angular.y = 0
                vel_msg.angular.z = self.angular_vel(wp[0], wp[1])
                # Publishing our vel_msg
                self.vel_pub.publish(vel_msg)
        vel_msg.linear.x = 0
        vel_msg.angular.z = 0
        self.vel_pub.publish(vel_msg)
        success_follow = True

        if success_follow:
            # try:
            #     self.teb_reconf_client.update_configuration({"min_obstacle_dist": 0.3}) # increase obstacle inflation distance after carrying cart
            #     rospy.loginfo('[ {} ]: Inflation distance updated successfully'. format(rospy.get_name()))
            # except:
            #     rospy.logerr('[ {} ]: Inflation distance update Failed!'.format(rospy.get_name))
            self.result.res = True
            self.follow_waypoints_server.set_succeeded(self.result)
        else: 
            self.result.res = False
            self.follow_waypoints_server.set_aborted(self.result)

    def update_vicon_pose(self, data):
        """ Robot vicon pose update. """
        self.vicon_pose_trans_x = data.transform.translation.x
        self.vicon_pose_trans_y = data.transform.translation.y
        #print(self.curr_pose_trans_x, self.curr_pose_trans_y)
        rot=[data.transform.rotation.x, data.transform.rotation.y, data.transform.rotation.z, data.transform.rotation.w]
        rot_euler = tf_conversions.transformations.euler_from_quaternion(rot)
        self.vicon_theta = rot_euler[2]
        self.curr_pose_trans_x = self.vicon_pose_trans_x
        self.curr_pose_trans_y = self.vicon_pose_trans_y
        self.curr_theta = self.vicon_theta    

    def klt_update(self, data):
        self.klt_num = data.data

    # def update_cart_id(self, data):
    #     self.cart_id = data.data
    #     #self.control_flag = True
    #     rospy.loginfo_throttle(1, '[ {} ]: Cart id updated to {}'.format(rospy.get_name(), self.cart_id))
    #     #self.cart_id_sub.unregister()

    def update_cart_pose(self, data):
        #rospy.loginfo_throttle(1, 'getting cart pose')
        cart_pose_trans_x = data.transform.translation.x
        cart_pose_trans_y = data.transform.translation.y
        rot = [data.transform.rotation.x, data.transform.rotation.y, data.transform.rotation.z, data.transform.rotation.w]
        rot_euler = tf_conversions.transformations.euler_from_quaternion(rot)
        cart_theta = rot_euler[2] 
        # self.cart_pose_sub.unregister()
        self.curr_pose_trans_x = cart_pose_trans_x
        self.curr_pose_trans_y = cart_pose_trans_y
        self.curr_theta = cart_theta
        rospy.loginfo_throttle(1, 'Cart pose is being updated!')
        if self.action == 'home':
            self.cart_pose_sub.unregister()
            print('Topic unregistered')

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
            #self.teb_reconf_client.update_configuration({"min_obstacle_dist": 0.1}) # original inflation distance: 0.1
            rospy.loginfo('[ {} ]: Inflation distance updated successfully'. format(rospy.get_name()))
        except:
            rospy.logerr('[ {} ]: Inflation distance update Failed!'.format(rospy.get_name))
        rospy.logwarn('[ {} ]: node shutdown by user'.format(rospy.get_name()))

if __name__ == '__main__':
    try:
        fwp = FollowWPActionServer()
    except KeyboardInterrupt:
        sys.exit()
        #rospy.logerr('Interrupted!')
    rospy.spin()
