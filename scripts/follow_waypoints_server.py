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
from fms_rob.srv import dockPose

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
        self.action = None
        try:
            self.follow_waypoints_server = actionlib.SimpleActionServer('do_follow_waypoints', followWaypointsAction, self.execute, False) # create dock-undock action server
        except:
            rospy.logerr('[ {} ]: Error Creating Action Server!'.format(rospy.get_name()))
        self.follow_waypoints_server.start()
        self.klt_num = ''
        # self.odom_sub = rospy.Subscriber('/'+ROBOT_ID+'/dummy_odom', Odometry, self.get_odom) # dummy odom is the remapped odom topic - please check ros_mocap package
        self.vel_pub = rospy.Publisher('/'+ROBOT_ID+'/move_base/cmd_vel', Twist, queue_size=10)
        self.klt_num_pub = rospy.Publisher('/'+ROBOT_ID+'/klt_num', String, queue_size=10) # used for interfacing with the ros_mocap package
        self.klt_num_sub = rospy.Subscriber('/'+ROBOT_ID+'/klt_num', String, self.klt_update)
        self.pose_sub = rospy.Subscriber('/vicon/'+ROBOT_ID+'/'+ROBOT_ID, TransformStamped, self.update_vicon_pose) ####
        self.cart_id_sub = rospy.Subscriber('/'+ROBOT_ID+'/pick_cart_id', String, self.get_cart_id) # cart id subscribed from pick client - must be latced for future subscribers
        # self.joystick_sub = rospy.Subscriber('/'+ROBOT_ID+'/joy', Joy, self.joy_update)
        try:
            self.reconf_client = dynamic_reconfigure.client.Client('dynamic_reconf_server', timeout=30) # client of fms_rob dynmaic reconfigure server
        except:
            rospy.logerr('Dynamic Reconf Server is Not running!')
        # try:
        #     self.teb_reconf_client = dynamic_reconfigure.client.Client('/'+ROBOT_ID+'/move_base/TebLocalPlannerROS', timeout=30)
        # except:
        #     rospy.logerr('TEB Planner is Not running!')
       
        ''' P-Controller settings for orientation modification '''
        self.robot_speed = 0.9
        self.heading_tolerance = 0.7
        self.distance_tolerance = 0.3
        self.cart_orientation_tolerance = 0.1 #0.2
        self.kp_orientation = 0.7
        self.orientation_error_theta = 1.0
    
        self.feedback = followWaypointsFeedback()
        self.result = followWaypointsResult()
        
        ''' PD-Controller settings for waypoints following '''
        self.error_theta = 1.0 #1.0
        self.kp_ang = 0.7 #0.7 
        self.kd_ang = 0.1 #0.1
        current_time = None
        self.sample_time = 0.0001
        self.current_time = current_time if current_time is not None else time.time()
        self.last_time = self.current_time
        self.p_term_ang = 0.0
        self.d_term_ang = 0.0
        self.last_error_theta = 0.0
        self.output = 0.0      
        rospy.sleep(1)
        rospy.on_shutdown(self.shutdown_hook) # used to reset the interface with the ros_mocap package
        rospy.loginfo('[ {} ]: Ready'.format(rospy.get_name()))

    def execute(self, data):
        station_id = data.station_id
        print('station id', station_id)
        self.action = data.action
        print('Action', self.action)
        self.control_flag = False   
        self.orientation_error_theta = 1.0         
        try:
            self.cart_pose_sub = rospy.Subscriber(self.klt_num, TransformStamped, self.update_cart_pose) # obtaining picked cart id pose
            print('klt num', self.klt_num)
        except:
            print('No cart ID')

        rospy.sleep(1.5)
        self.result.res = False

        Xwaypoints = data.Xwaypoints
        Ywaypoints = data.Ywaypoints
        success_follow = False
        vel_msg = Twist()
            
        for wp in zip(Xwaypoints, Ywaypoints):
            while(self.euclidean_distance(wp[0], wp[1]) >= self.distance_tolerance):
                while((abs(self.error_theta) > self.heading_tolerance)): 
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

        if self.action == 'return' :
            theta_goal = self.get_cart_return_orientation()
        if self.action == 'place' :
            theta_goal = self.get_cart_workstation_orientation(station_id)
            
        if success_follow:
            if self.action == 'return' or self.action == 'place':
                rospy.loginfo('[ {} ]: Correcting Orientation'. format(rospy.get_name())) 
                print(theta_goal)    
                print(self.orientation_error_theta)
                while((abs(self.orientation_error_theta) > self.cart_orientation_tolerance)): 
                    # Linear velocity in the x-axis.
                    vel_msg.linear.x = 0
                    vel_msg.linear.y = 0
                    vel_msg.linear.z = 0
                    # Angular velocity in the z-axis.
                    vel_msg.angular.x = 0
                    vel_msg.angular.y = 0
                    vel_msg.angular.z = self.orientation_controller(theta_goal)
                    # Publishing our vel_msg
                    self.vel_pub.publish(vel_msg)
                rospy.loginfo('[ {} ]: Orientation Corrected Successfully!'. format(rospy.get_name()))
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
        #rospy.loginfo_throttle(1, 'Cart pose is being updated!')
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
    
    def get_cart_return_orientation(self):
        cart_ortientation_x = rospy.get_param('/'+ROBOT_ID+'/dynamic_reconf_server/return_pose_rot_x')
        cart_ortientation_y = rospy.get_param('/'+ROBOT_ID+'/dynamic_reconf_server/return_pose_rot_y')
        cart_ortientation_z = rospy.get_param('/'+ROBOT_ID+'/dynamic_reconf_server/return_pose_rot_z')
        cart_ortientation_w = rospy.get_param('/'+ROBOT_ID+'/dynamic_reconf_server/return_pose_rot_w')
        rot_euler = tf_conversions.transformations.euler_from_quaternion([cart_ortientation_x, cart_ortientation_y, cart_ortientation_z, cart_ortientation_w])
        cart_return_theta = rot_euler[2]  
        return cart_return_theta
    
    def get_cart_workstation_orientation(self, station_id):
        self.station_pose = TransformStamped()
        station_pose_sub = rospy.Subscriber('/vicon/'+station_id+'/'+station_id, TransformStamped, self.get_station_pose)
        rospy.sleep(1) #waits for completion of topic subscribtion   
        station_pose_quat = [self.station_pose.transform.rotation.x, self.station_pose.transform.rotation.y, self.station_pose.transform.rotation.z, self.station_pose.transform.rotation.w]
        rot_euler = tf_conversions.transformations.euler_from_quaternion(station_pose_quat)
        station_theta = rot_euler[2]
        return station_theta

    def get_station_pose(self, data):
        """ Returns the location of the station in Vicon. """
        self.station_pose = data

    def orientation_controller(self, theta):
        self.orientation_error_theta = theta - self.curr_theta
        self.orientation_error_theta = atan2(sin(self.orientation_error_theta), cos(self.orientation_error_theta))
        orientation_control_signal = self.orientation_error_theta * self.kp_orientation
        return orientation_control_signal

    def shutdown_hook(self):
        self.klt_num_pub.publish('') # resets the picked up cart number in the ros_mocap package
        try:
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
