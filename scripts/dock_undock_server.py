#!/usr/bin/env python
'''
An action server to execute both the docking and undocking operations.
The docking operation is composed of moving under cart (in 2 phases), lifting 
the elevator, then rotation the cart. Undocking is composed of lowering the elevator, rotating 
the robot under the cart, then moving out from under the cart. 
Please note that the docking operation is executed after the robot has been 
positioned in the picking location.
'''

import rospy
import actionlib
import sys, time
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseStamped, TransformStamped, Twist
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

ROBOT_ID = rospy.get_param('/ROBOT_ID', 'rb1_base_b') # by default the robot id is set in the package's launch file

'''
#######################################################################################
'''

class du_action_server:

    def __init__(self):
        self.du_server = actionlib.SimpleActionServer('do_dock_undock', dockUndockAction, self.execute, False) # create dock-undock action server
        self.du_server.start()
        self.odom_sub = rospy.Subscriber('/'+ROBOT_ID+'/dummy_odom', Odometry, self.get_odom) # dummy odom is the remapped odom topic - please check ros_mocap package
        self.vel_pub = rospy.Publisher('/'+ROBOT_ID+'/move_base/cmd_vel', Twist, queue_size=10)
        self.klt_num_pub = rospy.Publisher('/'+ROBOT_ID+'/klt_num', String, queue_size=10) # used for interfacing with the ros_mocap package
        self.pose_subscriber = rospy.Subscriber('/vicon/'+ROBOT_ID+'/'+ROBOT_ID, TransformStamped, self.update_pose)
        self.joystick_sub = rospy.Subscriber('/'+ROBOT_ID+'/joy', Joy, self.joy_update)
        self.move_speed = 0.2
        #self.se_move_speed = 0.2
        #self.se_move_p_gain = 0.6
        self.rot_speed = 0.7
        #self.dist_tolerance = 0.005
        self.ang_tolerance = 0.002
        self.feedback = dockUndockFeedback()
        self.result = dockUndockResult()
        ''' PD-Controller settings for secondary move'''
        self.error_theta = 1.0
        self.kp_ang = 0.7 #0.7
        self.kd_ang = 0.1 #0.1
        self.kp_orient = 0.5
        self.kp_trans = 0.8
        self.distance_tolerance = 0.005
        self.orientation_tolerance = 0.01
        current_time = None
        self.sample_time = 0.0001
        self.current_time = current_time if current_time is not None else time.time()
        self.last_time = self.current_time
        self.p_term_ang = 0.0
        self.d_term_ang = 0.0
        self.last_error_theta = 0.0
        self.output = 0.0
        #self.kp_lin = 0.8
        self.start_msg = Bool()
        self.theta_msg = Float32()
        '''Create dynamic reconfigure client client to obtain cart id'''
        reconf_client = dynamic_reconfigure.client.Client("dynamic_reconf_server", timeout=30, config_callback=self.dynamic_params_update)
        rospy.sleep(1)
        rospy.on_shutdown(self.shutdown_hook) # used to reset the interface with the ros_mocap package
        rospy.loginfo('Dock-Undock Server Ready')

    def execute(self, goal):
        self.klt_sub = rospy.Subscriber('/vicon/'+self.cart_id+'/'+self.cart_id, TransformStamped, self.update_cart_pose)
        dock_distance = goal.distance # distance to be moved under cart
        dock_angle = goal.angle # rotation angle after picking cart
        elev_mode = goal.mode
        success_move = False
        success_se_move = False # secondary motion to adjust docking distance
        success_elev = False
        success_rotate = False
        self.result.res = False
        if (elev_mode == True): # True --> Dock // False --> Undock
            success_se_move = self.do_du_se_move(dock_distance) # pre-motion before cart
            rospy.sleep(0.2) # wait for complete halt of robot
            self.reset_odom()
            success_move = self.do_du_move(dock_distance/2.0) # move under cart
            success_elev = self.do_du_elev(elev_mode) # raise/lower elevator
            success_rotate = self.do_du_rotate(dock_angle) # rotate while picking cart
        else:
            success_elev = self.do_du_elev(elev_mode) # raise/lower elevator
            self.reset_odom()
            success_rotate = self.do_du_rotate(dock_angle) # rotate while picking cart
            success_move = self.do_du_move(dock_distance) # move under cart
        if (success_move and success_elev and success_rotate):
            self.result.res = True
            self.du_server.set_succeeded(self.result)
        else: 
            self.result.res = False
            self.du_server.set_aborted(self.result)

    def reset_odom(self):
        '''
        Service call to reset odom for motion under cart
        '''
        try:
            rospy.loginfo('Resetting Odom')
            rospy.wait_for_service('/'+ROBOT_ID+'/set_odometry')
            reset_odom1 = rospy.ServiceProxy('/'+ROBOT_ID+'/set_odometry', set_odometry)
            reset_odom1(0.0,0.0,0.0,0.0)
            rospy.sleep(0.2)
            rospy.loginfo('Odom Reset Successful')
        except rospy.ServiceException:
            rospy.logerr('Odom Reset Service call Failed!')

    def do_du_se_move(self, distance):
        '''
        Secondary motion before moving under cart using euclidean distance and a PD controller.
        The aim is to provide accurate docking with the cart and compensate for the errors
        in the target pose reached through the local planner.
        '''
        #if(data.data == True):
        success = True
        rospy.sleep(0.2)
        vel_msg = Twist()
        #heading_tolerance = 0.5
        rospy.loginfo('Navigating to Secondary Goal')
        goal = self.calc_se_dock_position(distance)
        goal_x = goal[0]
        goal_y = goal[1]
        r = rospy.Rate(10)
        while(self.euclidean_distance(goal_x, goal_y) >= self.distance_tolerance):
            if (self.du_server.is_preempt_requested()):
                self.du_server.set_preempted()
                success = False
                return success
            #vel_msg.linear.x = (self.euclidean_distance(goal_x, goal_y))*self.se_move_p_gain
            vel_msg.linear.x = self.euclidean_distance(goal_x, goal_y)*self.kp_trans
            vel_msg.linear.y = 0
            vel_msg.linear.z = 0
            # Angular velocity in the z-axis.
            vel_msg.angular.x = 0
            vel_msg.angular.y = 0
            vel_msg.angular.z = self.angular_vel(goal_x, goal_y)
            self.vel_pub.publish(vel_msg)
            r.sleep()
        vel_msg.linear.x = 0
        vel_msg.angular.z = 0
        self.vel_pub.publish(vel_msg)
        rospy.loginfo('Secondary Docking Goal Reached')
        #rospy.sleep(0.7)
        #self.start_subscriber.unregister()
        r = rospy.Rate(10)
        while(abs(self.cart_theta - self.curr_theta) >= self.orientation_tolerance):
            if (self.du_server.is_preempt_requested()):
                self.du_server.set_preempted()
                success = False
                return success
            vel_msg.angular.z = (self.cart_theta - self.curr_theta)*self.kp_orient
            self.vel_pub.publish(vel_msg)
            r.sleep()
        vel_msg.linear.x = 0
        vel_msg.angular.z = 0
        self.vel_pub.publish(vel_msg)
        rospy.loginfo('Secondary Docking Goal Orientation Reached')
        #else:
        #    pass
        return success

    def do_du_move(self, distance):
        ''' 
        Final (primary) motion under cart.
        Pleae note that motion under the cart is done blindly without the use of vicon or
        on-robot sensors other than the odom.
         '''
        success = True
        vel_msg = Twist()
        r = rospy.Rate(10)
        rospy.loginfo('Current Odom value{}'.format(abs(self.odom_coor.position.x)))
        while(abs(self.odom_coor.position.x) < distance):
            if (self.du_server.is_preempt_requested()):
                self.du_server.set_preempted()
                success = False
                return success
            rospy.loginfo_throttle(1, 'Moving under Cart') # periodic logging
            vel_msg.linear.x = self.move_speed
            vel_msg.angular.z = 0
            self.vel_pub.publish(vel_msg)
            self.feedback.odom_data = self.odom_data
            self.du_server.publish_feedback(self.feedback)
            r.sleep()
        vel_msg.linear.x = 0
        self.vel_pub.publish(vel_msg)
        #if ((self.odom_coor.position.x <= distance + self.dist_tolerance) and (self.odom_coor.position.x >= distance - self.dist_tolerance)):
        #    success = True
        #else:
        #    success = False
        return success

    def do_du_elev(self, mode):
        '''
        Raising or lowering of the elevator. The vicon reference to the robot (i.e: robot id)
        is changed to being that of the cart for further tracking of the robot using the 
        ros_mocap packagewhile under the cart.
        '''
        success = True
        if (self.du_server.is_preempt_requested()):
            self.du_server.set_preempted()
            success = False
            return success
        if (mode == True):
            elev_act = 3 # 1
        else:
            elev_act = 2 # -1
        try:
            self.klt_num_pub.publish('/vicon/'+self.cart_id+'/'+self.cart_id) # when robot is under cart publish entire vicon topic of cart for ros_mocap reference
            rospy.loginfo('Moving Elevator')
            time_buffer = time.time()
            while (time.time() - time_buffer <= 5.7): # solution for elevator bug (on robot b)
                if (self.joy_data.buttons[5] == 1 and (self.joy_data.axes[10] == 1.0 or self.joy_data.axes[10] == -1.0)): # Fuse protection
                    rospy.logwarn('Elevator motion interupted by joystick!')
                    break 
                rospy.wait_for_service('/'+ROBOT_ID+'/robotnik_base_hw/set_digital_output')
                move_elevator = rospy.ServiceProxy('/'+ROBOT_ID+'/robotnik_base_hw/set_digital_output', set_digital_output)
                move_elevator(elev_act,True) # 3 --> raise elevator // 2 --> lower elevator
                #rospy.sleep(0.2)
            '''
            rospy.wait_for_service('/'+ROBOT_ID+'/set_elevator')
            move_elevator = rospy.ServiceProxy('/'+ROBOT_ID+'/set_elevator', SetElevator)
            move_elevator(elev_act) # 1 --> raise elevator // -1 --> lower elevator
            rospy.sleep(6) # service returns immedietly, a wait time is needed
            '''
            #execfile('src/fms_rob/scripts/elevator_test.py')
            #elevator_test.do_elev_test()
            #rospy.sleep(6)
            rospy.loginfo('Elevator Service call Successful')
            success = True
        except: 
            rospy.logerr('Elevator Service call Failed!')
            success = False
        return success
    
    def do_du_rotate(self, angle):
        '''
        Execution of robot rotation around its axis
        '''
        success = True
        angle_quat = tf_conversions.transformations.quaternion_from_euler(0, 0, angle)
        #print(angle_quat)
        vel_msg = Twist()
        rospy.loginfo('Rotating Cart')
        r = rospy.Rate(10)
        #while not ((self.odom_coor.orientation.z <=  angle_quat[2] + self.ang_tolerance) and (self.odom_coor.orientation.z >=  angle_quat[2] - self.ang_tolerance)):
        while (abs(self.odom_coor.orientation.z) < angle_quat[2] - self.ang_tolerance):
            if (self.du_server.is_preempt_requested()):
                self.du_server.set_preempted()
                success = False
                return success  
            vel_msg.angular.z = self.rot_speed
            self.vel_pub.publish(vel_msg)
            self.feedback.odom_data = self.odom_data
            self.du_server.publish_feedback(self.feedback)
            r.sleep()
        vel_msg.angular.z = 0
        self.vel_pub.publish(vel_msg)
        rospy.loginfo('Rotation Successful')
        return success

    def get_odom(self, data):
        self.odom_data = data   
        self.odom_coor = data.pose.pose

    def calc_se_dock_position(self, se_distance):
        '''
        Calcuation of secondary docking position using the distance between the point calculated 
        by the dock_pose_server and the cart position
        '''
        goal_x = self.curr_pose_trans_x + (self.cart_pose_x - self.curr_pose_trans_x)/2.0
        goal_y = self.curr_pose_trans_y + (self.cart_pose_y - self.curr_pose_trans_y)/2.0
        return (goal_x, goal_y)

    def update_pose(self, data):
        '''
        Robot vicon pose update
        '''
        self.curr_pose_trans_x = data.transform.translation.x
        self.curr_pose_trans_y = data.transform.translation.y
        rot=[data.transform.rotation.x, data.transform.rotation.y, data.transform.rotation.z, data.transform.rotation.w]
        rot_euler = tf_conversions.transformations.euler_from_quaternion(rot)
        self.curr_theta = rot_euler[2]
    
    def update_cart_pose(self, data):
        '''
        Cart pose update for usage during the secondary motion
        '''
        self.cart_pose_x = data.transform.translation.x
        self.cart_pose_y = data.transform.translation.y
        rot=[data.transform.rotation.x, data.transform.rotation.y, data.transform.rotation.z, data.transform.rotation.w]
        rot_euler = tf_conversions.transformations.euler_from_quaternion(rot)
        self.cart_theta = rot_euler[2]

    def joy_update(self, data):
        '''
        Getting joystick data for use in case of interruption during elevator motion
        '''
        self.joy_data = data
    
    def dynamic_params_update(self, config):
        '''
        Obtaining of cart id dynamically as set by the previous picking action
        '''
        rospy.loginfo("Config set to {cart_id}".format(**config))
        self.cart_id = config['cart_id']

    def euclidean_distance(self, goal_x, goal_y):
        '''
        Euclidean distance between current pose and the next way point.
        '''
        return sqrt(pow((goal_x - self.curr_pose_trans_x), 2) + pow((goal_y - self.curr_pose_trans_y), 2))

    def goal_angle(self, goal_x, goal_y):
        '''Angle between current orientation and the heading of the next way point'''
        return atan2(goal_y - self.curr_pose_trans_y, goal_x - self.curr_pose_trans_x)

    def angular_vel(self, goal_x, goal_y):
        '''
        PD controller angle output calculation
        '''
        current_time = None
        self.error_theta= self.goal_angle(goal_x, goal_y) - self.curr_theta
        self.error_theta= atan2(sin(self.error_theta),cos(self.error_theta))
        self.theta_msg = self.error_theta
        #self.theta_pub.publish(self.theta_msg)
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
        rospy.logwarn('Dock Undock Server node shutdown by user')
    
if __name__ == '__main__':
    try:
        rospy.init_node('dock_undock_server')
        du = du_action_server()
    except KeyboardInterrupt:
        sys.exit()
        rospy.logerr('Interrupted!')
    rospy.spin()
