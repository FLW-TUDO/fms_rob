#!/usr/bin/env python

import rospy
import actionlib
import sys, time
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Twist
from fms_rob.msg import dockUndockAction, dockUndockGoal, dockUndockFeedback, dockUndockResult
from nav_msgs.msg import Odometry
from robotnik_msgs.srv import set_odometry, set_digital_output
#from actionlib_msgs.msg import GoalStatusArray
from std_msgs.msg import String
import tf_conversions
#from std_srvs.srv import Empty
import elevator_test


'''
#######################################################################################
'''

ROBOT_ID = 'rb1_base_b'

'''
#######################################################################################
'''

class du_action_server:

    def __init__(self):
        self.du_server = actionlib.SimpleActionServer('do_dock_undock', dockUndockAction, self.execute, False)
        self.du_server.start()
        self.odom_sub = rospy.Subscriber('/'+ROBOT_ID+'/dummy_odom', Odometry, self.get_odom)
        self.vel_pub = rospy.Publisher('/'+ROBOT_ID+'/move_base/cmd_vel', Twist, queue_size=10)
        self.status_flag = False
        self.move_speed = 0.2
        self.rot_speed = 0.6
        #self.dist_tolerance = 0.005
        self.ang_tolerance = 0.002
        self.feedback = dockUndockFeedback()
        self.result = dockUndockResult()
        print('Dock-Undock Server Running')
        rospy.sleep(1)

    def execute(self, goal):
        dock_distance = goal.distance # distance to be moved under cart
        dock_angle = goal.angle # rotation angle after picking cart
        elev_mode = goal.mode
        success_move = False
        success_elev = False
        success_rotate = False
        # Reset Odom before moving under cart
        try:
            print('Resetting Odom')
            rospy.wait_for_service('/'+ROBOT_ID+'/set_odometry')
            reset_odom1 = rospy.ServiceProxy('/'+ROBOT_ID+'/set_odometry', set_odometry)
            reset_odom1(0.0,0.0,0.0,0.0)
            print('Odom Reset Successful')
        except rospy.ServiceException:
            print ('Odom Reset Service call Failed!')
        self.result.res = False
        if (elev_mode == True): # True --> Dock // False --> Undock
            success_move = self.do_du_move(dock_distance) # move under cart
            success_elev = self.do_du_elev(elev_mode) # raise/lower elevator
            success_rotate = self.do_du_rotate(dock_angle) # rotate while picking cart
        else:
            success_elev = self.do_du_elev(elev_mode) # raise/lower elevator
            success_rotate = self.do_du_rotate(dock_angle) # rotate while picking cart
            success_move = self.do_du_move(dock_distance) # move under cart
        if (success_move and success_elev and success_rotate):
            self.result.res = True
            self.du_server.set_succeeded(self.result)
        else: 
            self.result.res = False
            self.du_server.set_aborted(self.result)

            

    def do_du_move(self, distance):
        success = True
        vel_msg = Twist()
        r = rospy.Rate(10)
        while(abs(self.odom_coor.position.x) < distance):
            if (self.du_server.is_preempt_requested()):
                self.du_server.set_preempted()
                success = False
                return success
            print('Moving under Cart')
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
        success = True
        if (self.du_server.is_preempt_requested()):
            self.du_server.set_preempted()
            success = False
            return success
        if (mode == True):
            elev_act = 3
        else:
            elev_act = 2
        try:
            #self.klt_num_pub.publish('/vicon/'+self.cart_id+'/'+self.cart_id) # when robot is under cart publish entire vicon topic of cart for ros_mocap reference
            print('Moving Elevator')
            time_buffer = time.time()
            while (time.time() - time_buffer <= 5.7): # temporary solution for elevator bug (on robot b)
                rospy.wait_for_service('/'+ROBOT_ID+'/robotnik_base_hw/set_digital_output')
                move_elevator = rospy.ServiceProxy('/'+ROBOT_ID+'/robotnik_base_hw/set_digital_output', set_digital_output)
                resp_move_elevator = move_elevator(elev_act,True) # 3 --> raise elevator // 2 --> lower elevator
                # rospy.sleep(7) # service returns immedietly, a wait time is needed#
            #execfile('src/fms_rob/scripts/elevator_test.py')
            #elevator_test.do_elev_test()
            print('Elevator Raise Successful')
            success = True
        except: 
            print ('Elevator Raise Service call Failed!')
            success = False
        return success
    
    def do_du_rotate(self, angle):
        success = True
        angle_quat = tf_conversions.transformations.quaternion_from_euler(0, 0, angle)
        print(angle_quat)
        vel_msg = Twist()
        print('Rotating Cart')
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
        print('Rotation Successful')
        return success

    def get_odom(self, data):
        self.odom_data = data
        self.odom_coor = data.pose.pose

    
if __name__ == '__main__':
    try:
        rospy.init_node('dock_undock_server')
        du = du_action_server()
    except KeyboardInterrupt:
        sys.exit()
        print('Interrupted!')
    rospy.spin()
