#!/usr/bin/env python
"""
A client that requests the navigation of the robot to its home position
infront of the cart in preparation for the docking action. It acts as a client 
to ROS's built in move base node,which is an implementation of an action server.
Please note that the status message architecture follows the Goal Status Array
type specified in ROS actions by default.
"""

import rospy
import actionlib
import sys
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Pose, TransformStamped
from fms_rob.msg import RobActionSelect, RobActionStatus
from actionlib_msgs.msg import GoalStatusArray
from std_msgs.msg import String, Bool
from std_srvs.srv import Empty
import time
import dynamic_reconfigure.client


'''
#######################################################################################
'''

ROBOT_ID = rospy.get_param('/ROBOT_ID') # by default the robot id is set in the package's launch file

'''
#######################################################################################
'''

class dyn_col_det:
    def __init__(self):
        rospy.init_node('dynamic_collision_detector')
        self.dyn_col_detector_sub = rospy.Subscriber('/'+ROBOT_ID+'/robotnik_safety_controller/warning_collision_point', PointStamped, self.collision_update)
        #self.collision_point = PointStamped()
        #self.collision_tolerance = PointStamped()
        self.dyn_collision_point_x = float('inf')
        self.dyn_collision_point_y = float('inf')
        self.dyn_collision_tolerance_x = 0.65 # <0.7
        self.dyn_collision_tolerance_y = 0.4 # 0.4
        self.curr_dyn_col_seq = 0
        self.last_dyn_col_seq = 0
    
if __name__ == '__main__':
    try:
        hac = dyn_col_det()
    except KeyboardInterrupt:
        sys.exit()
        #rospy.logerr('Interrupted!')
    rospy.spin()
