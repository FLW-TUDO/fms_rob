#!/usr/bin/env python

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


if __name__ == '__main__':
    rospy.init_node('test')
    print('testing')
    reconf_client = dynamic_reconfigure.client.Client("dynamic_reconf_server", timeout=30) # client of fms_rob dynmaic reconfigure server
    print('Updating cart config')
    reconf_client.update_configuration({"cart_id": 'KLT_KLT'})
    rospy.spin()
