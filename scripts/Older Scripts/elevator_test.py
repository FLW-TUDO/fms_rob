#!/usr/bin/env python

import rospy
from robotnik_msgs.srv import set_digital_output
import time



'''
#######################################################################################
'''

ROBOT_ID = 'rb1_base_b'

'''
#######################################################################################
'''

def do_elev_test():
    try:
        rospy.wait_for_service('/'+ROBOT_ID+'/robotnik_base_hw/set_digital_output')
        do_raise_elevator = rospy.ServiceProxy('/'+ROBOT_ID+'/robotnik_base_hw/set_digital_output', set_digital_output)
        resp_raise_elevator = do_raise_elevator(3,True) # 3 --> raise elevator // 2 --> lower elevator
        #rospy.sleep(7)
    except:
        print('Service Error')

if __name__ == '__main__':
    print('Testing elevator')
    rospy.init_node('dock_test1')
    do_elev_test()
    rospy.spin()



