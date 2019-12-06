#!/usr/bin/env python
"""
A node to detect disturbances in scanner readings.
"""

import rospy
from std_msgs.msg import Bool
from sensor_msgs.msg import LaserScan
import time, sys

'''
###########################################################################
'''

ROBOT_ID = rospy.get_param('/ROBOT_ID')

'''
###########################################################################
'''

class scanner_checker:

    def __init__(self):
        rospy.init_node('scanner_check')
        self.vicon_subscriber = rospy.Subscriber('/'+ROBOT_ID+'/scan', LaserScan, self.scan_update)
        self.check_pub = rospy.Publisher('/'+ROBOT_ID+'/laser_scanner_check', Bool, queue_size=10)
        self.check_msg = Bool()
        self.time_limit = 1
        self.scanner_data = LaserScan()
        self.pub_flag = False
        rospy.sleep(2)
        self.scanner_topic_check()

    def scan_update(self, data):
        self.scanner_data = data.header
        self.pub_flag = True

    def scanner_topic_check(self):
        topic = ['/'+ROBOT_ID+'/scan', 'sensor_msgs/LaserScan']
        last_seq_scanner = self.scanner_data.header.seq
        if(topic not in rospy.get_published_topics('/'+ROBOT_ID)):
            self.check_msg.data = False
            self.check_pub.publish(self.check_msg)
            print('Scanner Topic Not Registered!') # When Laser scanner node on robot is started without publishing the topic
            return
        else:
            #r = rospy.Rate(10)
            while (True):
                self.scanner_data.header.seq == last_seq_scanner
                timer = time.time()
                while (self.scanner_data.header.seq == last_seq_scanner):
                    rospy.sleep(0.2)
                    if((time.time() - timer) >= self.time_limit):
                        self.check_msg.data = False
                        self.check_pub.publish(self.check_msg)
                        print('Scanner Topic Stopped!') # When topic was already publishing
                    #r.sleep()
                    return
                else:
                    self.check_msg.data = True
                    self.check_pub.publish(self.check_msg)
                    print('Scanner is Online')
                    return

if __name__ == '__main__':
    try:
        x = scanner_checker()
    except KeyboardInterrupt:
        sys.exit()
        rospy.logerr('Interrupted!')
    rospy.spin()

