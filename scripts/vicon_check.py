#!/usr/bin/env python
"""
A node to detect disturbances in vicon readings - currently not complete.
"""

import rospy
from std_msgs.msg import Bool
from geometry_msgs.msg import TransformStamped
import time, sys

'''
###########################################################################
'''

ROBOT_ID = rospy.get_param('/ROBOT_ID')

'''
###########################################################################
'''

class vicon_checker:

    def __init__(self):
        rospy.init_node('vicon_check')
        self.vicon_subscriber = rospy.Subscriber('/vicon/'+ROBOT_ID+'/'+ROBOT_ID, TransformStamped, self.vi_update)
        self.check_pub = rospy.Publisher('/vicon/'+ROBOT_ID+'/'+'check', Bool, queue_size=10)
        self.check_msg = Bool()
        self.time_limit = 2
        self.vicon_data = TransformStamped()
        self.pub_flag = False
        rospy.sleep(2)
        self.vi_topic_check()

    def vi_update(self, data):
        self.vicon_data = data
        self.pub_flag = True

    def vi_topic_check(self):
        topic = ['/vicon/rb1_base_b/rb1_base_b', 'geometry_msgs/TransformStamped']
        if(topic not in rospy.get_published_topics('/vicon/')):
            self.check_msg.data = False
            self.check_pub.publish(self.check_msg)
            print('Vicon Topic Not Registered!') # When vicon bridge on robot died or nexus was started without selecting the topic
            return
        else:
            self.vi_status_check()

    def vi_status_check(self):
        #rospy.sleep(1)
        #r = rospy.Rate(10)
        last_seq_vicon = self.vicon_data.header.seq
        while(True):
            #print(self.vicon_data)
            if(self.pub_flag == False):
                self.check_msg.data = False
                self.check_pub.publish(self.check_msg)
                print('Vicon is Offline!') # When topic is registered in master but was not published on yet
                return
            elif(self.vicon_data.header.seq == last_seq_vicon):
                timer = time.time()
                #last_seq_vicon = self.vicon_data.header.seq
                while(self.vicon_data.header.seq == last_seq_vicon):
                    rospy.sleep(0.2)
                    if((time.time() - timer) >= self.time_limit):
                        self.check_msg.data = False
                        self.check_pub.publish(self.check_msg)
                        print('Vicon Connection Lost!') # When topic was already publishing
                        return
            else:
                self.check_msg.data = True
                self.check_pub.publish(self.check_msg)
                print('Vicon is Online')
                return  
            #r.sleep()
            rospy.sleep(0.2)

if __name__ == '__main__':
    try:
        #print(rospy.get_published_topics('/vicon/'+ROBOT_ID+'/'+ROBOT_ID))
        x = vicon_checker()
    except KeyboardInterrupt:
        sys.exit()          
    rospy.spin()

