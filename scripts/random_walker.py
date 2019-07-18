#!/usr/bin/env python
import rospy
import random
import time, sys
from geometry_msgs.msg import PoseStamped
from actionlib_msgs.msg import GoalStatusArray
#from move_base_msgs.msg import MoveBaseActionGoal, MoveBaseActionFeedback, MoveBaseActionResult


ROBOT_ID = 'rb1_base_b'


class random_walker:
    def __init__(self):
        rospy.init_node('random_walker')
        self.goal_pub = rospy.Publisher('/'+ROBOT_ID+'/move_base_simple/goal', PoseStamped, queue_size = 10)
        self.goal_status_sub = rospy.Subscriber('/'+ROBOT_ID+'/move_base/status', GoalStatusArray, self.simple_goal_status)
        #self.result_sub = rospy.Subscriber('/rb1_base_b/move_base/result', MoveBaseActionResult, )
        #self.feedback_sub = rospy.Subscriber('/rb1_base_b/move_base/feedback', MoveBaseAcrionFeedback, )
        self.x_bound_start = 0
        self.x_bound_stop = 3
        self.y_bound_start = -2
        self.y_bound_stop = 0
        self.x_step = 1
        self.y_step = 1
        self.obs_x_up = 2.5
        self.obs_x_down = 1.5
        self.obs_y_up = -0.5
        self.obs_y_down = -1.0
        #self.drive()

    def gen_goal(self): # prevent consequtive generation of same numbers
        print('Generating new goal')
        goal_x = random.randrange(self.x_bound_start, self.x_bound_stop, self.x_step) 
        goal_y = random.randrange(self.y_bound_start, self.y_bound_stop, self.y_step) 
        #goal_x = random.randrange(1,5,1) 
        #goal_y = random.randrange(1,5,1)
        return (goal_x, goal_y)

    def feasible(self, x, y):
        if (((x < self.obs_x_up) and (x > self.obs_x_down)) or \
            ((y < self.obs_y_up) and (y > self.obs_y_down))):
            print('Infeasible goal ignored')
            return False
        else:
            print('Goal is feasible')
            return True
    
    def simple_goal_status(self, data):
        self.goal_status = data

    def drive(self, x, y):
        #self.goal_status = GoalStatusArray()
        goal_msg = PoseStamped()
        goal_msg.header.frame_id = 'vicon_world'
        goal_msg.pose.position.x = x
        goal_msg.pose.position.y = y
        goal_msg.pose.orientation.z = 0
        goal_msg.pose.orientation.w = 1
        rospy.sleep(0.5)
        print(goal_msg)
        self.goal_pub.publish(goal_msg)
        #print(self.goal_status.status_list)

        # self.goal_status = GoalStatusArray()
        # self.goal_status.status_list = 3
        # while True:
        #     if (self.goal_status.status_list == 3):
        #         goal = self.gen_goal()
        #         print(goal)
        #         if(self.feasible(goal[0], goal[1])):
        #         goal_msg = PoseStamped()
        #         goal_msg.header.frame_id = 'vicon_world'
        #         goal_msg.pose.position.x = goal[0]
        #         goal_msg.pose.position.y = goal[1]
        #         goal_msg.pose.orientation.z = 0
        #         goal_msg.pose.orientation.w = 1
        #         rospy.sleep(0.5)
        #         print(goal_msg)
        #         self.goal_pub.publish(goal_msg)
        #         print(self.goal_status.status_list)
        #     else:
        #         print(self.goal_status.status_list)
        #         pass
            #while(self.goal_status == '1')
            #goal_id_num = int(goal_id) + 1 
            #goal_id = str(goal_id_num)
            #rospy.sleep(4)
        #else:
         #   continue


if __name__ == '__main__':
    try:
        rm_client = random_walker()
        #while True:
        goal = rm_client.gen_goal()
        goal_x = goal[0]
        goal_y = goal[1]
        if(rm_client.feasible(goal_x, goal_y)):
            rm_client.drive(goal_x, goal_y)
            #rospy.sleep(2)
            # print(rm_client.goal_status.status_list)
            # while(rm_client.goal_status.status_list == 1):
            #     print('Goal is pursued')
            #     rospy.sleep(0.2)
            # print('Goal reached')
        #else:
         #   continue

    except KeyboardInterrupt:
        sys.exit()
    rospy.spin()
        
