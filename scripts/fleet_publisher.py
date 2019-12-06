#!/usr/bin/env python

import rospy, math, tf
import sys
from costmap_converter.msg import ObstacleArrayMsg, ObstacleMsg
from geometry_msgs.msg import Point32, QuaternionStamped, Quaternion, TwistWithCovariance, TransformStamped
from tf.transformations import quaternion_from_euler

class FleetPub:
    def __init__(self):
        rospy.init_node('fleet_publisher')
        self.rob_2_pub = rospy.Publisher('/rb1_base_b/move_base/TebLocalPlannerROS/obstacles', ObstacleArrayMsg, queue_size=10)
        topic = ['/vicon/rb1_base_c/rb1_base_c', 'geometry_msgs/TransformStamped']
        if(topic not in rospy.get_published_topics('/vicon/')):
            rospy.logerr('Robot Topic Not Found!')
            return
        rospy.Subscriber('/vicon/rb1_base_c/rb1_base_c', TransformStamped, self.rob_1_vicon_update)
        self.vel_x = 1.0
        self.vel_y = 1.0
        rospy.sleep(1)
        self.publish_obstacle_msg()

    def publish_obstacle_msg(self):
        print("Testing_obstacle_msg")

        #obstacle_list = [ObstacleMsg()]
        obstacle_arr_msg = ObstacleArrayMsg() 
        obstacle_arr_msg.header.stamp = rospy.Time.now()
        obstacle_arr_msg.header.frame_id = "vicon_world" # CHANGE HERE: odom/map

        # Add point obstacle
        obstacle_msg = ObstacleMsg()
        obstacle_msg.header.stamp = rospy.Time.now()
        obstacle_msg.header.frame_id = "vicon_world" # CHANGE HERE: odom/map

        obstacle_msg.radius = 0.5
        obstacle_msg.id = 2
        obstacle_msg.polygon.points = [Point32()]
        obstacle_msg.polygon.points[0].x = self.rob_1_pose_trans[0]
        obstacle_msg.polygon.points[0].y = self.rob_1_pose_trans[1]
        obstacle_msg.polygon.points[0].z = 0

        obstacle_msg.orientation.x = self.rob_1_pose_rot[0]
        obstacle_msg.orientation.y = self.rob_1_pose_rot[1]
        obstacle_msg.orientation.z = self.rob_1_pose_rot[2]
        obstacle_msg.orientation.w = self.rob_1_pose_rot[3]


        obstacle_msg.velocities.twist.linear.x = self.vel_x
        obstacle_msg.velocities.twist.linear.y = self.vel_y
        obstacle_msg.velocities.twist.linear.z = 0
        obstacle_msg.velocities.twist.angular.x = 0
        obstacle_msg.velocities.twist.angular.y = 0
        obstacle_msg.velocities.twist.angular.z = 0

        obstacle_arr_msg.obstacles.append(obstacle_msg)
        #print(obstacle_arr_msg)

        r = rospy.Rate(10) # 10hz
        while not rospy.is_shutdown():
            self.rob_2_pub.publish(obstacle_arr_msg)
            r.sleep()

    def rob_1_vicon_update(self, data):
        self.rob_1_pose_trans = [data.transform.translation.x, data.transform.translation.y]
        self.rob_1_pose_rot = [data.transform.rotation.x, data.transform.rotation.y, data.transform.rotation.z, data.transform.rotation.w]

if __name__ == '__main__': 
    try:
        fbu = FleetPub()
    except KeyboardInterrupt:
        sys.exit()
        rospy.logerr('Interrupted!')
    rospy.spin()