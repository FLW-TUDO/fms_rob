#!/usr/bin/env python

'''
Server for calculating parking positions around a workstation 
'''

import rospy
from geometry_msgs.msg import TransformStamped, Pose
from math import cos, sin
import tf_conversions
from fms_rob.srv import parkPose, parkPoseResponse


'''
#######################################################################################
'''

ROBOT_ID = 'rb1_base_b'

'''
#######################################################################################
'''

station_pose = TransformStamped()

def get_parking_spots(req):
    global station_pose
    goal_inbound = Pose()
    goal_outbound = Pose()
    goal_queue = Pose()
    station_id = req.station_id
    distance = req.distance
    topic = ['/vicon/'+station_id+'/'+station_id, 'geometry_msgs/TransformStamped']
    if(topic not in rospy.get_published_topics('/vicon/')):
        print('Cart Topic Not Found!')
        return
    rospy.Subscriber('/vicon/'+station_id+'/'+station_id, TransformStamped, get_vicon_pose)
    rospy.sleep(1)
    station_pose_quat = [station_pose.transform.rotation.x, station_pose.transform.rotation.y, station_pose.transform.rotation.z, station_pose.transform.rotation.w]
    station_pose_euler = tf_conversions.transformations.euler_from_quaternion(station_pose_quat)
    # offset from station for parking
    goal_outbound.position.x = station_pose.transform.translation.x + (distance * cos(station_pose_euler[2]))
    goal_outbound.position.y = station_pose.transform.translation.y - (distance * sin(station_pose_euler[2]))
    goal_inbound.position.x = station_pose.transform.translation.x - (distance * cos(station_pose_euler[2]))
    goal_inbound.position.y = station_pose.transform.translation.y + (distance * sin(station_pose_euler[2]))
    goal_queue.position.x = station_pose.transform.translation.x - (distance * cos(station_pose_euler[2]))
    goal_queue.position.y = station_pose.transform.translation.y - (distance * sin(station_pose_euler[2]))
    # orientation wrt station
    goal_inbound.orientation.x = goal_outbound.orientation.x = goal_queue.orientation.x = station_pose_quat[0]
    goal_inbound.orientation.y = goal_outbound.orientation.x = goal_queue.orientation.x = station_pose_quat[1]
    goal_inbound.orientation.z = goal_outbound.orientation.x = goal_queue.orientation.x = station_pose_quat[2]
    goal_inbound.orientation.w = goal_outbound.orientation.x = goal_queue.orientation.x = station_pose_quat[3]
    print('Parking Spots Calculated')
    return [goal_inbound, goal_outbound, goal_queue]

def get_vicon_pose(data):
    global station_pose
    station_pose = data

def dock_pose_server():
    s = rospy.Service('/'+ROBOT_ID+'/get_parking_spots', parkPose, get_parking_spots)
    print ('Parking Spots Calculation Ready')
 

if __name__ == "__main__":
    rospy.init_node('park_pose_server')
    dock_pose_server()
    rospy.spin()
