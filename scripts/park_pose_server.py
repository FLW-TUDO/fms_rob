#!/usr/bin/env python

'''
Server for calculating parking positions around a workstation 
'''

import rospy
from geometry_msgs.msg import TransformStamped, PoseStamped
from math import cos, sin
import tf_conversions
from fms_rob.srv import parkPose, parkPoseResponse
import tf


'''
#######################################################################################
'''

ROBOT_ID = rospy.get_param('/ROBOT_ID', 'rb1_base_b')

'''
#######################################################################################
'''

station_pose = TransformStamped()

def get_parking_spots(req):
    global station_pose
    goal_inbound = PoseStamped()
    goal_outbound = PoseStamped()
    goal_queue = PoseStamped()
    outbound_to_station_listener = tf.TransformListener()
    inbound_to_station_listener = tf.TransformListener()
    queue_to_station_listener = tf.TransformListener()
    station_id = req.station_id
    distance = req.distance
    topic = ['/vicon/'+station_id+'/'+station_id, 'geometry_msgs/TransformStamped']
    if(topic not in rospy.get_published_topics('/vicon/')):
        rospy.logerr('Station Topic Not Found!')
        return
    rospy.Subscriber('/vicon/'+station_id+'/'+station_id, TransformStamped, get_vicon_pose)
    rospy.sleep(1)    
    station_pose_quat = [station_pose.transform.rotation.x, station_pose.transform.rotation.y, station_pose.transform.rotation.z, station_pose.transform.rotation.w]
    #station_pose_euler = tf_conversions.transformations.euler_from_quaternion(station_pose_quat)
   
    # offset from station for parking
    goal_outbound.header.frame_id = 'vicon/'+station_id+'/'+station_id
    goal_outbound.pose.position.y -= distance
    goal_outbound_transformed = outbound_to_station_listener.transformPose('vicon_world', goal_outbound) # goal point in vicon_world frame

    goal_inbound.header.frame_id = 'vicon/'+station_id+'/'+station_id
    goal_inbound.pose.position.y += distance
    goal_inbound_transformed = inbound_to_station_listener.transformPose('vicon_world', goal_inbound)

    goal_queue.header.frame_id = 'vicon/'+station_id+'/'+station_id
    goal_queue.pose.position.x -= distance
    goal_queue.pose.position.y += distance
    goal_queue_in_vicon_world = queue_to_station_listener.transformPose('vicon_world', goal_queue)
    '''
    goal_outbound.position.x = station_pose.transform.translation.x + (distance * cos(station_pose_euler[2]))
    goal_outbound.position.y = station_pose.transform.translation.y - (distance * sin(station_pose_euler[2]))
    goal_inbound.position.x = station_pose.transform.translation.x - (distance * cos(station_pose_euler[2])) # - cos
    goal_inbound.position.y = station_pose.transform.translation.y + (distance * sin(station_pose_euler[2])) # + sin
    goal_queue.position.x = station_pose.transform.translation.x - (distance * cos(station_pose_euler[2]))
    goal_queue.position.y = station_pose.transform.translation.y - (distance * sin(station_pose_euler[2]))
    '''
    # orientation wrt station
    goal_inbound.pose.orientation.x = goal_outbound.pose.orientation.x = goal_queue.pose.orientation.x = station_pose_quat[0]
    goal_inbound.pose.orientation.y = goal_outbound.pose.orientation.x = goal_queue.pose.orientation.x = station_pose_quat[1]
    goal_inbound.pose.orientation.z = goal_outbound.pose.orientation.x = goal_queue.pose.orientation.x = station_pose_quat[2]
    goal_inbound.pose.orientation.w = goal_outbound.pose.orientation.x = goal_queue.pose.orientation.x = station_pose_quat[3]
    rospy.loginfo('Parking Spots Calculated')
    return [goal_outbound_transformed, goal_inbound_transformed, goal_queue_in_vicon_world]

def get_vicon_pose(data):
    global station_pose
    station_pose = data

def dock_pose_server():
    s = rospy.Service('/'+ROBOT_ID+'/get_parking_spots', parkPose, get_parking_spots)
    rospy.loginfo('Parking Spots Calculation Ready')

def shutdown_hook():
    rospy.logwarn('Park Pose Server node shutdown by user')

if __name__ == "__main__":
    rospy.init_node('park_pose_server')
    rospy.on_shutdown(shutdown_hook)
    dock_pose_server()
    rospy.spin()
