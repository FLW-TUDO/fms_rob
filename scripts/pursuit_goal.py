#!/usr/bin/env python

'''
 Script providing pursuit of received goals via mqtt and smoothes the transitions 
 between them using a PD controller
'''

import rospy
from geometry_msgs.msg import PoseStamped, TransformStamped, Twist
from std_msgs.msg import String
import paho.mqtt.client as mqttClient
import time, sys, json, yaml
from math import pow, atan2, sqrt, cos, sin, pi
from robotnik_msgs.msg import path_request
import tf_conversions
from std_msgs.msg import Bool, Float32
from tf.transformations import *

#user = "yourUser"                    #Connection username
#password = "yourPassword"            #Connection password

'''
###########################################################################
'''


ROBOT_ID = 'rb1_base_b' #this should be taken from the ROS param server


'''
###########################################################################
'''


# MQTT settings
broker_address= "gopher.phynetlab.com"  #Broker address
port = 8883 

def on_connect(client, userdata, flags, rc):
    if rc == 0:
        print("Connected to broker")
        global Connected                #Use global variable
        Connected = True                #Signal connection
    else:
        print("Connection failed")

def on_message(client, userdata, message):
    print ('Message does Not match any Topic Filter!')
    return

client = mqttClient.Client()
client.on_connect= on_connect                          # attach function to callback
client.on_message= on_message                          # attach function to callback
client.connect(broker_address, port=port)              # connect to broker
client.loop_start()                                    # start the loop
client.subscribe("/robotnik/#", 0) # Topics with wild card and a robotnik namespace

'''
###########################################################################
'''


class goal_follower:
    def __init__(self):
        rospy.init_node('pursuit_goal')
        self.velocity_pub = rospy.Publisher('/'+ROBOT_ID+'/move_base/cmd_vel', Twist, queue_size=10)
        client.message_callback_add('/robotnik/'+ROBOT_ID+'/pursuit_goal', self.mqtt_parser)
        self.pose_subscriber = rospy.Subscriber('/vicon/'+ROBOT_ID+'/'+ROBOT_ID, TransformStamped, self.update_pose)
        self.start_subscriber = rospy.Subscriber('/'+ROBOT_ID+'/goal_path/start', Bool, self.drive) #to attach callback to driving methd
        self.start_pub = rospy.Publisher('/'+ROBOT_ID+'/goal_path/start', Bool, queue_size=10)
        self.theta_pub = rospy.Publisher('/'+ROBOT_ID+'/goal_path/rob_theta', Float32, queue_size=10) # for error visualization
        self.error_theta= 1.0
        self.kp_ang = 0.7 #0.7
        self.kd_ang = 0.1 #0.1
        self.distance_tolerance = 0.1
        current_time = None
        self.sample_time = 0.0001
        self.current_time = current_time if current_time is not None else time.time()
        self.last_time = self.current_time
        self.p_term_ang = 0.0
        self.d_term_ang = 0.0
        self.last_error_theta = 0.0
        self.output = 0.0
        #self.kp_lin = 0.8
        self.x_vel = 0.8
        self.start_msg = Bool()
        self.theta_msg = Float32()

    def mqtt_parser(self, client, userdata, message):
        mqtt_msg = json.loads(message.payload)
        if(mqtt_msg['id']==''+ROBOT_ID+''):
            #print ('Message received: '  + message.payload)
            self.goal_x = mqtt_msg['goal']['translation']['x']
            self.goal_y = mqtt_msg['goal']['translation']['y']
            self.start_pub.publish(True) # direct call to callback function buffers received mqtt messages   
        else:
            pass
        return

    def update_pose(self, data):
        """Callback function which is called when a new message of type Pose is received by the subscriber."""
        self.pose = data
        self.pose.transform.translation.x = round(self.pose.transform.translation.x, 10)
        self.pose.transform.translation.y = round(self.pose.transform.translation.y, 10)

    def euclidean_distance(self):
        """Euclidean distance between current pose and the next goal point."""
        return sqrt(pow((self.goal_x - self.pose.transform.translation.x), 2) +
                pow((self.goal_y - self.pose.transform.translation.y), 2))

    def steering_angle(self):
        """Angle between current orientation and the heading of the next goal point"""
        return atan2(self.goal_y - self.pose.transform.translation.y, self.goal_x - self.pose.transform.translation.x)
    '''
    def linear_vel(self):
        return self.kp_lin * self.euclidean_distance()
    '''

    def angular_vel(self):
        current_time = None
        rot=[self.pose.transform.rotation.x, self.pose.transform.rotation.y, self.pose.transform.rotation.z, self.pose.transform.rotation.w]
        rot_euler = tf_conversions.transformations.euler_from_quaternion(rot)
        theta = rot_euler[2]
        self.error_theta= self.steering_angle() - theta
        self.error_theta= atan2(sin(self.error_theta),cos(self.error_theta))
        self.theta_msg = self.error_theta
        self.theta_pub.publish(self.theta_msg)
        self.current_time = current_time if current_time is not None else time.time()
        delta_time = self.current_time - self.last_time
        delta_error = self.error_theta - self.last_error_theta
        if (delta_time > self.sample_time):
            self.p_term_ang = self.kp_ang * self.error_theta
            self.d_term_ang = 0.0
            if delta_time > 0:
                self.dTerm = delta_error / delta_time
            self.last_time = self.current_time
            self.last_error_theta = self.error_theta
            self.output = self.p_term_ang + (self.kd_ang * self.d_term_ang)
        return self.output

    def drive(self, data):
        if(data.data == True):
            rospy.sleep(0.2)
            vel_msg = Twist()
            print('Navigating to Goal')
            while(self.euclidean_distance() >= self.distance_tolerance):
                vel_msg.linear.x = self.x_vel
                vel_msg.linear.y = 0
                vel_msg.linear.z = 0
                # Angular velocity in the z-axis.
                vel_msg.angular.x = 0
                vel_msg.angular.y = 0
                vel_msg.angular.z = self.angular_vel()
                # Publishing vel_msg
                self.velocity_pub.publish(vel_msg)
            vel_msg.linear.x = 0
            vel_msg.angular.z = 0
            self.velocity_pub.publish(vel_msg)
            print('Goal Reached!')
            #rospy.sleep(0.7)
            #self.start_subscriber.unregister()
        else:
            pass
        return

    def exit_handler(self):
        client.disconnect()
        sys.exit()     
    
if __name__ == '__main__':
    
    try:
        path_control = goal_follower()
    except KeyboardInterrupt:
        sys.exit()
        
    rospy.spin()
