#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped, TransformStamped, Twist, PointStamped
from std_msgs.msg import String
import paho.mqtt.client as mqttClient
import time, sys, json
from math import pow, atan2, sqrt, cos, sin, pi
from robotnik_msgs.srv import drive, set_digital_output, set_odometry, place, follow
from robotnik_msgs.msg import MQTT_ack
from os import listdir
import time
from nav_msgs.msg import Odometry
from std_msgs.msg import Empty, String
import yaml
from subprocess import call
import tf_conversions
from rb1_base_msgs.srv import SetElevator, SetElevatorRequest, SetElevatorResponse
Connected = False   #global variable for the state of the connection

broker_address= "gopher.phynetlab.com"  #Broker address
port = 8883                         #Broker port
ROBOT_ID = 'rb1_base_b' #this should be taken from the ROS param server
global dock_status 
dock_status = 0
client = mqttClient.Client(ROBOT_ID+str(time.time()))     
global carried_cart
carried_cart= ''
robot_id = ''
client.connect(broker_address, port=port)          #connect to broker
global mqtt_inc
mqtt_inc = 0


def exit_handler():
    client.disconnect()
    sys.exit()

def msg2json(msg):
   ''' Convert a ROS message to JSON format'''
   y = yaml.load(str(msg))
   return json.dumps(y,indent=4)

def parse_data(mqtt_msg):
	global goal, robot_id, action, cart_id, cart_no_id, command_id, follow_id
	goal = TransformStamped()
	#pose.header.frame_id = 'world'
	robot_id = mqtt_msg['id']
	action = mqtt_msg['action']
	cart_no_id = mqtt_msg['cartid']
	command_id = mqtt_msg['commandid']
	follow_id = mqtt_msg['followid']
	cart_id = '/vicon/'+cart_no_id+'/'+cart_no_id

	# Pose

	# pose.pose
	goal.transform.translation.x = mqtt_msg['pose']['translation']['x']
	goal.transform.translation.y = mqtt_msg['pose']['translation']['y']
	goal.transform.translation.z = mqtt_msg['pose']['translation']['z']
	# pose.orientation
	goal.transform.rotation.x = mqtt_msg['pose']['rotation']['x']
	goal.transform.rotation.y = mqtt_msg['pose']['rotation']['y']
	goal.transform.rotation.z = mqtt_msg['pose']['rotation']['z']
	goal.transform.rotation.w = mqtt_msg['pose']['rotation']['w']
	return

def call_robot():
	global goal, robot_id, action, cart_id, dock_status, carried_cart, cart_no_id, follow_id, mqtt_inc
	mqtt_inc+=1
	klt_num_pub = rospy.Publisher('/'+ROBOT_ID+'/klt_num', String, queue_size=10)
	klt_num_msg = String()
	if(robot_id=='rb1_base_b'):
		if(action== 'drive'):
			rospy.wait_for_service('drivewo')
			drive_object = rospy.ServiceProxy('drivewo', drive)
			ack_routine('move')
			ros_response = drive_object(goal)
			print(ros_response.feedback)
			if(ros_response.feedback=='success'):
				ack_routine('finished')

		elif(action== 'driveWithOrientation'):
			rospy.wait_for_service('drive')
			drive_object = rospy.ServiceProxy('drive', drive)
			ack_routine('move')

			ros_response = drive_object(goal, action)
			print(ros_response.feedback)
			if(ros_response.feedback=='success'):
				ack_routine('finished')

		elif((action== 'dock')):
			 execfile('src/str_simple_run/scripts/robot_docking.py')
			 dock_status = 1
 			 ack_routine('finished')

		elif((action== 'unstickRobot') and dock_status==0):
			 execfile('src/str_simple_run/scripts/unstuck_robot_server.py')
 			 ack_routine('finished')

		elif((action== 'unstickCart') and dock_status==1):
			print(cart_id)
			rospy.wait_for_service('unstickcart')
			place_object = rospy.ServiceProxy('unstickcart', place)
			ack_routine('move')
			ros_response = place_object(goal, cart_id, action)
			print(ros_response.feedback)
			if(ros_response.feedback=='success'):
				ack_routine('finished')

		elif((action== 'pick') and dock_status==0):
			dock_status= 1
			rospy.wait_for_service('pick')
			drive_object = rospy.ServiceProxy('pick', place)
			ack_routine('move')
			ros_response = drive_object(goal, cart_id, action, str(mqtt_inc))
			print(ros_response.feedback, ros_response.command_id)
			if((ros_response.feedback == 'success') and (mqtt_inc == int(ros_response.command_id))):
				carried_cart = cart_no_id
				klt_num_msg.data = '/vicon/'+carried_cart+'/'+carried_cart+''
				rospy.sleep(0.05)
				klt_num_pub.publish(klt_num_msg)
				execfile('src/str_simple_run/scripts/robot_docking.py')
				ack_routine('finished')
		elif((action== 'are_you_free') and (dock_status==1)):
				ack_routine('busy')
		elif((action== 'are_you_free') and (dock_status==0)):
				ack_routine('free')	
		else:
			pass
	if(carried_cart == cart_no_id):
		if(action== 'place'):
			print(cart_id)
			rospy.wait_for_service('placewo')
			place_object = rospy.ServiceProxy('placewo', place)
			ack_routine('move')
			ros_response = place_object(goal, cart_id, action, str(mqtt_inc))
			print(ros_response.feedback)
			if(ros_response.feedback=='success'):
				ack_routine('finished')

		elif(action== 'placeWithOrientation'):
			rospy.wait_for_service('place')
			place_o_object = rospy.ServiceProxy('place', place)
			ack_routine('move')
			ros_response = place_o_object(goal, cart_id, action, str(mqtt_inc))
			print(ros_response.feedback)
			if(ros_response.feedback=='success'):
				ack_routine('finished')

		elif((action== 'undockF') and dock_status==1):
			execfile('src/str_simple_run/scripts/robot_undocking_forward.py')
			print('returns')
			#ros_response.feedback= 'success'
			dock_status = 0
			carried_cart = ''
			ack_routine('finished')
			klt_num_msg.data = ''
			rospy.sleep(0.05)
			klt_num_pub.publish(klt_num_msg)
		elif((action== 'undockB') and dock_status==1):
			execfile('src/str_simple_run/scripts/robot_undocking_backward.py')
			#ros_response.feedback= 'success'
			dock_status = 0	
			carried_cart = ''	
			ack_routine('finished')
			klt_num_msg.data = ''
			rospy.sleep(0.05)
			klt_num_pub.publish(klt_num_msg)
		else:
			pass		

	if((action== 'follow') or (action == 'unfollow') or (action == 'unfollowWithTurnaround')):
		if((robot_id == '') and (dock_status ==1) and carried_cart == cart_no_id):
			rospy.wait_for_service('follow_cart')
			try:			
				drive_object = rospy.ServiceProxy('follow_cart', follow)
				ack_routine('move')
				ros_response = drive_object(cart_id, follow_id, action)
				print(ros_response.feedback)
				if(ros_response.feedback=='success'):
					ack_routine('finished')
			except rospy.ServiceException, e:
				ack_routine('finished')
    			return
		elif((robot_id=='rb1_base_b')):
			rospy.wait_for_service('follow_robot')
			try:			
				drive_object = rospy.ServiceProxy('follow_robot', follow)
				ack_routine('move')
				ros_response = drive_object(cart_id, follow_id, action)
				print(ros_response.feedback)
				if(ros_response.feedback=='success'):
					ack_routine('finished')
			except rospy.ServiceException, e:
				ack_routine('finished')
    			return
		else:
			pass


def on_connect(client, userdata, flags, rc):
    if rc == 0:
        print("Connected to broker")
        global Connected                #Use global variable
        Connected = True                #Signal connection
    else:
        print("Connection failed")

def ack_routine(data_id):
	global ack_msg 
	#ack_msg = ''
	global goal, robot_id, action, cart_id, cart_no_id, command_id, follow_id
	if(data_id == 'mqtt'):
		ack_msg.response = 'ack'
	elif(data_id == 'move'):
		ack_msg.response = 'moving'
	elif(data_id == 'finished'):
		ack_msg.response = 'finished'
	elif(data_id == 'free'):
		ack_msg.response = 'free'
	elif(data_id == 'busy'):
		ack_msg.response = 'busy'
	if(cart_no_id != ''):
		ack_msg.cartid = cart_no_id
	if(robot_id != ''):
		ack_msg.robotid = robot_id
	ack_msg.command = action
	ack_msg.commandid = command_id
	ack_data = msg2json(ack_msg)
	client.publish('/robotnik/mqtt_ros_info',ack_data)	
	return

def on_message_info(client, userdata, message):
	global message_flag
	message_flag = message_flag + 1 
	mqtt_msg = json.loads(message.payload)
	if(mqtt_msg['id']==''+ROBOT_ID+''):
		print "Message received: "  + message.payload               #Use global variable
		reformilate_mqtt = parse_data(mqtt_msg)
		print message_flag
		call_robot()
		ack = ack_routine('mqtt')
		print('on message over')
	else:
		pass
	return

def on_message(client, userdata, message):
	#print 'on_message'
	return

if __name__ == '__main__':
    #pub = rospy.Publisher('/'+ROBOT_ID+'/move_base_simple/goal', PoseStamped, queue_size=10)
	global ack_msg, message_flag
	message_flag= 0
	ack_msg = MQTT_ack()
	rospy.init_node('mqtt_listener', anonymous=True)
	rate = rospy.Rate(30) # 10hz 
	client.on_connect= on_connect                      #attach function to callback
	client.on_message= on_message
	# Add message callbacks that will only trigger on a specific subscription match.
	client.message_callback_add("/robotnik/mqtt_ros_command", on_message_info)

	print('wait over')            #attach function to callback
	client.loop_start()        #start the loop
	while Connected != True:    #Wait for connection
		rate.sleep()
	client.subscribe("/robotnik/#", 0)
	if(message_flag== 1):
		
		message_flag = 0

	try:
		while True:
			time.sleep(1)
	except KeyboardInterrupt:
		sys.exit()
	rospy.spin()

