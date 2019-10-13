#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Pose
from std_msgs.msg import String
import paho.mqtt.client as mqttClient
import time, sys, json
#from math import pow, atan2, sqrt, cos, sin, pi
#from robotnik_msgs.srv import drive, set_digital_output, set_odometry, place, follow
from robotnik_msgs.msg import MQTT_ack
from fms_rob.msg import RobActionSelect, RobActionStatus
import yaml
#import tf_conversions


'''
#######################################################################################
'''

ROBOT_ID = rospy.get_param('/ROBOT_ID')

'''
#######################################################################################
'''

# MQTT Settings
broker_address= "gopher.phynetlab.com"
port = 8883
Connected = False   #global variable for the state of the connection
#client = mqttClient.Client(ROBOT_ID+str(time.time()))

def on_connect(client, userdata, flags, rc):
    if rc == 0:
        print("Connected to broker")
        global Connected                #Use global variable
        Connected = True                #Signal connection
    else:
        print("Connection failed")

def on_message(client, userdata, message):
    	#print 'on_message'
	return

def exit_handler():
    client.disconnect()
    sys.exit()

client = mqttClient.Client()
client.on_connect= on_connect                          # attach function to callback
client.on_message= on_message                          # attach function to callback
client.connect(broker_address, port=port)              # connect to broker
client.loop_start()                                    # start the loop
client.subscribe("/robotnik/#", 0) # Topics with wild card and a robotnik namespace

'''
#######################################################################################
'''

class command_router:
    def __init__(self):
        rospy.init_node('command_router')
        client.message_callback_add("/robotnik/mqtt_ros_command", self.parse_data)
        self.klt_num_pub = rospy.Publisher('/'+ROBOT_ID+'/klt_num', String, queue_size=10)
        self.action_pub = rospy.Publisher('/'+ROBOT_ID+'/rob_action', RobActionSelect, queue_size=10)
        self.action_status_sub = rospy.Subscriber('/'+ROBOT_ID+'/rob_action_status', RobActionStatus, self.status_mapping_update)

    def parse_data(self, client, userdata, message):
        mqtt_msg = json.loads(message.payload)
        goal = Pose()
        #pose.header.frame_id = 'world'
        if (mqtt_msg['robot_id'] == ROBOT_ID):
            print ("Message received: "  + message.payload)
            action = mqtt_msg['action']
            cart_id = mqtt_msg['cart_id']
            command_id = str(mqtt_msg['command_id'])
            station_id = mqtt_msg['station_id']
            bound_mode = mqtt_msg['bound_mode']
            follow_id = mqtt_msg['follow_id']
            #cart_id = '/vicon/'+cart_no_id+'/'+cart_no_id
            # pose_translation
            goal.position.x = mqtt_msg['pose']['translation']['x']
            goal.position.y = mqtt_msg['pose']['translation']['y']
            goal.position.z = mqtt_msg['pose']['translation']['z']
            # pose.orientation
            goal.orientation.x = mqtt_msg['pose']['rotation']['x']
            goal.orientation.y = mqtt_msg['pose']['rotation']['y']
            goal.orientation.z = mqtt_msg['pose']['rotation']['z']
            goal.orientation.w = mqtt_msg['pose']['rotation']['w']
            self.select_action(action, goal, command_id, cart_id, station_id, bound_mode)
            #ack = ack_routine('mqtt')
        else:
            pass
        return

    def msg2json(self, msg):
        y = yaml.load(str(msg))
        return json.dumps(y,indent=4)

    def select_action(self, action, goal, command_id, cart_id, station_id, bound_mode):
        if (action== 'drive'):
            print('Drive Action Selected')
            msg = RobActionSelect()
            msg.action = 'drive'
            msg.goal = goal
            msg.command_id = command_id
            self.action_pub.publish(msg)
        if (action== 'dock'): 
            print('Dock Action Selected')
            msg = RobActionSelect()
            msg.action = 'dock'
            msg.goal = goal
            msg.command_id = command_id
            self.action_pub.publish(msg)
        if (action== 'undock'):
            print('Undock Action Selected')
            msg = RobActionSelect()
            msg.action = 'undock'
            msg.goal = goal
            msg.command_id = command_id
            self.action_pub.publish(msg)
        if (action== 'pick'):
            print('Pick Action Selected')
            msg = RobActionSelect()
            msg.action = 'pick'
            msg.goal = goal
            msg.command_id = command_id
            msg.cart_id = cart_id
            self.action_pub.publish(msg)
        if (action== 'place'):
            print('Place Action Selected')
            msg = RobActionSelect()
            msg.action = 'place'
            msg.goal = goal
            msg.command_id = command_id
            msg.station_id = station_id
            msg.bound_mode = bound_mode # inbound, outbound, queue
            self.action_pub.publish(msg)
        elif (action == 'cancelCurrent'):
            msg = RobActionSelect()
            msg.action = 'cancelCurrent'
            #msg.goal = goal
            msg.command_id = command_id
            self.action_pub.publish(msg)
        elif (action == 'cancelAll'):
            msg = RobActionSelect()
            msg.action = 'cancelAll'
            #msg.goal = goal
            msg.command_id = command_id
            self.action_pub.publish(msg)
        elif (action == 'cancelAtAndBefore'):
            msg = RobActionSelect()
            msg.action = 'cancelAtAndBefore'
            #msg.goal = goal
            msg.command_id = command_id
            self.action_pub.publish(msg)
        else:
            pass

    def status_mapping_update(self, data):
        msg = MQTT_ack()
        msg.robotid = ROBOT_ID
        msg.cartid = data.cart_id
        msg.command = data.action
        msg.commandid = data.command_id
        if (data.status == 3):
            msg.response = 'finished'
        elif (data.status == 1):
            msg.response = 'moving'
        else: msg.response = 'free'
        msg_json = self.msg2json(msg)
        print('sending status data via mqtt')
     	client.publish('/robotnik/mqtt_ros_info',msg_json)

'''
    def ack_routine(data_id):
        global ack_msg 
        #ack_msg = ''
        global robot_id, action, cart_id, cart_no_id, command_id, follow_id
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
'''
if __name__ == '__main__':
    try:
        cr = command_router()
    except KeyboardInterrupt:
        sys.exit()
        print('Interrupted!')
    rospy.spin()

