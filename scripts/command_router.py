#!/usr/bin/env python

""" A node that distributes MQTT messages sent by the user to respective clients. """

import rospy
from geometry_msgs.msg import Pose
from std_msgs.msg import String
import paho.mqtt.client as mqttClient
import time, sys, json
#from robotnik_msgs.msg import MQTT_ack
from fms_rob.msg import RobActionSelect, RobActionStatus, MqttAck
import yaml


'''
#######################################################################################
'''

ROBOT_ID = rospy.get_param('/ROBOT_ID') # by default the robot id is set in the package's launch file

'''
#######################################################################################
'''

"""
MQTT Settings
"""
broker_address= "gopher.phynetlab.com"
port = 8883
Connected = False  

def on_connect(client, userdata, flags, rc):
    if rc == 0:
        rospy.loginfo('[ {} ]: Connected to Broker'.format(rospy.get_name()))
        global Connected                
        Connected = True                
    else:
        rospy.logerr('[ {} ]: Connection to Broker Failed!'.format(rospy.get_name()))

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

class CommandRouter:
    def __init__(self):
        rospy.init_node('command_router')
        client.message_callback_add("/robotnik/mqtt_ros_command", self.parse_data) # commands received from user ex: pick, place, etc
        self.klt_num_pub = rospy.Publisher('/'+ROBOT_ID+'/klt_num', String, queue_size=10) # used for interfacing with the ros_mocap package
        self.action_pub = rospy.Publisher('/'+ROBOT_ID+'/rob_action', RobActionSelect, queue_size=10) # topic to which the parsed action form the user is published
        rospy.Subscriber('/'+ROBOT_ID+'/rob_action_status', RobActionStatus, self.status_mapping_update) # subscribes to downstream status messages
        rospy.on_shutdown(self.shutdown_hook) # used to reset the interface with the ros_mocap package
        rospy.sleep(1)
        rospy.loginfo('[ {} ]: Ready'.format(rospy.get_name()))

    def parse_data(self, client, userdata, message):
        """ Parses data sent by user via MQTT. """
        mqtt_msg = json.loads(message.payload)
        goal = Pose()
        if (mqtt_msg['robot_id'] == ROBOT_ID):
            #print ("Message received: "  + message.payload)
            action = mqtt_msg['action']
            cart_id = mqtt_msg['cart_id'] # cart to be picked
            command_id = str(mqtt_msg['command_id']) # string for syncing commands - not used 
            station_id = mqtt_msg['station_id'] # station to place the cart at
            bound_mode = mqtt_msg['bound_mode'] # position relative to station
            cancellation_stamp = mqtt_msg['cancellation_stamp']
            # pose position
            goal.position.x = mqtt_msg['pose']['position']['x']
            goal.position.y = mqtt_msg['pose']['position']['y']
            goal.position.z = mqtt_msg['pose']['position']['z']
            # pose orientation
            goal.orientation.x = mqtt_msg['pose']['orientation']['x']
            goal.orientation.y = mqtt_msg['pose']['orientation']['y']
            goal.orientation.z = mqtt_msg['pose']['orientation']['z']
            goal.orientation.w = mqtt_msg['pose']['orientation']['w']
            rospy.loginfo('[ {} ]: MQTT Message Received >>> \n\t Action: {}, \n\t Cart ID: {}, \n\t Station ID: {}, \n\t Bound Mode: {}, \n\t Command ID: {}, \
                \n\t Cancellation timestamp: {}'.format(rospy.get_name(), action, cart_id, station_id, bound_mode, command_id, command_id)) # Goal Pose Not printed for convenience!
            self.control_flag = False
            self.select_action(action, goal, command_id, cart_id, station_id, bound_mode, cancellation_stamp)
        else:
            pass
        return

    def msg2json(self, msg):
        """ Converts ROS messages into json format. """
        y = yaml.load(str(msg))
        return json.dumps(y,indent=4)

    def select_action(self, action, goal, command_id, cart_id, station_id, bound_mode, cancellation_stamp):
        """ Reroutes parsed actions sent from user to the interested (corresponding) clients. """
        if (action == 'drive'):
            rospy.loginfo('[ {} ]: Drive Action Selected'.format(rospy.get_name()))
            msg = RobActionSelect()
            msg.action = 'drive'
            msg.goal = goal
            msg.command_id = command_id
            self.control_flag = True
            self.action_pub.publish(msg)
        if (action == 'dock'): 
            rospy.loginfo('[ {} ]: Dock Action Selected'.format(rospy.get_name()))
            msg = RobActionSelect()
            msg.action = 'dock'
            msg.goal = goal
            msg.command_id = command_id
            self.control_flag = True
            self.action_pub.publish(msg)
        if (action == 'undock'):
            rospy.loginfo('[ {} ]: Undock Action Selected'.format(rospy.get_name()))
            msg = RobActionSelect()
            msg.action = 'undock'
            msg.goal = goal
            msg.command_id = command_id
            self.control_flag = True
            self.action_pub.publish(msg)
        if (action == 'pick'):
            rospy.loginfo('[ {} ]: Pick Action Selected'.format(rospy.get_name()))
            msg = RobActionSelect()
            msg.action = 'pick'
            msg.goal = goal
            msg.command_id = command_id
            msg.cart_id = cart_id
            self.control_flag = True
            self.action_pub.publish(msg)
        if (action == 'place'):
            rospy.loginfo('[ {} ]: Place Action Selected'.format(rospy.get_name()))
            msg = RobActionSelect()
            msg.action = 'place'
            msg.goal = goal
            msg.command_id = command_id
            msg.station_id = station_id
            msg.bound_mode = bound_mode # inbound, outbound, queue
            self.control_flag = True
            self.action_pub.publish(msg)
        if (action == 'home'):
            rospy.loginfo('[ {} ]: Home Action Selected'.format(rospy.get_name()))
            msg = RobActionSelect()
            msg.action = 'home'
            msg.goal = goal
            msg.command_id = command_id
            self.control_flag = True
            self.action_pub.publish(msg)
        if (action == 'return'):
            rospy.loginfo('[ {} ]: Return Action Selected'.format(rospy.get_name()))
            msg = RobActionSelect()
            msg.action = 'return'
            msg.goal = goal
            msg.command_id = command_id
            self.control_flag = True
            self.action_pub.publish(msg)
        if (action == 'cancelCurrent'): # cancel current active goal
            rospy.loginfo('[ {} ]: cancelCurrent Action Selected'.format(rospy.get_name()))
            msg = RobActionSelect()
            msg.action = 'cancelCurrent'
            msg.command_id = command_id
            self.control_flag = True
            self.action_pub.publish(msg)
        if (action == 'cancelAll'): # cancel all goals
            rospy.loginfo('[ {} ]: cancelAll Action Selected'.format(rospy.get_name()))
            msg = RobActionSelect()
            msg.action = 'cancelAll'
            msg.command_id = command_id
            self.control_flag = True
            self.action_pub.publish(msg)
        if (action == 'cancelAtAndBefore'): # cancel goals at and before a certain timestamp
            rospy.loginfo('[ {} ]: cancelAtAndBefore Action Selected'.format(rospy.get_name()))
            msg = RobActionSelect()
            msg.action = 'cancelAtAndBefore'
            msg.command_id = command_id
            msg.cancellation_stamp = cancellation_stamp
            self.control_flag = True
            self.action_pub.publish(msg)
        elif (not self.control_flag):
            rospy.logerr('[ {} ]: Action Not Recognized!'.format(rospy.get_name()))
            return

    def status_mapping_update(self, data):
        """ Publishes status messages back to user via MQTT. """
        msg = MqttAck() # custom msg type that acts as container for ros messages pre-sending back to user
        msg.robot_id = ROBOT_ID
        msg.cart_id = data.cart_id
        msg.station_id = data.station_id
        msg.bound_mode = data.bound_mode
        msg.action = data.action
        msg.command_id = data.command_id
        msg.status = data.status
        '''
        if (data.status == 3):
            msg.response = 'finished'
        elif (data.status == 1):
            msg.response = 'moving'
        else: msg.response = 'free'
        '''
        msg_json = self.msg2json(msg)
        #rospy.loginfo_throttle(1, '{}: Sending status data via mqtt'.format(rospy.get_name()))
     	client.publish('/robotnik/mqtt_ros_info',msg_json)

    def shutdown_hook(self):
        """ Shutdown callback function. """
        self.klt_num_pub.publish('') # resets the picked up cart number in the ros_mocap package
        rospy.logwarn('[ {} ]: node shutdown by user'.format(rospy.get_name()))


if __name__ == '__main__':
    try:
        cr = CommandRouter()
    except KeyboardInterrupt:
        sys.exit()
        #print('Interrupted!')
    rospy.spin()

