#!/usr/bin/env python
#!/usr/bin/env python
import rospy
from geometry_msgs.msg import TransformStamped, PoseStamped
import tf_conversions
import paho.mqtt.client as mqttClient
import yaml, json
from robotnik_msgs.msg import path_request
import tf, sys
import time
from std_msgs.msg import String
from collections import deque, OrderedDict

ROBOT_ID = 'rb1_base_b'

'''
#######################################################################################
'''

# MQTT settings
broker_address= "gopher.phynetlab.com"  #Broker address
port = 8883 

def on_connect(client, userdata, flags, rc):
	if rc == 0:
		print("Connected to broker")
		global Connected				#Use global variable
		Connected = True				#Signal connection
	else:
		print("Connection failed")

def on_message(client, userdata, message):
	return

client = mqttClient.Client()
client.on_connect= on_connect						  # attach function to callback
client.on_message= on_message						  # attach function to callback
client.connect(broker_address, port=port)			  # connect to broker
client.loop_start()									# start the loop
client.subscribe("/robotnik/#", 0)

'''
#######################################################################################
'''

class TurtleBot:
		
    def __init__(self):
        rospy.init_node('wp_gen')
        self.wp_msg = PoseStamped()
        self.vicon_pose = TransformStamped()
        self.wp_queue = deque()
        self.waypoints = OrderedDict ([
                        ('wp1',[1, 1, 0, 0, 0 ,0, 1]),\
                        ('wp2',[2, 1, 0, 0, 0, 0, 1]),\
                        ('wp3',[3, 1, 0, 0, 0, 0, 1]),\
                        ('wp4',[3, 0, 0, 0, 0, 0, 1]),\
                        ('wp5',[3, -1, 0, 0, 0, 0, 1]),\
                        ('wp6',[4, 0, 0, 0, 0, 0, 1])#,\
                        #('wp7',[-3.5, -3.5, 0, 0, 0, 0, 1]),\
                        #('wp8',[4, -3, 0, 0, 0, 0, 1]),\
                        #('wp9',[6, 0, 0, 0, 0, 0, 1])
                        ])
        self.queuing()
        self.send_wp()

    def queuing(self):
        for wp in self.waypoints:
            self.wp_queue.append(wp)
            #print(self.wp_queue)
            
    def update_pose(self,data):
        self.vicon_pose = data

    def msg2json(self, msg):
        y = yaml.load(str(msg)) # unordered loading!
        return json.dumps(y, indent=4)
		
    def send_wp(self):
        r = rospy.Rate(1)
        rospy.sleep(2)
        wp_msg = path_request()
        while(len(self.wp_queue) != 0):
            wp_msg.id = ROBOT_ID
            pop_elem = self.wp_queue.popleft()
            wp_msg.goal.translation.x = self.waypoints[pop_elem][0]
            wp_msg.goal.translation.y = self.waypoints[pop_elem][1]
            wp_data = self.msg2json(wp_msg)
            client.publish('/robotnik/'+ROBOT_ID+'/pursuit_goal',wp_data)
            print(wp_data)
            r.sleep()

if __name__ == '__main__':
    try:
        x = TurtleBot()
    except KeyboardInterrupt:
        sys.exit()