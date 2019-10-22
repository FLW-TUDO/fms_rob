#!/usr/bin/env python
"""
Server for dynamically altering parameters that are shared between 
nodes at runtime 
"""

import rospy

from dynamic_reconfigure.server import Server
from fms_rob.cfg import dynamic_paramsConfig

def callback(config, level): # callback called when a parameter is updated
    rospy.loginfo('''Reconfigure Request: {cart_id}'''.format(**config))
    return config

def shutdown_hook():
    rospy.logwarn('Dynamic Reconf Server node shutdown by user')

if __name__ == "__main__":
    rospy.init_node('dynamic_reconf_server', anonymous = False)
    rospy.on_shutdown(shutdown_hook)
    srv = Server(dynamic_paramsConfig, callback)
    rospy.spin()