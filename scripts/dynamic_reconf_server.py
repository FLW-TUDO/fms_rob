#!/usr/bin/env python

import rospy

from dynamic_reconfigure.server import Server
from fms_rob.cfg import dynamic_paramsConfig

def callback(config, level):
    rospy.loginfo("""Reconfigure Request: {cart_id}""".format(**config))
    return config

if __name__ == "__main__":
    rospy.init_node("dynamic_reconf_server", anonymous = False)
    srv = Server(dynamic_paramsConfig, callback)
    rospy.spin()