#!/usr/bin/env python
from __future__ import division
from optparse import OptionParser
import roslib
import rospy
import rosparam
import numpy as np
from std_msgs.msg import Float32, Header, String
from alicat_flow.msg import bb9
import time

import alicat
from alicat_flow.msg import bb9

class BB9_AlicatFlowSetter:
    
    def __init__(self, publish_name='/alicat_bb9'):

        rospy.init_node('alicat_BB9_control')

        self.flowrate = 1.1  # L/min
        self.flowrate_pull_fixed = 1.0 # L/min
        assert self.flowrate >= self.flowrate_pull_fixed, 'flowrate must be > fixed pull value'

        self.push_addr_list = ['F']
        self.push_scale = 1.0
        self.flowrate_push = self.flowrate

        self.pull_addr_list = ['A','B','C','D','E']
        self.pull_scale = 1000.0
        self.flowrate_pull = max(self.flowrate - self.flowrate_pull_fixed,0)

        self.flowrate_pub = rospy.Publisher(publish_name, bb9, queue_size=10)
        self.pub_rate = rospy.Rate(1/3.0)
        
    def run(self):

        while not rospy.is_shutdown():

            addr_to_flowrate = {}
            for addr in self.push_addr_list:
                addr_to_flowrate[addr] = self.push_scale*self.flowrate_push/len(self.push_addr_list)

            for addr in self.pull_addr_list:
                addr_to_flowrate[addr] = self.pull_scale*self.flowrate_pull/len(self.pull_addr_list)

            msg = bb9()
            msg.header.stamp = rospy.Time.now()
            msg.address =  addr_to_flowrate.keys()
            msg.flowrate = [addr_to_flowrate[k] for k in msg.address]
            self.flowrate_pub.publish(msg)
            self.pub_rate.sleep()

        
if __name__ == '__main__':

    
    alicat_flow_controller = BB9_AlicatFlowSetter()
    alicat_flow_controller.run()    
