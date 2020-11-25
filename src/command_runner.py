#!/usr/bin/env python

import rospy
from std_msgs.msg import String
import subprocess

rospy.init_node('command_runner')

def run_command(msg):
    parts = msg.data.split(' ')

    subprocess.call(parts)

    # print(parts)

command_sub = rospy.Subscriber('cmds_to_run', String, run_command)

rospy.spin()