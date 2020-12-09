#!/usr/bin/env python

import rospy
from std_msgs.msg import String
import subprocess

rospy.init_node('command_runner')

process_lst = []

def run_command(msg):
    global process_lst

    if msg.data != '^C':
        parts = msg.data.split(' ')
        p = subprocess.Popen(parts, stdout=subprocess.PIPE)
        process_lst.append(p)

    else:
        for p in process_lst:
            p.terminate()

        process_lst = []

command_sub = rospy.Subscriber('cmds_to_run', String, run_command)

rospy.spin()