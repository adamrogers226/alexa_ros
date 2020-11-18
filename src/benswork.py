import rospy
import time
import random
import numpy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion, quaternion_from_euler

#Initialize as a node 
rospy.init_node("wanderer")

#initialize movement velocities
stop_twist = Twist()
forward_twist = Twist()
rotate_twist = Twist()
reverse_rotate_twist = Twist()
scan_forward = Twist()
forward_twist.linear.x = 0.2
rotate_twist.angular.z = 3.14/4
reverse_rotate_twist.angular.z = -3.14/4
scan_forward.linear.x = 0.1
first = True
map_time = False

# initialize rate at 10 Hz
rate = rospy.Rate(10)

position = []
roll = 0
pitch = 0
yaw = 0

global ranges
global scanned
global front
global left 
global right
global back
ranges = []

# scan callback
def scan_callback(msg):
    global ranges
    global scanned
    global front
    global left 
    global right
    global back
    ranges = msg.ranges
    front = find_min(ranges, 359, 0, 20)
    left = find_min(ranges, 269, 270, 20)
    right = find_min(ranges, 89, 90, 20)
    back = find_min(ranges, 179, 180, 20)
    if front < 1:
        scanned = True
    elif back < 1:
        scanned = False
    

#finds min value for a region of a scan/helper method for scan_callback
def find_min(rang, beg, end, wedge):
    r1 = rang[slice(beg-wedge, beg)]
    r2 = rang[slice(end, end+wedge)]
    r3 = r1 + r2
    for dist in r3:
        if dist == 0:
            r3.remove(0)
        else:
            pass
    return min(r3)


column = 0
row = 0

#Tells a robot to move forward 1.5m, turn around, return to the starting location, and rotate back forwards
def odom_callback(msg):
    orientation_q = msg.pose.pose.orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    (roll, pitch, yaw) = euler_from_quaternion (orientation_list)
    position = msg.pose.pose.position
    #specifcally works in rooms with 4m width
    column = 40 - ((msg.pose.pose.position.y + 2) * 10)
    row = 40 - ((msg.pose.pose.position.x + 2) * 10)


#subscribes to Odometry and LaserScan while publishing cmd_vels
sub = rospy.Subscriber('/odom', Odometry, odom_callback)
scan_sub = rospy.Subscriber('/scan', LaserScan, scan_callback)
pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)

# Puts the robot in the upper left corner of a square world
# Assumes there are no obstacles straight forward and up
def start_up():
    global first
    global map_time
    if front < .5:
        print('hi1')
        pub.publish(rotate_twist)
    elif back < .5 and left < .5:
        print('hi2')
        first = False
        map_time = True
        pub.publish(rotate_twist)
        rospy.sleep(2.5)
        pub.publish(stop_twist)
    else:
        print('hi3')
        pub.publish(forward_twist)


# initialize a numpy array to build map on
new_map = numpy.full((40, 40), 0.0)

iter = 0

def map():
    global map_time
    global iter
    global front
    global left 
    global right
    global back
    global row
    global column
    print(left, right, front, row, column)
    pub.publish(scan_forward)
    if row == 40:
        map_time = False
        return
    else:
        pass
    if left < .5:
        print('yur')
        # which way is robot facing
        # how to only add to the row the wall is on. not the current row we are on (solved?)
        if new_map[row - int(round(left, 1) * 10), column] < 1:
            new_map[row - int(round(left, 1) * 10), column] += .1
    elif right < .5:
        if new_map[row - int(round(right, 1) * 10), column] < 1:
            new_map[row - int(round(right, 1) * 10), column] += .1
    elif front < .5:
        col = int(column + (round(front, 1) * 10))
        if new_map[row, col] < 1:
            new_map[row, col] += .1
        turn()
        for each in range(0, 40):
            print(new_map[each])
    rospy.sleep(1)
    print(new_map[0])


#turns the robot around and moves then .1m down
def turn():
    global iter
    reverse = True
    if yaw < 0:
        pub.publish(reverse_rotate_twist)
        rospy.sleep(1)
    else:
        pub.publish(rotate_twist)
        rospy.sleep(1)
        reverse = False        
    pub.publish(stop_twist)
    pub.publish(scan_forward)
    rospy.sleep(1)
    pub.publish(stop_twist)
    if reverse:
        pub.publish(reverse_rotate_twist)
        rospy.sleep(1)
    else:
        pub.publish(rotate_twist)
        rospy.sleep(1)
    stop_twist
    iter += 1


# Variable used for switching direction with roam()
switch_time = rospy.Time.now()

# Roams the space while avoiding obstacles and follow walls for short periods of time
def roam():
    global front
    global right
    if front < .5:
        obstacle = found_object()
        print(right)
        if right > .6:
            temp = Twist()
            temp.angular.z = 3.14/2
            pub.publish(temp)
            #avoid(True)
            switch_time = rospy.Time.now() + rospy.Duration(10)
        else:
            follow()
            switch_time = rospy.Time.now() + rospy.Duration(10)
    else: 
        pub.publish(forward_twist)
        

# Determines if encountered objects are obstacles of size 1m x 1m or walls
def found_object():
    print('detecting')
    global front
    global right
    print(front)
    offset = False
    temp = Twist()
    goal_yaw = 0
    if yaw > 3.14/2:
        goal_yaw = (3.14/2 - (3.14 - yaw)) + -3.14
        offset = True
    else:
        goal_yaw = yaw + (3.14/2)
    overtime = rospy.Time.now() + rospy.Duration(3)
    while rospy.Time.now() < overtime:
        temp.angular.z = 3.14/6
        pub.publish(temp)
    temp.angular.z = 0
    temp.linear.x = .11
    overtime = rospy.Time.now() + rospy.Duration(10)
    while rospy.Time.now() < overtime:
        if front < .5:
            pub.publish(stop_twist)
            return False
        else:
            pub.publish(temp)
    pub.publish(stop_twist)


# Goes around obstacles the robot finds
def avoid(first):
    goal_yaw = 0
    offset = False
    temp = Twist()
    if yaw < -3.14/2:
        goal_yaw = 3.14 - (3.14/2 + (-3.14 + yaw))
        offset = True
    else:
        goal_yaw = yaw - (3.14/2)
    overtime = rospy.Time.now() + rospy.Duration(3)
    while rospy.Time.now() < overtime:
        temp.angular.z = -3.14/6
        pub.publish(temp)
    pub.publish(stop_twist)
    overtime = rospy.Time.now() + rospy.Duration(20)
    temp.linear.x = 0.1
    while rospy.Time.now() < overtime:
        if front < .5:
            pub.publish(stop_twist)
            print('Wall in front of rover, resetting')
            return
        else:
            pub.publish(temp)
    pub.publish(stop_twist)
    if yaw < -3.14/2:
        goal_yaw = 3.14 - (3.14/2 + (-3.14 + yaw))
        offset = True
    else:
        goal_yaw = yaw - (3.14/2)
    overtime = rospy.Time.now() + rospy.Duration(3)
    while rospy.Time.now() < overtime:
        temp.angular.z = -3.14/6
        pub.publish(temp)
    pub.publish(stop_twist)
    overtime = rospy.Time.now() + rospy.Duration(10)
    temp.linear.x = 0.1
    while rospy.Time.now() < overtime:
        if front < .5:
            pub.publish(stop_twist)
            print('Wall in front of rover, resetting')
            return
        else:
            pub.publish(temp)
    pub.publish(stop_twist)


#variables for wall following
goal = .5
prev_error = 0
int_error = 0

#proportional, integral, and derivative error gains
kp = 2     
ki = .01
kd = .25

#10 hz = 10/second 
#delta between each calc is 1/hz
hz = 10
dt = 1

def follow():
    print('following')
    global int_error
    global prev_error
    twist = Twist()
    if(front < goal):
        twist.linear.x = 0
        if left > right:
            while(front < goal):
                twist.angular.z = 3.14/8
                pub.publish(twist)
            twist.angular.z = 0
        elif left < right:
            while(front < .6):
                twist.angular.z = 3.14/8
                pub.publish(twist)
            twist.angular.z = 0
        else:
            twist.linear.x = -.4
            twist.angular.z = 0
        pub.publish(twist)
    else:
        if(left > right):
            error = goal - right
        else:
            error = goal - left
        int_error = int_error + error / dt 
        der_err = (error - prev_error) / dt
        pid = kp * error + ki * int_error + kd * der_err
        #restrict pid to be within [-2, 2]
        if pid > 2:
            pid = 2
        elif pid < -2:
            pid = -2
        else:
            pid = pid
        twist.linear.x = .2
        twist.angular.z = pid * (3.14/4)
        prev_error = error
        print(pid)   


# Control loop
while not rospy.is_shutdown():
    
    

    global ranges
    global scanned
    global front
    global left 
    global right
    global back
    if len(ranges) == 0:
        pass
    else:
        if first:
            start_up()
        elif map_time:
            map()
        else: 
            roam()
    
    rate.sleep()