#!/usr/bin/env python

import rospy, math, json
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

# constants
LINEAR_SPEED  = 0.22        # m/s
ANGULAR_SPEED = math.pi/4   # r/s

closest_ahead = 1
direction = [1, True]

locations = {}
curr_pos = None

prev_action = ''

building = False

# update the current pose of the robot
def updatePos(msg):
    global curr_pos

    curr_pos = msg.pose

# looks at the closest object in front of the robot
def get_pov(msg):
    global closest_ahead

    # obtain the section of the lidar scan that represents what is in front of the robot
    ranges = list(msg.ranges[314:])
    ranges.extend(list(msg.ranges[:45]))

    # find the minimum value of the given range
    closest_ahead = min(ranges)




def stop_cmds():
    commands_pub.publish('^C')

    return Twist()


# save the map currently being made
def save_map(time_elapsed):
    global action_type

    commands_pub.publish('rosrun map_server map_saver -f map1')
    action_type = prev_action

    return get_vel(time_elapsed)



# wander around the environment to build the map
def wander():
    global closest_ahead
    global direction
    global building

    if not building:
        commands_pub.publish('roslaunch cr_ros_3 turtlebot3_slam.launch')
        building = True

    t = Twist()
    if closest_ahead < 0.5:
        if direction[1] == True:
            direction[1] = False
            direction[0] *= -1
        t.angular.z = 0.6 * direction[0]
    else:
        direction[1] = True
        t.linear.x = 0.4
    return t



# save a location for the robot to be able to nav to
def mark(time_elapsed):
    global locations
    global action_type

    num = intent_details.get('location')
    if num not in locations:
        print('New pose for ' + str(num))
    else:
        print('Updating pose for ' + str(num))

    locations[num] = curr_pos
    action_type = prev_action
    print_locations() # debug

    return get_vel(time_elapsed)



# print each saved location in a nice format
def print_locations():
    for i in locations:
        print('Location ' + str(i))
        print(locations[i].pose.position)
        print('')

    print('\n\n\n')



# get rotation command 
def get_rotation(time_elapsed):
    sign = -1 if intent_details.get("direction") == "right" else 1
    total_time = get_rotation_duration()
    if time_elapsed < total_time:
        t = Twist()
        t.angular.z = ANGULAR_SPEED * sign
        return t
    return Twist()



# get translation command 
def get_translation(time_elapsed):
    sign = 1 if intent_details.get("direction") == "forward" else -1
    total_time = get_translation_duration()
    if time_elapsed < total_time:
        t = Twist()
        t.linear.x = LINEAR_SPEED * sign
        return t
    return Twist()



# get duration of angular command
def get_rotation_duration():
    angle = intent_details.get("angle")
    amount = float(angle.get("amount"))
    units = angle.get("units")
    if units == "degrees":
        amount = math.radians(amount)
    seconds = amount/ANGULAR_SPEED
    return rospy.Duration(secs=seconds)



# get duration of linear command
def get_translation_duration():
    distance = intent_details.get("distance")
    amount = float(distance.get("amount"))
    units = distance.get("units")
    if units == "feet":
        amount = amount * 0.3048
    seconds = amount/LINEAR_SPEED
    return rospy.Duration(secs=seconds)



# callback to parse intent json and update global vars
def perform_action_from_intent(message):

    # record time intent was received 
    global time_received; global intent_details; global action_type
    time_received = rospy.Time.now()

    # parse action and details fields from intent json
    intent = json.loads(message.data)
    action_type = intent['action']
    intent_details = intent['details']



# get command vel for each action type
def get_vel(time_elapsed):
    global prev_action
    # print(action_type)

    # want to call set location only once and continue what the robot was previously doing
    if action_type not in ['set.loc', 'save.map']:
        prev_action = action_type

    # call correct method for each action type
    if action_type == "navigation.translation":
        return get_translation(time_elapsed)

    elif action_type == "navigation.rotation":
        return get_rotation(time_elapsed)

    elif action_type == "explore.space":
        return wander()

    elif action_type == "set.loc":
        return mark(time_elapsed)

    elif action_type == "save.map":
        return save_map(time_elapsed)

    elif action_type == "stop.cmds":
        return stop_cmds()

    else:
        return Twist()



# init node and declare pub/subs
rospy.init_node('voice_intent_handler')
cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
commands_pub = rospy.Publisher('cmds_to_run', String, queue_size=1)
intent_sub = rospy.Subscriber('voice_intents', String, perform_action_from_intent)
scan_sub = rospy.Subscriber('/scan', LaserScan, get_pov)
odom_sub = rospy.Subscriber('/odom', Odometry, updatePos)
rate = rospy.Rate(10)

# some variables
time_received = rospy.Time.now()
intent_details = None
action_type = None

# Wait for published topics, exit on ^c
while not rospy.is_shutdown():

    # calculate time since last intent
    time_elapsed = rospy.Time.now() - time_received
    
    # publish cmd_vel based on last intent
    cmd_vel_pub.publish(get_vel(time_elapsed))

    # run at 10 hz
    rate.sleep()