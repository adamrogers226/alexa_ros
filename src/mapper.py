import rospy
import math
import numpy 
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, OccupancyGrid
from tf.transformations import euler_from_quaternion, quaternion_from_euler


#Initialize as a node 
rospy.init_node("mapper")

# initialize rate at 10 Hz
rate = rospy.Rate(10)

# initialize a numpy array to build map on
new_map = numpy.full((40, 40), 0.0)

roll = 0
pitch = 0
yaw = 0
row = 0
column = 0
inf = float('inf')

def odom_callback(msg):
    orientation_q = msg.pose.pose.orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    (roll, pitch, yaw) = euler_from_quaternion (orientation_list)
    row = msg.pose.pose.position.x * 2
    column = msg.pose.pose.position.y * 2


def scan_callback(msg):
    global inf 
    ranges = msg.ranges
    count = 0
    for dist in ranges:
        if dist == 0:
            pass
        elif dist == inf:
            pass
        else:
            map_add(dist, count)
        count += 1


def map_add(distance, angle):

    (x2,y2) = (round(row + distance*math.cos(angle), 1), round(column + distance*math.sin(angle), 1))
    x2 *= 10
    y2 *= 10

    new_map[(int(x2) + row), (int(y2) + column)] += .1
    print('---------------------------------------------')
    for x in range(0,40):
        print((new_map[x]))


        

odom_sub = rospy.Subscriber('/odom', Odometry, odom_callback)
scan_sub = rospy.Subscriber('/scan', LaserScan, scan_callback)
pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)

while not rospy.is_shutdown():
    
    
    rate.sleep()



'''width      --  Number of columns in the occupancy grid.
        height     --  Number of rows in the occupancy grid.
        resolution --  Width of each grid square in meters. 
        origin_x   --  Position of the grid cell (0,0) in 
        origin_y   --    in the map coordinate system.
        grid       --  numpy array with height rows and width columns.'''