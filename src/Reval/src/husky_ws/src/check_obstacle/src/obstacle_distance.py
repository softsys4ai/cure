#!/usr/bin/env python2
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import time



def callback(msg):
    average_distance = []
    obstacle_distance = msg.ranges[360]
    average_distance.append(obstacle_distance)
    average_distance = sum(average_distance)
    print("Obstacle distance using LiDAR: " + str(obstacle_distance))
    print(average_distance)
     


    # move.linear.x = 0.1
    # if msg.ranges[360] < 1:
    #     move.linear.x = 0
    # pub.publish(move)

rospy.init_node('check_obstacle')
sub = rospy.Subscriber('/scan', LaserScan, callback)
# pub = rospy.Publisher('/cmd_vel', Twist)
# move = Twist()

rospy.spin()