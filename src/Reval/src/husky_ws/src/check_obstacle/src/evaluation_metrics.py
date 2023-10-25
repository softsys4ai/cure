# DO NOT skip the next commented line
#!/usr/bin/env python2

import rospy
from sensor_msgs.msg import LaserScan

def callback(LaserScan):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", LaserScan.LaserScan)

def listener_new():
    rospy.init_node('listener_new', anonymous=True)
    rospy.Subscriber("/scan", LaserScan, callback)
    rospy.spin()

if __name__=='__main__':
    listener_new()
