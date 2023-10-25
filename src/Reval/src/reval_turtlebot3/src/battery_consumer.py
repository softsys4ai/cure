#! /usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32

class BatteryConsumer:
    def __init__(self):
        rospy.init_node("battery_consumer")
        cmd_vel_subscriber = rospy.Subscriber("/cmd_vel", Twist, self.cmd_callback)
        self.battery_consumer_0 = rospy.Publisher("/battery/consumer/0", Float32, queue_size=1)
        self.moving = Float32(data=10.0)
        self.idle = Float32(data=2.5)

    def cmd_callback(self, msg):
        if msg.linear.x != 0.0 or msg.angular.z != 0.0:
            self.battery_consumer_0.publish(self.moving)
        else:
            self.battery_consumer_0.publish(self.idle)
    
    def run(self):
        rospy.spin()

def main():
    battery_consumer = BatteryConsumer()
    battery_consumer.run()

if __name__ == "__main__":
    main()