#! /usr/bin/env python

import rospy 
from gazebo_msgs.msg import ContactsState
from std_msgs.msg import Int32

class CollisionCounter:
    def __init__(self):
        rospy.init_node("collision_counter")
        bumper_subscriber = rospy.Subscriber("/bumper_vals", ContactsState, self.bumber_callback)
        self.collision_publisher = rospy.Publisher("/collisions", Int32, queue_size=10)
        rospy.Timer(rospy.Duration(10), self.reset_contact)
        self.collision_msg = Int32()
        self.occurance = False
        self.old_contact = ""

    def reset_contact(self, event):
        self.old_contact = ""

    def bumber_callback(self, msg):
        if msg.states:
            if not self.occurance:
                if self.old_contact != msg.states[0].collision2_name:
                    self.collision_msg.data+=len(msg.states)
                    self.occurance = True
                    self.old_contact = msg.states[0].collision2_name
        else:
            self.occurance = False
        self.collision_publisher.publish(self.collision_msg)

    def run(self):
        rospy.spin()

def main():
    collision_counter = CollisionCounter()
    collision_counter.run()

if __name__ == "__main__":
    main()
        