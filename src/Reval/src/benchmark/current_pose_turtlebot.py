import sys
import rospy
import time
from nav_msgs.msg import Odometry

def GetCurrPose(msg):
    cx = msg.pose.pose.position.x
    cy = msg.pose.pose.position.y
    with open('log/cx.txt', 'w') as f:
        f.write(str(cx))
        f.close()  
    with open('log/cy.txt', 'w') as f:
        f.write(str(cy))
        f.close()        
    rospy.wait_for_message('/odom', Odometry)


if __name__ == '__main__':
    rospy.init_node('poisition_metrics', anonymous=True)
    odom_sub = rospy.Subscriber('/odom', Odometry, GetCurrPose)
    time.sleep(0.1)
    sys.exit()
