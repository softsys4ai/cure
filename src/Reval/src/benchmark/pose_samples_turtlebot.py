import rospy
import pickle
from nav_msgs.msg import Odometry

x_samples = []
y_samples = []

def callback(msg):
    cx = msg.pose.pose.position.x
    cy = msg.pose.pose.position.y
    x_samples.append(cx)
    y_samples.append(cy)

    return x_samples, y_samples

def StoreSamples():
    with open('log/x_samples.ob', 'wb') as fp:
        pickle.dump(x_samples, fp)
        fp.close()  
    with open('log/y_samples.ob', 'wb') as fp:
        pickle.dump(y_samples, fp)
        fp.close()  


if __name__ == '__main__':
    rospy.init_node('poisition_metrics', anonymous=True)
    odom_sub = rospy.Subscriber('/odom', Odometry, callback)
    rate = rospy.Rate(0.5)
    rate.sleep()

    StoreSamples()
