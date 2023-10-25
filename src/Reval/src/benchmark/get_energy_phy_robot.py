import rospy
import pickle
from sensor_msgs.msg import BatteryState

voltage = []
def get_voltage(msg):     
    voltage.append(msg.voltage)
    with open('log/tbot_voltage.ob', 'wb') as fp:
        pickle.dump(voltage, fp)
        fp.close() 
        print("voltage:",msg.voltage)

if __name__ == '__main__':
    rospy.init_node('reval_tbot_battery', anonymous=False)
    odom_sub = rospy.Subscriber('/battery_state', BatteryState, get_voltage)
    rospy.spin()