import actionlib
import rospy
import time
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from goals_turtlebot_phy import *

goal = MoveBaseGoal()
goal.target_pose.header.frame_id = 'map' 

def target(x, y, z, w):
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y
    goal.target_pose.pose.orientation.z = z
    goal.target_pose.pose.orientation.w = w


if __name__ == "__main__":
    rospy.init_node('send_client_goal')
    client = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
    client.wait_for_server()  

    pose_error = []
    start_time = time.time()

    target(home_x, home_y, home_z, home_w)
    # time.sleep(1)
    client.send_goal(goal)
    client.wait_for_result() 