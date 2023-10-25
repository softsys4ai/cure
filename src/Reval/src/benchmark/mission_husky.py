import actionlib
import rospy
import time
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from service import pose_husky
from positional_error import EuclideanDistance
from rns_husky import *
from goals_husky import *

goal = MoveBaseGoal()
goal.target_pose.header.frame_id = 'odom' 

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

    # This is where we define mission specifications
    # ----------------Target 1----------------
    target(goal1_x, goal1_y, goal1_z, goal1_w)
    client.send_goal(goal)
    client.wait_for_result()  
    pose_husky.GetCurrPose()
    cx = open("log/cx.txt", "r")
    cy = open("log/cy.txt", "r")
    curr_x = cx.read()
    curr_y = cy.read()
    curr_x = float(curr_x)
    curr_y = float(curr_y)
    rns(curr_x, goal1_x)
    pose_error.append(EuclideanDistance(goal1_x, goal1_y, curr_x, curr_y)) 
    cx.close()

   # ----------------Target 2----------------
    target(goal2_x, goal2_y, goal2_z, goal2_w)
    client.send_goal(goal)
    client.wait_for_result()   
    pose_husky.GetCurrPose()
    cx = open("log/cx.txt", "r")
    cy = open("log/cy.txt", "r")
    curr_x = cx.read()
    curr_y = cy.read()
    curr_x = float(curr_x)
    curr_y = float(curr_y)    
    rns(curr_x, goal2_x)
    pose_error.append(EuclideanDistance(goal2_x, goal2_y, curr_x, curr_y))
    time.sleep(0.5)
    cx.close()

    # ----------------Target 3----------------
    target(goal3_x, goal3_y, goal3_z, goal3_w)
    client.send_goal(goal)
    client.wait_for_result()   
    pose_husky.GetCurrPose()
    cx = open("log/cx.txt", "r")
    cy = open("log/cy.txt", "r")
    curr_x = cx.read()
    curr_y = cy.read()
    curr_x = float(curr_x)
    curr_y = float(curr_y)    
    rns(curr_x, goal3_x)  
    pose_error.append(EuclideanDistance(goal3_x, goal3_y, curr_x, curr_y))
    cx.close()

    # ----------------Target 4----------------
    target(goal4_x, goal4_y, goal4_z, goal4_w)
    client.send_goal(goal)
    client.wait_for_result()      
    pose_husky.GetCurrPose()
    cx = open("log/cx.txt", "r")
    cy = open("log/cy.txt", "r")
    curr_x = cx.read()
    curr_y = cy.read()
    curr_x = float(curr_x)
    curr_y = float(curr_y)    
    rns(curr_x, goal4_x)  
    pose_error.append(EuclideanDistance(goal4_x, goal4_y, curr_x, curr_y))
    cx.close() 

    # ----------------Target 5----------------
    target(goal5_x, goal5_y, goal5_z, goal5_w)
    client.send_goal(goal)
    client.wait_for_result()       
    pose_husky.GetCurrPose()
    cx = open("log/cx.txt", "r")
    cy = open("log/cy.txt", "r")
    curr_x = cx.read()
    curr_y = cy.read()
    curr_x = float(curr_x)
    curr_y = float(curr_y)    
    rns(curr_x, goal5_x)
    pose_error.append(EuclideanDistance(goal5_x, goal5_y, curr_x, curr_y))
    cx.close()

    time.sleep(0.5)
    CalcRns() 
    ResetRNS()

    end_time = round((time.time() - start_time), 2)
    avg_pose_error = sum(pose_error) / len(pose_error)
    with open('log/mission_time.txt', 'w') as f:
        f.write(str(end_time))
        f.close()
    with open('log/euclidean_distance.txt', 'w') as f:
        f.write(str(avg_pose_error))
        f.close()         
