#!/usr/bin/env python
import actionlib
import rospy
import time
import subprocess
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from service import pose_turtlebot
from positional_error import EuclideanDistance
from rns_turtlebot import *
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

    # This is where we define mission specifications
    # ----------------Target 1----------------
    target(goal1_x, goal1_y, goal1_z, goal1_w)
    # time.sleep(1)
    client.send_goal(goal)
    client.wait_for_result()  
    pose_turtlebot.GetCurrPoseAMCL()
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
    # target(goal4_x, goal4_y, goal4_z, goal4_w)
    # client.send_goal(goal)
    # time.sleep(8)
    # client.wait_for_result()      
    # pose_turtlebot.GetCurrPoseAMCL()
    # cx = open("log/cx.txt", "r")
    # cy = open("log/cy.txt", "r")
    # curr_x = cx.read()
    # curr_y = cy.read()
    # curr_x = float(curr_x)
    # curr_y = float(curr_y)    
    # rns(curr_x, goal4_x)  
    # pose_error.append(EuclideanDistance(goal4_x, goal4_y, curr_x, curr_y))  
    # cx.close() 


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

