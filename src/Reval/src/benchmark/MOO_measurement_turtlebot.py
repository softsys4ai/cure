import os
import time
import pandas as pd
import pickle
from datetime import datetime
from tabulate import tabulate
from goals_turtlebot import *

class bcolors:
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKCYAN = '\033[96m'
    OKGREEN = '\033[92m'
    CGREEN  = '\33[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'

def penalty(obs_dsit, sc):  
    if obs_dsit < sc:
        return abs(-10.00*obs_dsit + 2.500)
    else:
        return 0
def normalize_list(input_list):
    # Find the minimum and maximum values in the list
    min_val = min(input_list)
    max_val = max(input_list)
    # Normalize the list
    normalized_list = [(x - min_val) / (max_val - min_val) for x in input_list]
    return normalized_list 

# Looking for these occurances
DWA_paln_fail = 'msg: "DWA planner failed to produce path."'
DWA_new_plan = 'msg: "Got new plan"'
DWA_invalid_trajectory = 'msg: "Invalid Trajectory'
rotate_recovery = 'msg: "Rotate recovery behavior started."'
ClearCostMaps_ur_recovery = 'msg: "Clearing both costmaps to unstuck robot'
clearCostMaps_layer_recovery = 'msg: "Recovery behavior will clear layer'
rotation_cmd_collision = 'msg: "Rotation cmd in collision'
error_rotating = 'msg: "Error when rotating'
success = 'msg: "Goal reached'

# battery percentage post-processing
skip_word_bp = "percentage: "
 
# opening a text file
DWA_failed = open("log/DWA_failed.txt", "r")
DWA_plan = open("log/DWA_newplan.txt", "r")
DWA_trajectory = open("log/DWA_invalid_trajectory.txt", "r")
rotate_recovery_executed = open("log/rotate_recovery_executed.txt", "r")
clearCostMaps_ur_recovery_executed = open("log/clearCostMaps_ur_recovery_executed.txt", "r")
clearCostMaps_layer_recovery_executed = open("log/clearCostMaps_layer_recovery_executed.txt", "r")
invalid_rotation_cmd = open("log/invalid_rotation_cmd.txt", "r")
rotating_goal_error = open("log/rotating_goal_error.txt", "r")
rns = open("log/robustness_narrow_space.txt", "r")
euclidean_distance = open("log/euclidean_distance.txt", "r")
traveled_distance = open("log/traveled distance.txt", "r")
mission_time = open("log/mission_time.txt", "r")
bat_percentage = open("log/battery_percentage.txt", "r")
mission_success = open("log/mission_success.txt", "r")

# read file content
read_DWA_failed = DWA_failed.read()
read_DWA_plan = DWA_plan.read()
read_DWA_trajectory = DWA_trajectory.read()
read_rotate_recovery_executed = rotate_recovery_executed.read()
read_clearCostMaps_ur_recovery_executed = clearCostMaps_ur_recovery_executed.read()
read_clearCostMaps_layer_recovery = clearCostMaps_layer_recovery_executed.read()
read_invalid_rotation_cmd = invalid_rotation_cmd.read()
read_rotating_goal_error = rotating_goal_error.read()
read_mission_success = mission_success.read()

# Count occurance
total_DWA_failed_occurrences = read_DWA_failed.count(DWA_paln_fail)
total_DWA_newplan_occurances = read_DWA_plan.count(DWA_new_plan)
total_DWA_invalid_trajectory = read_DWA_trajectory.count(DWA_invalid_trajectory)
total_rotate_recovery_executed = read_rotate_recovery_executed.count(rotate_recovery)
total_clearCostMaps_ur_recovery_executed = read_clearCostMaps_ur_recovery_executed.count(ClearCostMaps_ur_recovery)
total_clearCostMaps_layer_recovery = read_clearCostMaps_layer_recovery.count(clearCostMaps_layer_recovery)
total_invalid_rotation_cmd = read_invalid_rotation_cmd.count(rotation_cmd_collision)
total_rotating_goal_error = read_rotating_goal_error.count(error_rotating)
read_rns = rns.read()
read_euclidean_distance = euclidean_distance.read()
read_traveled_distance = traveled_distance.read()
read_mission_time = mission_time.read()

# determining mission success
total_mission_success  = read_mission_success.count(success)
read_rns = total_mission_success / target_locations
# If all target reached then mission is a success
if total_mission_success == target_locations:
    ms = 1
else:
    ms = 0   
# battery
first_line = bat_percentage.readline().strip()
battery_percentage = first_line.replace(skip_word_bp, '')
if battery_percentage == '-inf' or battery_percentage == 'inf':
    if ms == 1:
        battery_percentage = 85.0
    if ms == 0:
        battery_percentage = 10.0   
battery_percentage = float(battery_percentage)
battery_percentage = round(battery_percentage, 2)    

# safety metrics
with open("log/obs_distance_all.ob", 'rb') as fr:
    obs_distance = pickle.load(fr)
with open("log/sc.ob", 'rb') as fr:
    sc = pickle.load(fr)    
p = []
obs_dis = []
for i in range(len(obs_distance)):
    obs_dis.append(obs_distance[i][0])
    p.append(penalty(obs_distance[i][0], sc))  
if sum(p) != 0:     
    Pentalty = sum(normalize_list(p)) / len(normalize_list(p))
    log_p = normalize_list(p)
else:
    Pentalty = 0
    log_p = p
df_pen = pd.DataFrame({"Penalty":log_p})
df_obs_dis = pd.DataFrame({"Penalty":obs_dis})
if not os.path.isfile('../../../../cure_log/penalty.csv'):
    df_pen.to_csv('../../../../cure_log/penalty.csv', index=False)
else:
    df_pen.to_csv('../../../../cure_log/penalty.csv', mode='a', header=False, index=False)  
    
if not os.path.isfile('../../../../cure_log/obs_distance.csv'):
    df_obs_dis.to_csv('../../../../cure_log/obs_distance.csv', index=False)
else:
    df_obs_dis.to_csv('../../../../cure_log/obs_distance.csv', mode='a', header=False, index=False) 

df_eval = pd.DataFrame({
                "Planner_failed":[total_DWA_failed_occurrences], 
                "DWA_new plan":[total_DWA_newplan_occurances],
                "DWA_invalid_trajectory":[total_DWA_invalid_trajectory],
                "Recovery_executed":[total_rotate_recovery_executed], 
                "ClearCostMaps_unstuck_recovery_executed":[total_clearCostMaps_ur_recovery_executed],
                "ClearCostMaps_layer_recovery_executed":[total_clearCostMaps_layer_recovery],
                "Invalid_rotation_cmd":[total_invalid_rotation_cmd],
                "Error_rotating_goal":[total_rotating_goal_error],
                "Positional_error":[read_euclidean_distance],
                "RNS":[read_rns],
                "Traveled_distance":[read_traveled_distance],
                "Mission_time":[read_mission_time],
                "Energy":[battery_percentage],
                "Obstacle_distance":min(obs_distance),
                "Mission_success":[ms],
                "Penalty":[Pentalty]
                })               

df_eval.to_csv("../../../../cure_log/MOBO_measurement_Turtlebot3_sim.csv", index=False)

# closing a file
DWA_failed.close() 
DWA_plan.close()
DWA_trajectory.close()
rotate_recovery_executed.close()
clearCostMaps_ur_recovery_executed.close()
clearCostMaps_layer_recovery_executed.close()
invalid_rotation_cmd.close()
rotating_goal_error.close()
rns.close()
euclidean_distance.close()
traveled_distance.close()
mission_time.close()
bat_percentage.close()
mission_success.close()
