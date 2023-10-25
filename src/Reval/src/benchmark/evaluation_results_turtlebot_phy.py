import os
import pandas as pd
import pickle
from datetime import datetime
from tabulate import tabulate
from goals_turtlebot_phy import *

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

# battery percentage post-processing
with open("log/tbot_voltage.ob", 'rb') as fr:
    tbot_voltage = pickle.load(fr)
voltage = tbot_voltage[0] - tbot_voltage[-1]
max_voltage = 12.24 # observed full charge voltage
cutoff_voltage = 9.0
nominal_voltage = 11.1
battery_percentage = 19.98 * (voltage/max_voltage)


# determining mission success
total_mission_success  = read_mission_success.count(success)
read_rns = total_mission_success / target_locations
# If all target reached then mission is a success
if total_mission_success == target_locations:
    ms = 1
else:
    ms = 0    

# safety metrics
with open("log/obs_distance_all.ob", 'rb') as fr:
    obs_distance = min(pickle.load(fr))

df_eval = pd.DataFrame({
                "Planner_failed":[total_DWA_failed_occurrences], 
                "DWA_new_plan":[total_DWA_newplan_occurances],
                "DWA_invalid_trajectory":[total_DWA_invalid_trajectory],
                "Recovery_executed":[total_rotate_recovery_executed], 
                "ClearCostMaps_unstuck_recovery_executed":[total_clearCostMaps_ur_recovery_executed],
                "ClearCostMaps_layer_recovery_executed":[total_clearCostMaps_layer_recovery],
                "Invalid_rotation_cmd":[total_invalid_rotation_cmd],
                "Error_rotating_goal":[total_rotating_goal_error],
                "Positional_error":[read_euclidean_distance],
                "Task_success_rate":[read_rns],
                "Traveled_distance":[read_traveled_distance],
                "Mission_time":[read_mission_time],
                "Energy":[battery_percentage],
                "Obstacle_distance":obs_distance,
                "Mission_success":[ms]
                })                
df_config = pd.read_csv('log/config_turtlebot.csv')
df_result = pd.concat([df_config, df_eval], axis=1)
if not os.path.isfile('log/eval_turtlebot.csv'):            
    df_result.to_csv("log/eval_turtlebot.csv", mode='a', index=False, header=True) 
else:
    df_result.to_csv("log/eval_turtlebot.csv", mode='a', index=False, header=False)

df = pd.read_csv('log/eval_turtlebot.csv')
timestamp = datetime.now().strftime("%Y_%m_%d-%I_%M_%S_%p") 
df.to_csv(f"../../results/Evaluation_results_turtlebot_{timestamp}.csv", mode='a', index=False, header=True) 

df_table = pd.read_csv('log/eval_turtlebot.csv')
print(tabulate(df_table.loc[:, 'Task_success_rate':'Mission_success'], headers='keys', tablefmt='grid'))

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
mission_success.close()
