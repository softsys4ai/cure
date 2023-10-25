#!/bin/sh
cd ..

# Extract /rosout and other topics
ros_readbagfile log/all_topics.bag /rosout > log/topics.yaml
ros_readbagfile log/all_topics.bag /battery/status > log/battery_status.yaml
sleep 3

# How many times DWA planner failed
rg 'msg: "DWA planner failed to produce path' log/topics.yaml | sort -V > log/DWA_failed.txt

# How many times DWA replanned
rg 'msg: "Invalid Trajectory' log/topics.yaml | sort -V > log/DWA_invalid_trajectory.txt

# How many DWA planner invalid trajectory
rg 'msg: "Got new plan' log/topics.yaml | sort -V > log/DWA_newplan.txt


# How many times executed rotate recovery behavior
rg 'msg: "Rotate recovery behavior started' log/topics.yaml | sort -V > log/rotate_recovery_executed.txt

# How many times executed ClearCostMaps recovery to unstuck robot
rg 'msg: "Clearing both costmaps to unstuck robot' log/topics.yaml | sort -V > log/clearCostMaps_ur_recovery_executed.txt

# How many times executed ClearCostMaps recovery to clear a layer (e.g., obstacle layer)
rg 'msg: "Recovery behavior will clear layer'  log/topics.yaml | sort -V > log/clearCostMaps_layer_recovery_executed.txt


# Once a goal position is reached... rotate to the goal orientation.
# checking if the action is leagal while rotating on goal LatchedStopRotateController::rotateToGoal
# invalid rotation cmd
rg 'msg: "Rotation cmd in collision' log/topics.yaml | sort -V > log/invalid_rotation_cmd.txt

# Error rotating on goal
rg 'msg: "Error when rotating' log/topics.yaml | sort -V > log/rotating_goal_error.txt

# Mission failed
# rg 'msg: "Aborting' log/topics.yaml | sort -V > log/mission_fail.txt

# Mission success
rg 'msg: "Goal reached' log/topics.yaml | sort -V > log/mission_success.txt

# Battery 
rg 'percentage: ' log/battery_status.yaml | sort -V > log/battery_percentage.txt

