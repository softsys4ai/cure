#!/bin/sh
cd ..
python run_cure_MOO.py --robot Turtlebot3_phy -root_cause --outlier_data data/bug/turtlebot3_outlier.csv -l --model model/care_Turtlebot_sim.model --f Task_success_rate --nf Energy Positional_error Obstacle_distance --top_k 5 -opt --f1 Energy --f2 Positional_error --f1_pref 2.0 --f2_pref 0.1 --sc 0.25 --tcr 0.8 --hv_ref_f1 19.98 --hv_ref_f2 3.0 --budget 50 --init_trails 15
