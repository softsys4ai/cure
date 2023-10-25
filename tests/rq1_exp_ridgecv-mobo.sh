#!/bin/sh
cd ..
python run_baselineSF_MOO.py --robot Husky_sim --data data/husky_outlier.csv --f Task_success_rate --nf Energy Positional_error Obstacle_distance --top_k 5 -opt --f1 Energy --f2 Positional_error --f1_pref 40.0 --f2_pref 0.18 --sc 0.25 --tcr 0.8 --hv_ref_f1 400.0 --hv_ref_f2 15 --budget 201