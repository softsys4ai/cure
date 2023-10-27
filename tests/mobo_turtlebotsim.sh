#!/bin/sh
cd ..
python run_baselineMOO.py --robot Turtlebot3_sim --f1 Energy --f2 Positional_error --f1_pref 1.0 --f2_pref 0.18 --sc 0.25 --tcr 0.8 --hv_ref_f1 19.98 --hv_ref_f2 15 --budget 200 & gnome-terminal -- python live_plot.py & python live_plot.py --hv_ref_f1 19.98 --hv_ref_f2 35