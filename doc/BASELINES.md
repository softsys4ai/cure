### MOBO `python run_baselineMOO.py -h`
```
optional arguments:
  -h, --help            show this help message and exit
  --robot {Husky_sim,Turtlebot3_sim,Turtlebot3_phy}
                        robotic platform
  --budget              number of evaluation to be performed (default: 1)
  --f1                  objectiive 1 (default: Energy)
  --f2                  objectiive 2 (default: Positional_error)
  --f1_pref             target f1 performance (default: Energy<=25.0)
  --f2_pref             target f2 performance (default: Positional_error<=0.1)
  --sc                  safety constraint, maintained distance from obstacle
                        (default: Obstacle_distance>=0.25)
  --tcr                 task completion rate (targets reached/total targets)
                        constraint (default: Task_completion_rate>=0.8)
  --hv_ref_f1           hypervolum reference for f1
  --hv_ref_f2           hypervolum reference for f2
  --init_trails         number of initial trails before fitting GP
  -viz                  show gazebo and rviz
  -l_opt                reload optimization from JSON
  --json                optmization model to load from
  -v, --verbose         show statements during optimization
```

### RidgeCV MOBO `python run_baselineSF_MOO.py -h`
```
optional arguments:
  -h, --help            show this help message and exit
  --robot {Husky_sim,Turtlebot3_sim,Turtlebot3_phy}
                        robotic platform
  --train_data          path of the training data (.csv)
  --outlier_data OUTLIER_DATA
                        path of the data (.csv)
  --f  [ ...]           functional properties (fixed: Task_success_rate)
  --nf  [ ...]          non-functional properties (default: [Energy,
                        Positional_error])
  --top_k               number of root-causes based on Ridge coeffecients
                        (default: 5)
  -l                    diagnose root-causes from the saved model [must be
                        True to use --model]
  --model               saved model path (.joblib)
  -opt                  perform optimization [must be True to use --budget,
                        --f1, --f2, --f1_pref, --f2_pref, --sc]
  --budget              number of evaluation to be performed (default: 10)
  --f1                  objectiive 1 (default: Energy)
  --f2                  objectiive 2 (default: Positional_error)
  --f1_pref             target f1 performance (default: Energy<=25.0)
  --f2_pref             target f2 performance (default: Positional_error<=0.1)
  --sc                  safety constraint, maintained distance from obstacle
                        (default: Obstacle_distance>=0.25)
  --tcr                 task completion rate (targets reached/total targets)
                        constraint (default: Task_completion_rate>=0.8)
  --hv_ref_f1           hypervolum reference for f1
  --hv_ref_f2           hypervolum reference for f2
  --init_trails         number of initial trails before fitting GP
  -viz                  show gazebo and rviz
  -l_opt                reload optimization from JSON
  --json                optmization model to load from
  -v, --verbose         show statements during optimization
```
