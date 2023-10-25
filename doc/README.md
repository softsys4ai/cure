## Cure funtionality
In this section, we introduce Cure with a walkthrough example. Initially we will get to familiarize with the directory structure used in this repository.

### Structure
```
cure
  |-- ananke (contains third party code for causal effect estimation)
  |-- config (contains search space for Husky and Turtlebot 3)
  |-- data
  |   |-- obs_data (contains observational data used to train the causal models)
  |   |-- bug (contains outlier samples)
  |   |-- results (contains results for each method)
  |   |   |-- cure_results
  |   |   |   |-- platform  
  |   |   |   |   |-- model
  |   |   |   |   |   |-- causal models
  |   |   |   |   |   |-- optimization_snapshot 
  |   |   |-- mobo_results
  |   |   |   |-- platform  
  |   |   |   |   |-- model
  |   |   |   |   |   |-- optimization_snapshot 
  |   |   |-- mobo_results
  |   |   |   |-- platform  
  |   |   |   |   |-- model
  |   |   |   |   |   |-- optimization_snapshot                      
  |-- fig (contains plots used in the paper)
  |-- model (contains pre-trained causal models)  
  |-- src (contains necessary code to implement cure and other baselines)
  |   |-- config (conatins configs used in optimization)
  |   |-- Reval (API used to perform measurements from robots)
  |-- tests (contains necessary code to run and test unicorn)

```
### Modes
- `Simulation mode` CURE is compatible with any device utilizing Husky and Turtlebot 3 in Gazebo environment.
  - Debugging
  - Multi-objective optimzation
- `Reality mode` the performance measurements are directly taken from the physical robot.
  - Debugging
  - Multi-objective optimzation
- `Debugging mode` users can query the root cause of a certain functional and non-functional faults.

### Functionality testing
Once the pre-requisites are installed clone the repo and navigate to the root directory using the following:
```
git clone https://github.com/softsys4ai/cure.git
cd cure
```
Consider an user deoployed a mobile robot to a new environment and definied the following task specification:
- navigate to target locations
- reduced energy consumption
- higher positional accuracy
- ensure safety 

After deployment. the user encounters `energy` and `task completion` faults as the following:  

```
**Example Bug**
----------------------------------------------
controller_frequency	        5.73
planner_patience	            6.37
controller_patience	          6.23
conservative_reset_dist	      2.86
acc_lim_theta	                2.66
acc_lim_trans	                0.21
acc_lim_x	                    2.88
forward_point_distance	      0.63
goal_distance_bias	          5.28
max_scaling_factor	          0.45
max_vel_theta	                1.29
max_vel_trans	                0.58
max_vel_x	                    0.49
min_vel_theta	                2.69
min_vel_trans	                0.17
min_vel_x	                    0
occdist_scale	                0.17
oscillation_reset_angle	      0.23
path_distance_bias	          33.48
scaling_speed	                0.35
sim_granularity	              0.03
sim_time	                    1.65
stop_time_buffer	            1.13
theta_stopped_vel	            0.06
trans_stopped_vel	            0.08
vth_samples	                  20
vx_samples	                  3
vy_samples	                  6
publish_frequency	            3.81
resolution	                  0.09
transform_tolerance	          1.15
update_frequency	            4.1
cost_scaling_factor	          10.3
inflation_radius	            1.19
Planner_failed	              117
Recovery_executed	            13
Positional_error	            2.14
Traveled_distance	            204.86
Mission_time	                1749.19
Obstacle_distance	            0.4
Energy	                      364.04
Task_success_rate	            0
----------------------------------------------
```
The reported `energy=364.04 Wh`,  `Task_success_rate=0`, and `Positional_error=2.14 meter`. To ensure optimial performance of the robot, the user utilizes Cure to determine the root cause of such performance and expects to achieve `energy<=40 Wh`,  `Task_success_rate>=0.8`, and `Positional_error<=0.18 meter`. Optionally, the user can also use constraint on `Obstacle_distance<=0.25 meter` to ensure safety. 

To avoid constant human supervision and effort required to perform execution in physical robots, we collect observation data in simulation to learn the causal model using [Reval](https://github.com/softsys4ai/cure/blob/main/src/Reval/README.md). We already included the observational data in `cure/data` directory.

#### Learn causal model
Learning the causal model from observational collected in simulation
```
python run_cure_MOO.py --robot Turtlebot3_sim --train_data data/obs_data/turtlebot3_1000.csv --outlier_data data/bug/turtlebot_outlier.csv -root_cause --f Task_success_rate --nf Energy Positional_error Obstacle_distance --top_k 5

```

