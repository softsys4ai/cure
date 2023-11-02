# Cure reproducibility
In this section, we discuss the steps required to reproduce our key findings reported in the paper. Please check the [Transferibility](https://github.com/softsys4ai/cure/tree/main#transferibility) section.

## Major Claims
- Cure ensures optimal performance while efficiently utilizing the allocated budget by identifying the root causes of configuration bugs.
- The causal models are transferable across different environments~(Sim-to-real) and different robotic systems Husky sim. to Turtlebot 3 physical

## Experiments
To support our claims, we perform the following experiments.

### E1: Optimizing robot performance with emphasis on faster convergence
To support this claim, we have (i) trained the causal model, (ii) generated a reduced search space by identifying the core configuration options, and (iii) performed MOBO on the reduced search space. We reproduce the results reported in RQ1. This experiment would require ~15 hours to complete. We also compare the results with baseline methods.

To run the experiment, the following commands need to be executed:
For Cure-MOBO:
```sh
python run_cure_MOO.py --robot Husky_sim -l --model model/care_Husky_sim.model --outlier_data data/bug/husky_outlier.csv -root_cause  --f Task_success_rate --nf Energy Positional_error Obstacle_distance --top_k 5 -opt --f1 Energy --f2 Positional_error --f1_pref 40.0 --f2_pref 0.18 --sc 0.25 --tcr 0.8 --hv_ref_f1 400.0 --hv_ref_f2 15 --budget 200
```
For MOBO:
```sh
python run_baselineMOO.py --robot Husky_sim --f1 Energy --f2 Positional_error --f1_pref 40.0 --f2_pref 0.18 --sc 0.25 --tcr 0.8 --hv_ref_f1 400.0 --hv_ref_f2 15 --budget 200
```
For RidgeCV-MOBO
```sh
python run_baselineSF_MOO.py --robot Husky_sim --outlier_data data/bug/husky_outlier.csv -l --model model/RidgeCV_Husky_sim_model --f Task_success_rate --nf Energy Positional_error Obstacle_distance --top_k 5 -opt --f1 Energy --f2 Positional_error --f1_pref 40.0 --f2_pref 0.18 --sc 0.25 --tcr 0.8 --hv_ref_f1 400.0 --hv_ref_f2 15 --budget 200
```
##### Results
The results reported in this paper are stored in a `CSV` file located in the `cure/cure_log` directory. Note that, during hypervolume computation, the execution might show warnings if the ovserved `f^{ref}` points are higher than the defined points. Therefore, we have computed the hypervoulme after the experiments are over from using `CSV` file. Note that this experiment is conducted once without repetition; thus, there are no error bars.

### E2: Demonstrating transferability
To support this claim, we trained the (i) causal model using observational data collected from Turtlebot 3 in simulation and reuse the causal model in Turtlebot 3 physical robot, and (ii) causal model using observational data collected from Husky in simulation and reuse the causal model in Turtlebot 3 physical robot for performance optimization. This experiment is anticipated to require ~20 to 24 hours, contingent on the time needed for the complete charging of the Turtlebot 3 physical robot’s battery during execution.

The following commands need to be executed to run the experiment.

#### Turtlebot 3 sim -> Turtlebot 3 real
For Cure-MOBO:
```sh
python run_cure_MOO.py --robot Turtlebot3_phy -root_cause --outlier_data data/bug/turtlebot3_outlier.csv -l --model model/care_Turtlebot_sim.model --f Task_success_rate --nf Energy Positional_error Obstacle_distance --top_k 5 -opt --f1 Energy --f2 Positional_error --f1_pref 1.2 --f2_pref 0.15 --sc 0.25 --tcr 0.8 --hv_ref_f1 19.98 --hv_ref_f2 3 --budget 50 --init_trails 15
```
For MOBO:
```sh
python run_baselineMOO.py --robot Turtlebot3_phy -l_opt --json model/optimodels/mobo/Husky_sim_ax_client_snapshot_201  --f1 Energy --f2 Positional_error --f1_pref 1.2 --f2_pref 0.15 --sc 0.25 --tcr 0.8 --hv_ref_f1 19.98 --hv_ref_f2 3 --budget 50
```
For RidgeCV-MOBO
```sh
python run_baselineSF_MOO.py --robot Turtlebot3_phy --outlier_data data/bug/turtlebot3_outlier.csv -l --model model/RidgeCV_Turtlebot3_sim_model --f Task_success_rate --nf Energy Positional_error Obstacle_distance --top_k 5 -opt --f1 Energy --f2 Positional_error --f1_pref 2.0 --f2_pref 0.1 --sc 0.25 --tcr 0.8 --hv_ref_f1 19.98 --hv_ref_f2 3 --budget 50 --init_trails 15
```

#### Husky sim -> Turtlebot 3 real
For Cure-MOBO:
```sh
python run_cure_MOO.py --robot Turtlebot3_phy -root_cause --outlier_data data/bug/turtlebot3_outlier.csv -l --model model/care_Husky_sim.model --f Task_success_rate --nf Energy Positional_error Obstacle_distance --top_k 5 -opt --f1 Energy --f2 Positional_error --f1_pref 2.0 --f2_pref 0.1 --sc 0.25 --tcr 0.8 --hv_ref_f1 19.98 --hv_ref_f2 3 --budget 50 --init_trails 15
```
For MOBO
```sh
python run_baselineMOO.py --robot Turtlebot3_phy -l_opt --json model/optimodels/mobo/Husky_sim_ax_client_snapshot_201  --f1 Energy --f2 Positional_error --f1_pref 1.2 --f2_pref 0.15 --sc 0.25 --tcr 0.8 --hv_ref_f1 19.98 --hv_ref_f2 3 --budget 50
```
For RidgeCV-MOBO
```sh
python run_baselineSF_MOO.py --robot Turtlebot3_phy --outlier_data data/bug/turtlebot3_outlier.csv -l --model model/RidgeCV_Husky_sim_model --f Task_success_rate --nf Energy Positional_error Obstacle_distance --top_k 5 -opt --f1 Energy --f2 Positional_error --f1_pref 2.0 --f2_pref 0.1 --sc 0.25 --tcr 0.8 --hv_ref_f1 19.98 --hv_ref_f2 3 --budget 50 --init_trails 15
```
##### Results
We store the result in a CSV file located in the .`cure/cure_log` directory. Note that this experiment is run once without repeating, so there are no error bars.

### To visualize the results in real time, please run the following commands in a seperate terminal:
```sh
python live_plot.py --hv_ref_f1 19.98 --hv_ref_f2 3.2 # For Turtlebot3
python live_plot.py --hv_ref_f1 400 --hv_ref_f2 35 # For Husky
```

