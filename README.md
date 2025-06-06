# Cure
Causal Understanding and Remediation for Enhancing Robot Performance

## Requirements
* Ubuntu 20.04 LTS
* [ROS Noetic](http://wiki.ros.org/noetic/Installation/Ubuntu)

## Build Cure
1. Install [Reval](https://github.com/softsys4ai/cure/tree/main/src/Reval) for performance measurement
2. Please use the following commands to build cure from source
```sh
git clone https://github.com/softsys4ai/cure.git
cd ~/cure && pip install -r requirements.txt
source /opt/ros/noetic/setup.bash
catkin build
source devel/setup.bash
```
N.b. If you face `Catkin command not found`, install `sudo apt-get install python3-catkin-tools` OR you can use `catkin_make`. For completeness we included the `Husky` and `Turtlebot 3` source files in this repo to avoid complexity.

## How to use Cure
CURE is used for tasks such as performance optimization and performance debugging in robotic systems. Given the cost and human involvement associated with collecting training data from physical robots for these tasks, CURE addresses these challenges by learning the performance behavior of the robot in simulations and transferring the acquired knowledge to physical robots. CURE also works with data from physical robots, we use simulator data to evaluate the transferability of the causal model.
- In offline mode, CURE is compatible with any device utilizing Husky and Turtlebot in Gazebo environment.
- In online mode, the performance measurements are directly taken from the physical robot. In the experiments, we have used Turtlebot 3 platform. 
- Debugging mode, users can query the root cause of a certain functional and non-functional faults. For example, what is the root cause of the task failure and higher energy consumption

### Arguments
```
optional arguments:
  -h, --help            show this help message and exit
  --robot {Husky_sim,Turtlebot3_sim,Turtlebot3_phy}
                        robotic platform
  --train_data          path of the training data (.csv)
  -root_cause           diagnose root-causes
  --outlier_data        path of the anomaly data (.csv)
  --f  [ ...]           functional properties (fixed: Task_success_rate)
  --nf  [ ...]          non-functional properties (default: [Energy,
                        Positional_error])
  --top_k               number of root-causes based on average causal effect
                        (default: 5)
  -l                    diagnose root-causes from the saved model [must be True
                        to use --model]
  --model               saved model path (.model)
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
  --snap                optmization iteration to load from
  -v, --verbose         show statements during learning, effect estimation,
                        and optimization
```
### Learning causal model
The causal model was trained on 1000 observational data obtained using [Reval](https://github.com/softsys4ai/cure/tree/main/src/Reval). The following commands can be used for training and saving the causal model. The saved model can later be utilized for both inference purposes and transferring knowledge. We have already included the saved models both for Husky and Turtlebot 3 in the `cure/model` directory.
```sh
python run_cure_MOO.py --robot Turtlebot3_sim --train_data data/obs_data/turtlebot3_1000.csv 
```
### Multi-objective debugging
To find the root causes functional and non-functional faults from the saved causal model, please use the following command:
```sh
python run_cure_MOO.py --robot Turtlebot3_sim -root_cause --outlier_data data/bug/turtlebot3_outlier.csv -l --model model/care_Turtlebot3_sim.model --f Task_success_rate --nf Energy Positional_error Obstacle_distance --top_k 5
```
### Multi-objective optimization
To run multi-objective optimization using cure, please run the following command:
```
python run_cure_MOO.py --robot Turtlebot3_sim --outlier_data data/bug/turtlebot3_outlier.csv -l --model model/care_Turtlebot3_sim.model --f Task_success_rate --nf Energy Positional_error Obstacle_distance --top_k 5 -opt --f1 Energy --f2 Positional_error --f1_pref 1.2 --f2_pref 0.15 --sc 0.25 --tcr 0.8 --hv_ref_f1 19.98 --hv_ref_f2 3.0 --budget 200
```
In this example, we used `Energy` and `Positional_error` as our two objectives, and `Task_success_rate` and `Obstacle_distance` as constraints.

### Transferibility
#### Hardware
- Turtlebot 3 burger
#### Setup
1. Install the required dependencies for [Turtlebot 3](https://emanual.robotis.com/docs/en/platform/turtlebot3/overview/)
2. Run `roscore` on remote PC
3. Run `Bringup` on Turtlebot 3 SBC
```
ssh ubuntu@{IP_ADDRESS_OF_RASPBERRY_PI}
roslaunch turtlebot3_bringup turtlebot3_robot.launch
```
4. Set Turtlebot 3 model on remote PC
```
$ export TURTLEBOT3_MODEL=burger
```

5. Run Cure
To transfer the causal model learned from simulation to a physical robot `(Turtlebot 3 sim -> Turtlebot 3 real)`, please use the following command.
```
python run_cure_MOO.py --robot Turtlebot3_phy --outlier_data data/bug/turtlebot3_outlier.csv -l --model model/care_Turtlebot3_sim.model --f Task_success_rate --nf Energy Positional_error Obstacle_distance --top_k 5 -opt --f1 Energy --f2 Positional_error --f1_pref 1.2 --f2_pref 0.18 --sc 0.25 --tcr 0.8 --hv_ref_f1 19.98 --hv_ref_f2 3 --budget 50
```

### Real time results
To visualize the results in real time, please run the following commands in a separate terminal:
```sh
python live_plot.py --hv_ref_f1 19.98 --hv_ref_f2 3.2
```

## More details abour Cure
For detailed instructions to reproduce our results, please use [functionality](https://github.com/softsys4ai/cure/blob/main/doc/FUNTIONALITY.md) and [reproducibility](https://github.com/softsys4ai/cure/blob/main/doc/REPRODUCE.md)

## Cite CURE
If you use CURE in your research or the dataset in this repository please cite the following:
```
@ARTICLE{10916515,
  author={Hossen, Md Abir and Kharade, Sonam and O'Kane, Jason M. and Schmerl, Bradley and Garlan, David and Jamshidi, Pooyan},
  journal={IEEE Transactions on Robotics}, 
  title={CURE: Simulation-Augmented Autotuning in Robotics}, 
  year={2025},
  volume={41},
  pages={2825-2842},
  keywords={Robots;Optimization;Transfer learning;Robot sensing systems;Navigation;Safety;Computer bugs;Bayes methods;Tuning;Software algorithms;Causal inference;optimization;robot testing;robotics and cyber-physical systems},
  doi={10.1109/TRO.2025.3548546}
}

```
