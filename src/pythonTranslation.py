import pickle

def srcipt_gen():
    with open('cure_log/parameters.ob', 'rb') as fp:
        p = pickle.load(fp) 

    with open('cure_log/nodes.ob', 'rb') as fp:
        node = pickle.load(fp) 

    fname = 'src/SFAXMO.py'

    char_remov = ["]", "[", "'", "'"]
    argv = str(p)
    for char in char_remov:
        argv = argv.replace(char, "")

    char_remov_node = ["]", "[", '"']
    move_base = str(node['MoveBase'])
    dwa = str(node['DWAPlannerROS'])
    cmap_common = str(node['costmap_common'])
    cmap_common_inf = str(node['costmap_common_inflation'])

    for char in char_remov_node:
        move_base = move_base.replace(char, "")
    move_base = f"MoveBase = "+"{"+move_base+"}"

    for char in char_remov_node:
        dwa = dwa.replace(char, "")
    dwa = f"DWAPlannerROS = "+"{"+dwa+"}"

    for char in char_remov_node:
        cmap_common = cmap_common.replace(char, "")
    cmap_common = f"costmap_common = "+"{"+cmap_common+"}"

    for char in char_remov_node:
        cmap_common_inf = cmap_common_inf.replace(char, "")
    cmap_common_inf = f"costmap_common_inflation = "+"{"+cmap_common_inf+"}"

    # Launching robot
    with open(fname, 'w') as f:
        f.write('''
import os 
import numpy as np
import pandas as pd
from src.measurement import TurtlebotSimMeasurement, HuskySimMeasurement, TurtlebotPhyMeasurement
from ax.service.ax_client import AxClient
from ax.service.utils.instantiation import ObjectiveProperties
from tabulate import tabulate
import pygmo as pg


class SFAXMO():
    def __init__(self, robot:str, viz:bool):
        print("[STATUS]: Intitializing AXMO class")
        global params
        if viz:
            self.v = True
        else:
            self.v = False    
        if robot == 'Turtlebot3_sim':
            self.s_robot = 'Turtlebot3_sim'
        if robot == 'Husky_sim':
            self.s_robot = 'Husky_sim'
        if robot == 'Turtlebot3_phy':
            self.s_robot = 'Turtlebot3_phy' 
        from src.config.AXMO_config_dynamic import params  
        '''
        )        

    # Measurement function
    with open(fname, "a") as myfile:
        myfile.write(f"\n\n    def measurement(self, {argv}):")

    with open(fname, "a") as myfile:
        myfile.write(''' 
        """ This function is used for measurement from robot
        """ 
        
        print("")
        if self.s_robot == 'Turtlebot3_sim':
            TM = TurtlebotSimMeasurement()
            TM.launch_turtlebot3_sim(self.v)
        if self.s_robot == 'Husky_sim':
            TM = HuskySimMeasurement()
            TM.launch_husky_sim(self.v)
        if self.s_robot == 'Turtlebot3_phy':
            TM = TurtlebotPhyMeasurement()  
            TM.launch_turtlebot3_phy(self.v) 
    ''')
    # ROS nodes                 
    with open(fname, "a") as myfile:
        myfile.write(f"    {move_base}\n        {dwa}\n        {cmap_common}\n        {cmap_common_inf}")  
    with open(fname, "a") as myfile:
        myfile.write(''' 
        if self.s_robot == 'Turtlebot3_sim':
            TM.tbot_dynamic_reconfig(MoveBase=MoveBase, DWAPlannerROS=DWAPlannerROS, costmap_common=costmap_common, 
                                costmap_common_inflation=costmap_common_inflation)
            TM.launch_turtlebot3_task()
            (energy, mission_time, traveled_distance, 
            planner_failed, euc_distance, recovery_executed, 
            task_completion, safety, task_completion_rate, penalty)                           = TM.tbot_get_measurement()             
        if self.s_robot == 'Husky_sim':
            TM.husky_dynamic_reconfig(MoveBase=MoveBase, DWAPlannerROS=DWAPlannerROS, costmap_common=costmap_common, 
                                costmap_common_inflation=costmap_common_inflation)
            TM.launch_husky_task()
            (energy, mission_time, traveled_distance, 
            planner_failed, euc_distance, recovery_executed, 
            task_completion, safety, task_completion_rate, penalty)                           = TM.husky_get_measurement()             
        if self.s_robot == 'Turtlebot3_phy':
            TM.tbot_phy_dynamic_reconfig(MoveBase=MoveBase, DWAPlannerROS=DWAPlannerROS, costmap_common=costmap_common, 
                                costmap_common_inflation=costmap_common_inflation)
            TM.launch_turtlebot3_phy_task()
            (energy, mission_time, traveled_distance, 
            planner_failed, euc_distance, recovery_executed, 
            task_completion, safety, task_completion_rate, pentalty)                           = TM.tbot_phy_get_measurement()   
        return (energy, mission_time, traveled_distance,
                planner_failed, euc_distance, recovery_executed,
                task_completion, safety, task_completion_rate, penalty)
    ''')

    # Evaluate function
    with open(fname, "a") as myfile:
        myfile.write(f"\n    def evaluate(self, parameters, f1:str, f2:str, hv_ref_f1:float, hv_ref_f2:float):\n        evaluation = self.measurement(")  
        for i in range(len(p)):    
            myfile.write(f"\n                     parameters.get('{p[i]}'),")   
        myfile.write(f")")    
    with open(fname, "a") as myfile:
        myfile.write(''' 
        if f2 == "Energy":              
            f2_eval_index = 0
        if f2 == "Mission_time": 
            f2_eval_index = 1     
        if f2 == "Traveled_distance":
            f2_eval_index = 2
        if f2 == "Planner_failed":
            f2_eval_index = 3
        if f2 == "Positional_error":
            f2_eval_index = 4
        if f2 == "Recovery_executed": 
            f2_eval_index = 5

        if f1 == "Energy": 
            f1_eval_index = 0         
        if f1 == "Mission_time":
            f1_eval_index = 1
        if f1 == "Traveled_distance":
            f1_eval_index = 2
        if f1 == "Planner_failed":
            f1_eval_index = 3
        if f1 == "Positional_error":
            f1_eval_index = 4
        if f1 == "Recovery_executed": 
            f1_eval_index = 5                    
        hyp = pg.hypervolume([[evaluation[f1_eval_index], evaluation[f2_eval_index]]])
        try:
            # print("[WARN]: HV computation may be inaccurate. Compute HV separately!")
            self.hv = (hyp.compute([hv_ref_f1, hv_ref_f2]) / np.prod([hv_ref_f1, hv_ref_f2]))
        except:
            self.hv = 0
            print(f"[WARN]: Hypervolume reference violated! (logged HV {self.hv})")
        self.obj_f1 = evaluation[f1_eval_index]
        self.obj_f2 = evaluation[f2_eval_index]
        self.task_success = evaluation[6]
        self.min_distance = evaluation[7]
        self.penalty = evaluation[9]
        self.task_success_rate = evaluation[8] - self.penalty 
        if self.penalty != 0:
            print("[STATUS]: Penalized Tcr")

        return {f"{f1}": (evaluation[f1_eval_index]),
                f"{f2}": (evaluation[f2_eval_index]),
                "Task_completion": (evaluation[6]),
                "Obstacle_distance": (evaluation[7]),
                "Task_completion_rate": (evaluation[8] - self.penalty),
                "Penalty_track": (evaluation[9]),     
                "Energy_track": (evaluation[0]),
                "Mission_time_track": (evaluation[1]),
                "Traveled_distance_track": (evaluation[2]),
                "Planner_failed_track": (evaluation[3]),
                "Positional_error_track": (evaluation[4]),
                "Recovery_executed_track": (evaluation[5]),
                }  
    ''') 

    # Etc
    with open(fname, "a") as myfile:
        myfile.write(''' 
        
    def ax(self, robot:str, n_iter:int, f1:str, f2:str, f1_pref:float, 
           f2_pref:float, safety_constarint:str, task_completion_constraint:str, l_opt:bool, json:str, 
           hv_ref_f1:float, hv_ref_f2:float, init_trails:int, verbose_logging:bool):           
        self.f1 = f1
        self.f2 = f2
        self.safety_constarint = safety_constarint
        if l_opt:
            ax_client = AxClient(verbose_logging=verbose_logging)
            ax_client = AxClient.load_from_json_file(filepath=f'{json}.json')
            print("[STATUS]: Optimization snapshot loaded!")
            ax_client.make_experiment(
                parameters=params,
                objectives={f"{self.f1}": "minimize", f"{self.f2}": "minimize"},
                objective_thresholds = [f"{self.f1} <= {f1_pref}", f"{self.f2} <= {f2_pref}"],
                outcome_constraints=[f"{self.safety_constarint}", 
                                    f"{task_completion_constraint}",
                                    ],
                tracking_metric_names=["Task_completion", "Energy_track", "Mission_time_track",
                                       "Traveled_distance_track", "Planner_failed_track",
                                       "Positional_error_track", "Recovery_executed_track", 
                                       "Penalty_track"],                       
            )         
        else:    
            ax_client = AxClient(verbose_logging=verbose_logging)
            ax_client.create_experiment(
                name="moo_experiment",
                parameters=params,
                objectives={
                    # `threshold` arguments are optional
                    f"{f1}": ObjectiveProperties(minimize=True, threshold=f1_pref),
                    f"{f2}": ObjectiveProperties(minimize=True, threshold=f2_pref),
                },
                outcome_constraints=[f"{self.safety_constarint}", 
                                    f"{task_completion_constraint}",
                                    ],
                tracking_metric_names=["Task_completion", "Energy_track", "Mission_time_track",
                                       "Traveled_distance_track", "Planner_failed_track",
                                       "Positional_error_track", "Recovery_executed_track", 
                                       "Penalty_track"],
                choose_generation_strategy_kwargs = {"num_initialization_trials": init_trails},     
    )

        for i in range(n_iter):
            parameters, trial_index = ax_client.get_next_trial()
            # Local evaluation here can be replaced with deployment to external system.
            ax_client.complete_trial(trial_index=trial_index, raw_data=self.evaluate(parameters, f1, f2, hv_ref_f1, hv_ref_f2))
            result = {"Iteration": (i+1),
                      f"{self.f1}": (self.obj_f1),
                      f"{self.f2}": (self.obj_f2),
                      "Task_success": (self.task_success),
                      "Obstacle_distance": (self.min_distance),
                      "Penalty": (self.penalty),
                      "Task_success_rate": (self.task_success_rate),
                      "HV": (round(self.hv, 2)),
                      "Parameters": (parameters)}
            table = {"Iteration": (i+1),
                     "Task_success": (self.task_success),
                      f"{self.f1}": (self.obj_f1),
                      f"{self.f2}": (self.obj_f2),
                      f"{self.safety_constarint}": (self.min_distance),
                      "Task_success_rate": (self.task_success_rate),
                      "Penalty": (self.penalty),
                      "HV": (self.hv)}            
            df = pd.DataFrame(table.items())
            self._log_results(result['Iteration'],
                              result[f"{self.f1}"],
                              result[f"{self.f2}"],
                              result['Task_success'],
                              result['Task_success_rate'],
                              result["Obstacle_distance"],
                              result["Penalty"],
                              result['HV'],
                              result['Parameters'],
                              'cure_log/AXMO_results.csv')
            print(tabulate(df, tablefmt='github', showindex=False))
            ax_client.save_to_json_file(filepath=f'model/optimization_snapshot/{robot}_ax_client_snapshot_{trial_index+1}.json')
        print("[STATUS]: Optimization done!")
        return ax_client.experiment    


    def _log_results(self, iteration, f1, f2, task, task_success_rate, safety, penalty, HV, params, filename):
        df = pd.DataFrame({"Iteration":iteration, 
                           f"{self.f1}":f1, f"{self.f2}":f2, 
                           "Obstacle_distance":safety,
                           "Task success":task, 
                           "Task_success_rate":task_success_rate,
                           "Penalty":penalty,
                           "HV":HV, "Parameters":[params]})
        if not os.path.isfile(filename):
            df.to_csv(filename, index=False)
        else:
            df.to_csv(filename, mode='a', header=False, index=False)
    ''')