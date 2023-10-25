import os 
import numpy as np
import pandas as pd
from src.measurement import TurtlebotSimMeasurement, HuskySimMeasurement, TurtlebotPhyMeasurement
from ax.service.ax_client import AxClient
from ax.modelbridge import dispatch_utils
from ax.service.utils.instantiation import ObjectiveProperties
from ax.runners.synthetic import SyntheticRunner
from ax.plot.pareto_utils import compute_posterior_pareto_frontier, get_observed_pareto_frontiers
from tabulate import tabulate
import pygmo as pg


class AXMO():
    def __init__(self, robot:str, viz:bool):
        print("[STATUS]: Intitializing AXMO class")
        global params
        if viz:
            self.v = True
        else:
            self.v = False    
        if robot == 'Turtlebot3_sim':
            self.s_robot = 'Turtlebot3_sim'
            from src.config.AXMO_config_turtlebot import params
        if robot == 'Husky_sim':
            self.s_robot = 'Husky_sim'
            from src.config.AXMO_config_huksy import params 
        if robot == 'Turtlebot3_phy':
            self.s_robot = 'Turtlebot3_phy' 
            from src.config.AXMO_config_turtlebot import params            

    def measurement(self, 
                    controller_frequency,
                    planner_patience,
                    controller_patience,
                    conservative_reset_dist,
                    planner_frequency,
                    oscillation_timeout,
                    oscillation_distance,
                    acc_lim_theta,
                    acc_lim_trans,
                    acc_lim_x,
                    acc_lim_y,
                    angular_sim_granularity,
                    forward_point_distance,
                    goal_distance_bias,
                    max_scaling_factor,
                    max_vel_theta,
                    max_vel_trans,
                    max_vel_x,
                    max_vel_y,
                    min_vel_theta,
                    min_vel_trans,
                    min_vel_x,
                    min_vel_y,
                    occdist_scale,
                    oscillation_reset_angle,
                    oscillation_reset_dist,
                    path_distance_bias,
                    scaling_speed,
                    sim_granularity,
                    sim_time,
                    stop_time_buffer,
                    theta_stopped_vel,
                    trans_stopped_vel,
                    twirling_scale,
                    vth_samples,
                    vx_samples,
                    vy_samples,
                    xy_goal_tolerance,
                    yaw_goal_tolerance,
                    publish_frequency,
                    resolution,
                    transform_tolerance,
                    update_frequency,
                    cost_scaling_factor,
                    inflation_radius):

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

        MoveBase = {'controller_frequency':controller_frequency,
                    'planner_patience':planner_patience,
                    'controller_patience':controller_patience,
                    'conservative_reset_dist':conservative_reset_dist,
                    'planner_frequency':planner_frequency,
                    'oscillation_timeout':oscillation_timeout,
                    'oscillation_distance':oscillation_distance}

        DWAPlannerROS = {'acc_lim_theta':acc_lim_theta,
                        'acc_lim_trans':acc_lim_trans,
                        'acc_lim_x':acc_lim_x,
                        'acc_lim_y':acc_lim_y,
                        'angular_sim_granularity':angular_sim_granularity,
                        'forward_point_distance':forward_point_distance,
                        'goal_distance_bias':goal_distance_bias,
                        'max_scaling_factor':max_scaling_factor,
                        'max_vel_theta':max_vel_theta,
                        'max_vel_trans':max_vel_trans,
                        'max_vel_x':max_vel_x,
                        'max_vel_y':max_vel_y,
                        'min_vel_theta':min_vel_theta,
                        'min_vel_trans':min_vel_trans,
                        'min_vel_x':min_vel_x,
                        'min_vel_y':min_vel_y,
                        'occdist_scale':occdist_scale,
                        'oscillation_reset_angle':oscillation_reset_angle,
                        'oscillation_reset_dist':oscillation_reset_dist,
                        'path_distance_bias':path_distance_bias,
                        'scaling_speed':scaling_speed,
                        'sim_granularity':sim_granularity,
                        'sim_time':sim_time,
                        'stop_time_buffer':stop_time_buffer,
                        'theta_stopped_vel':theta_stopped_vel,
                        'trans_stopped_vel':trans_stopped_vel,
                        'twirling_scale':twirling_scale,
                        'vth_samples':vth_samples,
                        'vx_samples':vx_samples,
                        'vy_samples':vy_samples,
                        'xy_goal_tolerance':xy_goal_tolerance,
                        'yaw_goal_tolerance':yaw_goal_tolerance}

        costmap_common = {'publish_frequency':publish_frequency,
                        'resolution':resolution,
                        'transform_tolerance':transform_tolerance,
                        'update_frequency':update_frequency}
        
        costmap_common_inflation = {'cost_scaling_factor':cost_scaling_factor,
                                    'inflation_radius':inflation_radius}
        
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
            task_completion, safety, task_completion_rate, penalty)                           = TM.tbot_phy_get_measurement()              
        return (energy, mission_time, traveled_distance,
                planner_failed, euc_distance, recovery_executed,
                task_completion, safety, task_completion_rate, penalty)


    def evaluate(self, parameters, f1:str, f2:str, hv_ref_f1:float, hv_ref_f2:float):
        evaluation = self.measurement(
                        parameters.get("controller_frequency"), 
                        parameters.get("planner_patience"),
                        parameters.get("controller_patience"),
                        parameters.get("conservative_reset_dist"),
                        parameters.get("planner_frequency"),
                        parameters.get("oscillation_timeout"),
                        parameters.get("oscillation_distance"),
                        parameters.get("acc_lim_theta"),
                        parameters.get("acc_lim_trans"),
                        parameters.get("acc_lim_x"),
                        parameters.get("acc_lim_y"),
                        parameters.get("angular_sim_granularity"),
                        parameters.get("forward_point_distance"),
                        parameters.get("goal_distance_bias"),
                        parameters.get("max_scaling_factor"),
                        parameters.get("max_vel_theta"),
                        parameters.get("max_vel_trans"),
                        parameters.get("max_vel_x"),
                        parameters.get("max_vel_y"),
                        parameters.get("min_vel_theta"),
                        parameters.get("min_vel_trans"),
                        parameters.get("min_vel_x"),
                        parameters.get("min_vel_y"),
                        parameters.get("occdist_scale"),
                        parameters.get("oscillation_reset_angle"),
                        parameters.get("oscillation_reset_dist"),
                        parameters.get("path_distance_bias"),
                        parameters.get("scaling_speed"),
                        parameters.get("sim_granularity"),
                        parameters.get("sim_time"),
                        parameters.get("stop_time_buffer"),
                        parameters.get("theta_stopped_vel"),
                        parameters.get("trans_stopped_vel"),
                        parameters.get("twirling_scale"),
                        parameters.get("vth_samples"),
                        parameters.get("vx_samples"),
                        parameters.get("vy_samples"),
                        parameters.get("xy_goal_tolerance"),
                        parameters.get("yaw_goal_tolerance"),
                        parameters.get("publish_frequency"),
                        parameters.get("resolution"),
                        parameters.get("transform_tolerance"),
                        parameters.get("update_frequency"),
                        parameters.get("cost_scaling_factor"),
                        parameters.get("inflation_radius"),
        )

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
            print("[WARN]: HV computation may be inaccurate. Compute HV separately!")
            self.hv = (hyp.compute([hv_ref_f1, hv_ref_f2]) / np.prod([hv_ref_f1, hv_ref_f2]))
        except:
            self.hv = 0
            print(f"[STATUS]: Hypervolume reference violated! (logged HV {self.hv})")
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
                "Task_completion_rate": (evaluation[8]),
                "Penalty_track": (evaluation[9]),
                "Energy_track": (evaluation[0]),
                "Mission_time_track": (evaluation[1]),
                "Traveled_distance_track": (evaluation[2]),
                "Planner_failed_track": (evaluation[3]),
                "Positional_error_track": (evaluation[4]),
                "Recovery_executed_track": (evaluation[5]),
                }                                                                          
    
    def ax(self, robot:str, n_iter:int, f1:str, f2:str, f1_pref:float, 
           f2_pref:float, safety_constarint:str, task_completion_constraint:str, l_opt:bool, snap:int, 
           hv_ref_f1:float, hv_ref_f2:float, init_trails:int, verbose_logging:bool):           
        self.f1 = f1
        self.f2 = f2
        self.safety_constarint = safety_constarint
        if l_opt:
            ax_client = AxClient(verbose_logging=verbose_logging)
            ax_client = AxClient.load_from_json_file(filepath=f'model/optimization_snapshot/{robot}_ax_client_snapshot_{snap}.json')
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
                                       "Positional_error_track", "Recovery_executed_track"],                       
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

