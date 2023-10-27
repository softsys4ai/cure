import pandas as pd
import numpy as np
import pygmo as pg
from ax.service.ax_client import AxClient
from ax.plot.pareto_utils import compute_posterior_pareto_frontier, get_observed_pareto_frontiers
from ax.service.utils.report_utils import exp_to_df

class EvalMetrics():
    def read_data(self, data:str, obj_1:str, obj_2:str):
        df = pd.read_csv(data)
        objective_1 = df[f'{obj_1}']
        objective_2 = df[f'{obj_2}']
        safety = df['Obstacle_distance']
        task_success = df['Task success']
        hv = df['HV']
        n_iter = df['Iteration']
        rns = df['Task_success_rate']
        p_obj_1= []
        p_p_obj_2 = []
        p_niter = []
        for i in range(len(n_iter)):
            if rns[i] >= 0.8:
                p_obj_1.append(objective_1[i])
                p_p_obj_2.append(objective_2[i])
                p_niter.append(n_iter[i])
        pareto_df = pd.DataFrame({"obj_1": p_obj_1, "obj_2":p_p_obj_2, "n_iter":p_niter}) 
        pareto = ([sorted(pareto_df['obj_1'], reverse=False)], [sorted(pareto_df['obj_2'], reverse=True)], [p_niter])
        return (n_iter, objective_1, objective_2, safety, task_success, rns, hv, pareto)
        
    def hypervolume(self, data:str, f1:str, f2:str, f1_ref:float, f2_ref:float):
        df = pd.read_csv(data)
        f1 = df[f1]
        f2 = df[f2]   
        n_iter = df["Iteration"]
        hv = []
        for i in range(len(f1)):
            hyp = pg.hypervolume([[f1[i], f2[i]]])
            hv.append((hyp.compute([f1_ref, f2_ref]) / np.prod([f1_ref, f2_ref])))
        hypervolume = pd.DataFrame({"HV": hv}) 
        return n_iter, hypervolume

    def success_rate(self, task_success):
        buffer = []
        sucess_rate = []
        for i in range(len(task_success)):
            buffer.append(task_success[i])
            sucess_rate.append(sum(buffer)/(i+1))  
        return sucess_rate  

    def observed_pareto_front(self, json:str):
        ax_client = AxClient.load_from_json_file(filepath=json)
        arm_name = exp_to_df(ax_client.experiment)['arm_name']
        frontier = get_observed_pareto_frontiers(
            experiment=ax_client.experiment,
            data=ax_client.experiment.fetch_data(),
            rel=False,
        )
        return frontier 

    def posterior_pareto_front(self, json:str, obj_1:str, obj_2:str):
        ax_client = AxClient.load_from_json_file(filepath=json)
        objectives = ax_client.experiment.optimization_config.objective.objectives
        frontier = compute_posterior_pareto_frontier(
            experiment=ax_client.experiment,
            data=ax_client.experiment.fetch_data(),
            primary_objective=objectives[1].metric,
            secondary_objective=objectives[0].metric,
            absolute_metrics=[f"{obj_1}", f"{obj_2}"],
            outcome_constraints = ax_client.experiment.optimization_config.outcome_constraints,
            num_points=50,
        )   
        return  frontier
    