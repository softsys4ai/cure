import sys
import os
import pickle
import signal
import pandas as pd
import numpy as np
from tabulate import tabulate
from src.baseline_featureSelection import FeatureSelection
from src.searchSpaceGen import SearchSpaceGen
from ax.plot.pareto_utils import compute_posterior_pareto_frontier, get_observed_pareto_frontiers
from ax.service.ax_client import AxClient
from ax.plot.pareto_frontier import plot_pareto_frontier
from ax.utils.notebook.plotting import render, init_notebook_plotting
from src.utils import Argsparser_baselineSF, KeyboardInterrupt, Exception
import warnings
warnings.filterwarnings('ignore')

if not os.path.exists('model'):
    os.makedirs('model')
if not os.path.exists('model/optimization_snapshot'):
    os.makedirs('model/optimization_snapshot')    
if not os.path.exists('cure_log'):
    os.makedirs('cure_log')  
if not os.path.exists('fig'):
    os.makedirs('fig')
if not os.path.exists('src/Reval/results'):
    os.makedirs('src/Reval/results')          
if not os.path.exists('src/Reval/src/benchmark/log'):
    os.makedirs('src/Reval/src/benchmark/log')

if __name__ == "__main__":
    p = Argsparser_baselineSF()
    args, parser = p.get_args()    
    robot = args.robot  
    viz = args.viz
    functional = args.f
    non_functional = args.nf
    try:
        user_interest = functional + non_functional
    except:
        parser.print_help()
        print("")
        sys.exit("[ERROR]: At least one non-functional property is required!")  

    if args.f1 == args.f2:    
        parser.print_help()
        print("")
        sys.exit("[ERROR]: f1 and f2 can not be same!")  
    try:
        data = pd.read_csv(args.data)
    except:
        parser.print_help()
        print("")
        sys.exit("[ERROR]: provide the path of the data!")             
    columns = data.columns
    config = np.array(data.iloc[:, :34]) # only the configs
    config_columns = columns[0:34].to_list()    

    FS = FeatureSelection()
    root_causes = []
    for name in range(len(user_interest)):
        print(f"[STATUS]: Diagnosing root-cause for {user_interest[name]}")
        root_causes.append(FS.get_rootCause(config_columns=config_columns,
                            config=config,
                            objective_name=user_interest[name],
                            objective=np.array(data[user_interest[name]]),
                            top_k=args.top_k))

    # Display root-causes
    table = []
    for target in range(len(root_causes)):
        k = list(root_causes[target].keys())
        v = (k) + list(root_causes[target].values())[0]
        table.append(v)
    print("")
    print("[STATUS]: Diagnosed root-causes!")
    print(tabulate((pd.DataFrame(table).T), tablefmt="simple", showindex=False)) 
    (pd.DataFrame(table).T).to_csv("cure_log/baseline_root_causes.csv", index=False, header=False)

    if args.opt:
        if args.l_opt:
            if not os.path.isfile(f'model/{args.robot}_ax_client_snapshot_{args.snap}.json'):
                parser.print_help()
                print("")
                sys.exit(f"[ERROR]: Saved optimization JSON (snapshot {args.snap}) not found!")  
            else:
                print("[STATUS]: Optimization snapshot restored!")                       
    # select config based on importance
        name = []
        for item in range(len(root_causes)):
            unzipped_name = list(list(zip(*list(root_causes[item].values())[0]))[0])
            unzipped_importance = list(list(zip(*list(root_causes[item].values())[0]))[1])
            name.append(unzipped_name)
        obj_union_met = list(set().union(*name))  
        print("[STATUS]: Selecting varibales based on importance")
        
        # Generate search space
        SG = SearchSpaceGen(robot)
        arameters, nodes, search_space = SG.get_configSpcae(obj_union_met)    

        from src.SFAXMO import SFAXMO
        MO = SFAXMO(robot, viz)
        
        signal.signal(signal.SIGINT, KeyboardInterrupt.signal_handler)
        # Perform optimization
        print("[STATUS]: Performing multi-objective BO")
        with open('src/Reval/src/benchmark/log/sc.ob', 'wb') as fp:
            pickle.dump(args.sc, fp)        
        AX_exp = MO.ax(n_iter=args.budget,
                    robot=args.robot,
                    f1=args.f1,
                    f2=args.f2,
                    f1_pref=args.f1_pref,
                    f2_pref=args.f2_pref,                
                    safety_constarint=f"Obstacle_distance >= {args.sc}",
                    task_completion_constraint=f"Task_completion_rate >= {args.tcr}",
                    hv_ref_f1=args.hv_ref_f1,
                    hv_ref_f2=args.hv_ref_f2,
                    init_trails=args.init_trails,
                    l_opt=args.l_opt,
                    snap=args.snap,
                    verbose_logging=args.verbose)   