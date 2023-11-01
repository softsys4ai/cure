import sys
import os
import pickle
import signal
import subprocess
import pandas as pd
import numpy as np
from tabulate import tabulate
from src.baseline_featureSelection import FeatureSelection
from src.searchSpaceGen import SearchSpaceGen
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
    # Loading outlier data
    try:
        outlier_data = pd.read_csv(args.outlier_data)
    except:
        parser.print_help()
        print("")
        sys.exit("[ERROR]: Provide the path of the outlier data!")      
    outlier_columns = outlier_data.columns
    outlier_config = outlier_data.iloc[:, :34].values # only the configs
    outlier_config_columns = outlier_columns[0:34] 
    outlier_obj_columns = outlier_columns[34:42]

    FS = FeatureSelection()

    if args.l:
        # Diagonose root causes
        root_causes = []
        for name in range(len(user_interest)):
            print(f"[STATUS]: Diagnosing root-cause for {user_interest[name]}")
            root_causes.append(FS.get_rootCause(outlier_data=outlier_data,
                                            model_name=args.model,
                                            config_columns=outlier_config_columns,
                                            obj_columns=outlier_obj_columns,
                                            config=outlier_config,
                                            objective_name=user_interest[name],
                                            top_k=args.top_k))        
    else:    
        # Loading the train data
        try:
            train_data = pd.read_csv(args.train_data)
        except:
            parser.print_help()
            print("")
            sys.exit("[ERROR]: Provide the path of the train data!")           
        train_columns = train_data.columns
        train_config = train_data.iloc[:, :34].values # only the configs
        train_obj = train_data.iloc[:, 34:42].values
        train_config_columns = train_columns[0:34] 
        train_obj_columns = train_columns[34:42]        
        # Train the model
        FS.train(config=train_config, objectives=train_obj, model_name=f"model/RidgeCV_{args.robot}_model")
        # Diagonose root causes
        root_causes = []
        for name in range(len(user_interest)):
            print(f"[STATUS]: Diagnosing root-cause for {user_interest[name]}")
            root_causes.append(FS.get_rootCause(outlier_data=outlier_data,
                                            model_name=f"model/RidgeCV_{args.robot}_model",
                                            config_columns=outlier_config_columns,
                                            obj_columns=outlier_obj_columns,
                                            config=outlier_config,
                                            objective_name=user_interest[name],
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
        if args.robot == "Turtlebot3_phy":
            print("[STATUS]: Launching navigation node")
            turtlebot_nav = subprocess.check_call("./turtlebot3_move_base_phy.sh '%s'", cwd="src/Reval/src/benchmark/service", shell=True)        
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
                    json=args.json,
                    verbose_logging=args.verbose)   