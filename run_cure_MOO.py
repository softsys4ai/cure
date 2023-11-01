import os
import signal
import sys
import pickle
import subprocess
import pandas as pd
import networkx as nx
from tabulate import tabulate
from src.causal_model import CausalModel
from ananke.graphs import ADMG
from src.searchSpaceGen import SearchSpaceGen
from src.utils import Argsparser_care, Loader, KeyboardInterrupt, Exception

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

def run_care(rcausal, df, tabu_edges_remove, columns, objectives, alpha, verbose):
        fci_edges = rcausal.care_fci(df, tabu_edges_remove, alpha, verbose)
        edges = []
        # resolve notears_edges and fci_edges and update
        di_edges, bi_edges = rcausal.resolve_edges(edges, fci_edges, columns,
                                            tabu_edges_remove, objectives)
        G = ADMG(columns, di_edges=di_edges, bi_edges=bi_edges)
        return G, di_edges, bi_edges

if __name__ == '__main__':
    p = Argsparser_care()
    args, parser = p.get_args()
    l = False    
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

    alpha = 0.2     # use 0.2
    verbose = args.verbose # print statements    

    if args.l:
        try:
            # Loading the svaed causla model
            with open(args.model, 'rb') as fp:
                care = pickle.load(fp)      
            vertices = care[0]
            di_edges = care[1]
            bi_edges = care[2]
            CM = CausalModel(vertices)
            G = ADMG(vertices, di_edges=di_edges, bi_edges=bi_edges) 
            print("[STATUS]: Model loaded!")
        except:
            parser.print_help()
            print("")
            sys.exit("[ERROR]: No saved model found! Tarin the model")               

    else:      
        # nodes for causal graph
        try:
            df = pd.read_csv(args.train_data)
        except:
            parser.print_help()
            print("")
            sys.exit("[ERROR]: provide the path of the data!")             
        columns = df.columns
        options = df.iloc[:, 0:34]
        metrics = df.iloc[:, 34:40]  
        objectives = df.iloc[:, 40:42] 
        # initialize causal model object
        CM = CausalModel(columns)      
        # Learning the causal model
        tabu_edges_remove = CM.get_tabu_edges_care_remove(options=options, 
                                                          metrics=metrics,
                                                          objectives=objectives)
        G, di_edges, bi_edges = run_care(CM, 
                                         df=df, 
                                         tabu_edges_remove=tabu_edges_remove,
                                         columns=columns, 
                                         objectives=objectives, 
                                         alpha=alpha, 
                                         verbose=verbose)
        # saving the model
        care = [columns, di_edges, bi_edges]        
        with open(f'model/care_{robot}.model', 'wb') as fp:
            pickle.dump(care, fp)
        print(f"[STATUS]: Model saved to /model/care_{robot}.model!")    

    if args.root_cause:
        try:
            outlier_data = pd.read_csv(args.outlier_data)
        except:
            parser.print_help()
            print("")
            sys.exit("[ERROR]: provide the path of the outlier data!") 
        root_causes = []
        for name in range(len(user_interest)):
            if not args.verbose:
                l = True
                loader = Loader(f"[STATUS]: Diagnosing root-cause for {user_interest[name]}", 
                                f"[STATUS]: Computed causal effects for {user_interest[name]}!", 0.05).start()                    
            # Compute causal effect
            outlier_config = outlier_data.iloc[:, 0:34]
            root_causes.append(CM.CausalEffect(G=G,
                            data=outlier_data,
                            config_col=outlier_config,
                            target=str(user_interest[name]),
                            n_bootstraps=5,
                            alpha=0.05,
                            top_k=args.top_k,
                            verbose=args.verbose))
            if l == True:
                loader.stop()        
                        
        # Display root-causes
        table = []
        for target in range(len(root_causes)):
            k = list(root_causes[target].keys())
            v = (k) + list(root_causes[target].values())[0]
            table.append(v)
        print("")
        print("[STATUS]: Diagnosed root-causes!")
        print(tabulate((pd.DataFrame(table).T), tablefmt="simple", showindex=False)) 
        (pd.DataFrame(table).T).to_csv(f"cure_log/care_root_causes_{args.robot}.csv", index=False, header=False)

    if args.opt:
        # select config based on causal effect
        name = []
        for item in range(len(root_causes)):
            unzipped_name = list(list(zip(*list(root_causes[item].values())[0]))[0])
            unzipped_importance = list(list(zip(*list(root_causes[item].values())[0]))[1])
            name.append(unzipped_name)
        obj_union_met = list(set().union(*name))    
        print("[STATUS]: Selecting varibales based on causal effects")

        # Generate search space
        SG = SearchSpaceGen(robot)
        arameters, nodes, search_space = SG.get_configSpcae(obj_union_met)                     
        if args.robot == "Turtlebot3_phy":
            print("[STATUS]: Launching navigation node")
            turtlebot_nav = subprocess.check_call("./turtlebot3_move_base_phy.sh '%s'", cwd="src/Reval/src/benchmark/service", shell=True)  
        print("[STATUS]: Performing multi-objective BO")
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
                    json=args.json,
                    verbose_logging=args.verbose)  