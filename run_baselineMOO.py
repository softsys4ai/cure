import signal
import sys
import pickle
import os
import subprocess
from src.baseline_MOO import AXMO
from src.utils import Argsparser_baselineMOO, KeyboardInterrupt, Exception
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
    p = Argsparser_baselineMOO()
    args, parser = p.get_args()
    robot = args.robot
    with open('src/Reval/src/benchmark/log/sc.ob', 'wb') as fp:
        pickle.dump(args.sc, fp)    
    viz = args.viz             
    MO = AXMO(robot, viz)
    if args.robot == "Turtlebot3_phy":
        print("[STATUS]: Launching navigation node")
        turtlebot_nav = subprocess.check_call("./turtlebot3_move_base_phy.sh '%s'", cwd="src/Reval/src/benchmark/service", shell=True)    
    print("[STATUS]: Performing multi-objective BO")
    signal.signal(signal.SIGINT, KeyboardInterrupt.signal_handler)
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
