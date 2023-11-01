import sys
import argparse
from itertools import cycle
from shutil import get_terminal_size
from threading import Thread
from time import sleep

class Argsparser_care():
    def get_args(self):
        parser = argparse.ArgumentParser(description='causal-inference+MOO: debugging and fixing root-causes of functional and non-functional faults')
        parser.add_argument('--robot', 
                            required=True,
                            choices=["Husky_sim", "Turtlebot3_sim", "Turtlebot3_phy"],                    
                            type=str,                                      
                            help='robotic platform')       
        parser.add_argument('--train_data', 
                            metavar='',                  
                            type=str,                                      
                            help='path of the training data (.csv)') 
        parser.add_argument('-root_cause', 
                            action="store_true",                                     
                            help='diagnose root-causes')        
        # for dowhy
        parser.add_argument('--normal_data',
                            metavar='',           
                            type=str,                                      
                            help='path of the normal data (.csv)')
        parser.add_argument('--outlier_data',
                            metavar='',                  
                            type=str,                                      
                            help='path of the anomaly data (.csv)')                       
        parser.add_argument('--f', 
                            metavar='', 
                            choices=['Task_success_rate'],
                            default=['Task_success_rate'], 
                            type=str,
                            nargs='+',
                            action='store',                    
                            help='functional properties (fixed:  Task_success_rate)')
        parser.add_argument('--nf', 
                            metavar='', 
                            choices=['Energy', 'Mission_time', 'Traveled_distance', 
                                    'Positional_error', 'Planner_failed', 'Recovery_executed', 'Obstacle_distance'],
                            default=['Energy', 'Positional_error'], 
                            type=str,   
                            nargs='+',
                            action='store',                                     
                            help='non-functional properties (default:  [Energy, Positional_error])')
        # for dowhy
        parser.add_argument('--repeat',
                            metavar='', 
                            default=1, 
                            type=int,                                       
                            help='number of bootstrap resamples (default:  1)')
        parser.add_argument('--top_k',
                            metavar='', 
                            choices=range(1, 45),
                            default=5, 
                            type=int,                                       
                            help='number of root-causes based on average causal effect (default:  5)')
        parser.add_argument('-l', 
                            action="store_true",                                     
                            help='diagnose root-causes from the saved model [must be True to use --model]') 
        parser.add_argument('--model', 
                            metavar='',                              
                            help='saved model path (.model)')         
        parser.add_argument('-opt', 
                            action="store_true",                                     
                            help='perform optimization [must be True to use --budget, --f1, --f2, --f1_pref, --f2_pref, --sc]')     
        parser.add_argument('--budget', 
                            metavar='', 
                            default=1, 
                            type=int,                                      
                            help='number of evaluation to be performed (default:  10)')     
        parser.add_argument('--f1', 
                            metavar='', 
                            default="Energy", 
                            choices=['Energy', 'Mission_time', 'Traveled_distance', 
                                    'Positional_error', 'Planner_failed', 'Recovery_executed'],                    
                            type=str,                                      
                            help='objectiive 1 (default:  Energy)')
        parser.add_argument('--f2', 
                            metavar='', 
                            default="Positional_error", 
                            choices=['Energy', 'Mission_time', 'Traveled_distance', 
                                    'Positional_error', 'Planner_failed', 'Recovery_executed'],                    
                            type=str,                                      
                            help='objectiive 2 (default:  Positional_error)')           
        parser.add_argument('--f1_pref', 
                            metavar='', 
                            default=25.0, 
                            type=float,                                      
                            help='target f1 performance (default:  Energy<=25.0)')
        parser.add_argument('--f2_pref', 
                            metavar='', 
                            default=0.15, 
                            type=float,                                      
                            help='target f2 performance (default:  Positional_error<=0.1)')
        parser.add_argument('--sc', 
                            metavar='', 
                            default=0.25, 
                            type=float,                                      
                            help='safety constraint, maintained distance from obstacle (default:  Obstacle_distance>=0.25)')
        parser.add_argument('--tcr', 
                            metavar='', 
                            default=0.8, 
                            type=float,                                      
                            help='task completion rate (targets reached/total targets) constraint (default:  Task_completion_rate>=0.8)')        
        parser.add_argument('--hv_ref_f1', 
                            metavar='', 
                            type=float,                                      
                            help='hypervolum reference for f1')    
        parser.add_argument('--hv_ref_f2', 
                            metavar='', 
                            type=float,                                      
                            help='hypervolum reference for f2')      
        parser.add_argument('--init_trails', 
                            metavar='', 
                            default=68,
                            type=int,                                      
                            help='number of initial trails before fitting GP')              
        parser.add_argument('-viz',
                            action="store_true",                                      
                            help='show gazebo and rviz')   
        parser.add_argument('-l_opt',
                            action="store_true",                                      
                            help='reload optimization from JSON')   
        parser.add_argument('--json', 
                            metavar='',                                    
                            help='optmization model to load from')                    
        parser.add_argument('-v', '--verbose',
                            action="store_true",                                      
                            help='show statements during learning, effect estimation, and optimization')
        args = parser.parse_args()  
        return args, parser  


class Argsparser_baselineSF():
    def get_args(self):
        parser = argparse.ArgumentParser(description='RidgeCV+MOO: debugging and fixing root-causes of functional and non-functional faults')
        parser.add_argument('--robot', 
                            required=True,
                            choices=["Husky_sim", "Turtlebot3_sim", "Turtlebot3_phy"],                    
                            type=str,                                      
                            help='robotic platform')         
        parser.add_argument('--train_data', 
                            metavar='',                  
                            type=str,                                      
                            help='path of the training data (.csv)')  
        parser.add_argument('--outlier_data', 
                            required=True,                 
                            type=str,                                      
                            help='path of the data (.csv)')               
        parser.add_argument('--f', 
                            metavar='', 
                            choices=['Task_success_rate'],
                            default=['Task_success_rate'], 
                            type=str,
                            nargs='+',
                            action='store',                    
                            help='functional properties (fixed:  Task_success_rate)')
        parser.add_argument('--nf', 
                            metavar='', 
                            choices=['Energy', 'Mission_time', 'Traveled_distance', 
                                    'Positional_error', 'Planner_failed', 'Recovery_executed', 'Obstacle_distance'],
                            default=['Energy', 'Positional_error'], 
                            type=str,   
                            nargs='+',
                            action='store',                                     
                            help='non-functional properties (default:  [Energy, Positional_error])')
        parser.add_argument('--top_k',
                            metavar='', 
                            choices=range(1, 45),
                            default=5, 
                            type=int,                                       
                            help='number of root-causes based on Ridge coeffecients (default:  5)')
        parser.add_argument('-l', 
                            action="store_true",                                     
                            help='diagnose root-causes from the saved model [must be True to use --model]') 
        parser.add_argument('--model', 
                            metavar='',                              
                            help='saved model path (.joblib)')        
        parser.add_argument('-opt', 
                            action="store_true",                                     
                            help='perform optimization [must be True to use --budget, --f1, --f2, --f1_pref, --f2_pref, --sc]')     
        parser.add_argument('--budget', 
                            metavar='', 
                            default=1, 
                            type=int,                                      
                            help='number of evaluation to be performed (default:  10)')     
        parser.add_argument('--f1', 
                            metavar='', 
                            default="Energy", 
                            choices=['Energy', 'Mission_time', 'Traveled_distance', 
                                    'Positional_error', 'Planner_failed', 'Recovery_executed'],                    
                            type=str,                                      
                            help='objectiive 1 (default:  Energy)')
        parser.add_argument('--f2', 
                            metavar='', 
                            default="Positional_error", 
                            choices=['Energy', 'Mission_time', 'Traveled_distance', 
                                    'Positional_error', 'Planner_failed', 'Recovery_executed'],                    
                            type=str,                                      
                            help='objectiive 2 (default:  Positional_error)')           
        parser.add_argument('--f1_pref', 
                            metavar='', 
                            default=25.0, 
                            type=float,                                      
                            help='target f1 performance (default:  Energy<=25.0)')
        parser.add_argument('--f2_pref', 
                            metavar='', 
                            default=0.15, 
                            type=float,                                      
                            help='target f2 performance (default:  Positional_error<=0.1)')
        parser.add_argument('--sc', 
                            metavar='', 
                            default=0.25, 
                            type=float,                                      
                            help='safety constraint, maintained distance from obstacle (default:  Obstacle_distance>=0.25)')
        parser.add_argument('--tcr', 
                            metavar='', 
                            default=0.8, 
                            type=float,                                      
                            help='task completion rate (targets reached/total targets) constraint (default:  Task_completion_rate>=0.8)')         
        parser.add_argument('--hv_ref_f1', 
                            metavar='', 
                            type=float,                                      
                            help='hypervolum reference for f1')    
        parser.add_argument('--hv_ref_f2', 
                            metavar='', 
                            type=float,                                      
                            help='hypervolum reference for f2') 
        parser.add_argument('--init_trails', 
                            metavar='', 
                            default=68,
                            type=int,                                      
                            help='number of initial trails before fitting GP')                   
        parser.add_argument('-viz',
                            action="store_true",                                      
                            help='show gazebo and rviz') 
        parser.add_argument('-l_opt',
                            action="store_true",                                      
                            help='reload optimization from JSON') 
        parser.add_argument('--json', 
                            metavar='',                                      
                            help='optmization model to load from')                        
        parser.add_argument('-v', '--verbose',
                            action="store_true",                                      
                            help='show statements during optimization')
        args = parser.parse_args() 
        return args, parser       

class Argsparser_baselineMOO():
    def get_args(self):
        parser = argparse.ArgumentParser(description='MOO: multi-objective Bayesian optimization for fixing functional and non-functional faults')    
        parser.add_argument('--robot', 
                            required=True,
                            choices=["Husky_sim", "Turtlebot3_sim", "Turtlebot3_phy"],                    
                            type=str,                                      
                            help='robotic platform')        
        parser.add_argument('--budget', 
                            metavar='', 
                            default=1, 
                            type=int,                                      
                            help='number of evaluation to be performed (default:  1)')     
        parser.add_argument('--f1', 
                            metavar='', 
                            default="Energy", 
                            choices=['Energy', 'Mission_time', 'Traveled_distance', 
                                    'Positional_error', 'Planner_failed', 'Recovery_executed'],                    
                            type=str,                                      
                            help='objectiive 1 (default:  Energy)')
        parser.add_argument('--f2', 
                            metavar='', 
                            default="Positional_error", 
                            choices=['Energy', 'Mission_time', 'Traveled_distance', 
                                    'Positional_error', 'Planner_failed', 'Recovery_executed'],                    
                            type=str,                                      
                            help='objectiive 2 (default:  Positional_error)')           
        parser.add_argument('--f1_pref', 
                            metavar='', 
                            default=25.0, 
                            type=float,                                      
                            help='target f1 performance (default:  Energy<=25.0)')
        parser.add_argument('--f2_pref', 
                            metavar='', 
                            default=0.15, 
                            type=float,                                      
                            help='target f2 performance (default:  Positional_error<=0.1)')
        parser.add_argument('--sc', 
                            metavar='', 
                            default=0.25, 
                            type=float,                                      
                            help='safety constraint, maintained distance from obstacle (default:  Obstacle_distance>=0.25)')
        parser.add_argument('--tcr', 
                            metavar='', 
                            default=0.8, 
                            type=float,                                      
                            help='task completion rate (targets reached/total targets) constraint (default:  Task_completion_rate>=0.8)')         
        parser.add_argument('--hv_ref_f1', 
                            metavar='', 
                            type=float,                                      
                            help='hypervolum reference for f1')    
        parser.add_argument('--hv_ref_f2', 
                            metavar='', 
                            type=float,                                      
                            help='hypervolum reference for f2')    
        parser.add_argument('--init_trails', 
                            metavar='', 
                            default=68,
                            type=int,                                      
                            help='number of initial trails before fitting GP')                   
        parser.add_argument('-viz',
                            action="store_true",                                      
                            help='show gazebo and rviz')
        parser.add_argument('-l_opt',
                            action="store_true",                                      
                            help='reload optimization from JSON') 
        parser.add_argument('--json', 
                            metavar='',                                    
                            help='optmization model to load from')               
        parser.add_argument('-v', '--verbose',
                            action="store_true",                                      
                            help='show statements during optimization')
        args = parser.parse_args() 
        return args, parser        


class bcolors:
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKCYAN = '\033[96m'
    OKGREEN = '\033[92m'
    CGREEN  = '\33[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'   

class KeyboardInterrupt:
    def signal_handler(sig, frame):
        print("")
        print(bcolors.WARNING + 'You pressed Ctrl+C!' + bcolors.ENDC)
        sys.exit(0)  

"""
The Loader class was developed by JS Lavertu
We thank JS Lavertu and stackoverflow
"""
class Loader:
    def __init__(self, desc="Loading...", end="Done!", timeout=0.1):
        """
        A loader-like context manager

        Args:
            desc (str, optional): The loader's description. Defaults to "Loading...".
            end (str, optional): Final print. Defaults to "Done!".
            timeout (float, optional): Sleep time between prints. Defaults to 0.1.
        """
        self.desc = desc
        self.end = end
        self.timeout = timeout

        self._thread = Thread(target=self._animate, daemon=True)
        self.steps = ["⢿", "⣻", "⣽", "⣾", "⣷", "⣯", "⣟", "⡿"]
        self.done = False

    def start(self):
        self._thread.start()
        return self

    def _animate(self):
        for c in cycle(self.steps):
            if self.done:
                break
            print(f"\r{self.desc} {c}", flush=True, end="")
            sleep(self.timeout)

    def __enter__(self):
        self.start()

    def stop(self):
        self.done = True
        cols = get_terminal_size((80, 20)).columns
        print("\r" + " " * cols, end="", flush=True)
        print(f"\r{self.end}", flush=True)

    def __exit__(self, exc_type, exc_value, tb):
        # handle exceptions with those variables ^
        self.stop()

class Exception():
    def Except(self):
        import subprocess
        from tqdm import tqdm
        import time

        color = 'white'
        ASCII = '.#'
        print(bcolors.WARNING + "Killing rosnodes and roslaunch" + bcolors.ENDC)
        kill_rosnode = subprocess.check_call("./kill_rosnode_turtlebot.sh '%s'", cwd="src/Reval/src/benchmark/service", shell=True)        
        for i in tqdm(range(5),  desc="Closing everything", colour=color, ascii=ASCII, bar_format='{l_bar}{bar:20}{r_bar}{bar:-20b}'):
            time.sleep(1)
        print(bcolors.FAIL + "Execution Failed!" + bcolors.ENDC)             