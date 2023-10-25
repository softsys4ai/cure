import os
import shutil
import signal
import subprocess
import time
from turtle import bgcolor
from tqdm import tqdm
from utils.utils import bcolors, Loader, KeyboardInterrupt
from utils.arg_parse import command


if not os.path.exists('src/benchmark/log'):
    os.makedirs('src/benchmark/log')
if not os.path.exists('results'):
    os.makedirs('results')     

color = 'white'
ASCII = '.#'
loading = False

def reval():
    tbot_battery = subprocess.check_call("gnome-terminal -- python get_energy_phy_robot.py '%s'", cwd="src/benchmark", shell=True)
    rosbag = subprocess.check_call("gnome-terminal -- ./ros_record.sh '%s'", cwd="src/benchmark/service", shell=True)
    for i in tqdm(range(5),  desc="Data logger", colour=color, ascii=ASCII, bar_format='{l_bar}{bar:20}{r_bar}{bar:-20b}', leave=False): 
        time.sleep(1)
    calculate_distance = subprocess.check_call("gnome-terminal -- python calculate_distance_traveled.py '%s'", cwd="src/benchmark", shell=True)    
    get_obstacle_distance =  subprocess.check_call("gnome-terminal -- python get_obstacleDistance.py '%s'", cwd="src/benchmark", shell=True)       
    print("") 
    set_config = subprocess.check_call("python sampling_interface_turtlebot.py '%s'", cwd="src/benchmark/", shell=True)
    for i in tqdm(range(5),  desc="Set config method=random", colour=color, ascii=ASCII, bar_format='{l_bar}{bar:20}{r_bar}{bar:-20b}', leave=False): 
        time.sleep(1)
    reval.loader = Loader("Mission in progress...", bcolors.CGREEN + "" + bcolors.ENDC, 0.05).start()
    loading = True
    nav2d_goal = subprocess.check_call("python mission_turtlebot_phy.py '%s'", cwd="src/benchmark/", shell=True)
    reval.loader.stop() 
    loading = False
    print(bcolors.CGREEN + "Mission Done!" + bcolors.ENDC)
    for i in tqdm(range(10),  desc="Finishing execution", colour=color, ascii=ASCII, bar_format='{l_bar}{bar:20}{r_bar}{bar:-20b}', leave=False):
        time.sleep(1)

    kill_rosnode = subprocess.check_call("./kill_rosnode_turtlebot_phy.sh &>/dev/null", cwd="src/benchmark/service", shell=True)

    for i in tqdm(range(5),  desc="Generating logs", colour=color, ascii=ASCII, bar_format='{l_bar}{bar:20}{r_bar}{bar:-20b}', leave=False):
        time.sleep(1)

    reval.loader = Loader("Evaluating logs...", bcolors.HEADER + "" + bcolors.ENDC, 0.05).start()
    loading = True    
    eval = subprocess.check_call("./eval_turtlebot_phy.sh '%s'", cwd="src/benchmark/service", shell=True)
    reval.loader.stop()
    loading = False  
    evaluation_results = subprocess.check_call("python evaluation_results_turtlebot_phy.py '%s'", cwd="src/benchmark/", shell=True)  


if __name__== '__main__':
    try:
        signal.signal(signal.SIGINT, KeyboardInterrupt.signal_handler)
        cursor_off = subprocess.check_call("tput civis", shell=True)         
        for i in tqdm(range(command.args.e), colour="green", desc="Epoch", bar_format='{l_bar}{bar:20}{r_bar}{bar:-20b}'):        
            reval()
            # print("[STATUS]: Switched to default config. If not known, place  the robot to home manually!")       
            subprocess.check_call("rosrun dynamic_reconfigure dynparam load /move_base param/move_base_params.yaml", shell=True)
            subprocess.check_call("rosrun dynamic_reconfigure dynparam load /move_base/DWAPlannerROS param/dwa_local_planner_params_burger.yaml", shell=True)
            subprocess.check_call("rosrun dynamic_reconfigure dynparam load /move_base/global_costmap param/global_costmap_params.yaml", shell=True)
            subprocess.check_call("rosrun dynamic_reconfigure dynparam load /move_base/local_costmap param/local_costmap_params.yaml", shell=True)
            subprocess.check_call("rosrun dynamic_reconfigure dynparam load /move_base/global_costmap/inflation_layer param/costmap_inflation_params.yaml", shell=True)
            subprocess.check_call("rosrun dynamic_reconfigure dynparam load /move_base/local_costmap/inflation_layer param/costmap_inflation_params.yaml", shell=True)
            loader = Loader("Sending the robot to home...", bcolors.CGREEN + "" + bcolors.ENDC, 0.05).start() 
            send_home = subprocess.check_call("python send_home.py '%s'", cwd="src/benchmark/", shell=True)
            loader.stop()      
            time.sleep(5) 
        cursor_on = subprocess.check_call("tput cvvis", shell=True)


    except:
        if loading == True:
            reval.loader.stop()    
        print(bcolors.WARNING + "Killing rosnodes and roslaunch" + bcolors.ENDC)
        kill_rosnode = subprocess.check_call("./kill_rosnode_turtlebot_phy.sh '%s'", cwd="src/benchmark/service", shell=True)        
        for i in tqdm(range(5),  desc="Closing everything", colour=color, ascii=ASCII, bar_format='{l_bar}{bar:20}{r_bar}{bar:-20b}'):
            time.sleep(1)
        print(bcolors.FAIL + "Mission Failed!" + bcolors.ENDC)    
        cursor_on = subprocess.check_call("tput cvvis", shell=True)  