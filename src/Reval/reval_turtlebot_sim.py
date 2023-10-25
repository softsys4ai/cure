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
    battery = subprocess.check_call("gnome-terminal -- python battery_consumer.py '%s'", cwd="src/reval_turtlebot3/src", shell=True)
    set_config = subprocess.check_call("python sampling_interface_turtlebot.py '%s'", cwd="src/benchmark/", shell=True)
    for i in tqdm(range(5),  desc="Set config method=random", colour=color, ascii=ASCII, bar_format='{l_bar}{bar:20}{r_bar}{bar:-20b}', leave=False): 
        time.sleep(1)

    rosbag = subprocess.check_call("gnome-terminal -- ./ros_record.sh '%s'", cwd="src/benchmark/service", shell=True)
    calculate_distance = subprocess.check_call("gnome-terminal -- python calculate_distance_traveled.py '%s'", cwd="src/benchmark", shell=True)
    for i in tqdm(range(5),  desc="Data logger", colour=color, ascii=ASCII, bar_format='{l_bar}{bar:20}{r_bar}{bar:-20b}', leave=False): 
        time.sleep(1)
    get_obstacle_distance =  subprocess.check_call("gnome-terminal -- python get_obstacleDistance.py '%s'", cwd="src/benchmark", shell=True)       
    print("")
    reval.loader = Loader("Mission in progress...", bcolors.CGREEN + "" + bcolors.ENDC, 0.05).start()
    loading = True
    nav2d_goal = subprocess.check_call("python mission_turtlebot.py '%s'", cwd="src/benchmark/", shell=True)
    reval.loader.stop() 
    loading = False
    print(bcolors.CGREEN + "Mission Done!" + bcolors.ENDC)
    for i in tqdm(range(10),  desc="Finishing simulation", colour=color, ascii=ASCII, bar_format='{l_bar}{bar:20}{r_bar}{bar:-20b}', leave=False):
        time.sleep(1)

    kill_rosnode = subprocess.check_call("./kill_rosnode_turtlebot.sh &>/dev/null", cwd="src/benchmark/service", shell=True)

    for i in tqdm(range(5),  desc="Generating logs", colour=color, ascii=ASCII, bar_format='{l_bar}{bar:20}{r_bar}{bar:-20b}', leave=False):
        time.sleep(1)

    reval.loader = Loader("Evaluating logs...", bcolors.HEADER + "" + bcolors.ENDC, 0.05).start()
    loading = True    
    eval = subprocess.check_call("./eval.sh '%s'", cwd="src/benchmark/service", shell=True)
    reval.loader.stop()
    loading = False  
    evaluation_results = subprocess.check_call("python evaluation_results_turtlebot.py '%s'", cwd="src/benchmark/", shell=True)  


if __name__== '__main__':
    try:
        signal.signal(signal.SIGINT, KeyboardInterrupt.signal_handler)
        cursor_off = subprocess.check_call("tput civis", shell=True)

        for i in tqdm(range(command.args.e), colour="green", desc="Epoch", bar_format='{l_bar}{bar:20}{r_bar}{bar:-20b}'):
            if command.args.v == 'off' or command.args.v == 'Off':
                turtlebot_gazebo = subprocess.check_call("./turtlebot3_gazebo_nogui.sh '%s'", cwd="src/benchmark/service", shell=True)
                for i in tqdm(range(8), desc="Launching turtlebot3_gazebo", colour=color, ascii=ASCII, bar_format='{l_bar}{bar:20}{r_bar}{bar:-20b}', leave=False):
                    time.sleep(1)   
                turtlebot_nav = subprocess.check_call("./turtlebot3_move_base_nogui.sh '%s'", cwd="src/benchmark/service", shell=True)
                for i in tqdm(range(8), desc="Launching turtlebot3_gazebo", colour=color, ascii=ASCII, bar_format='{l_bar}{bar:20}{r_bar}{bar:-20b}', leave=False):
                    time.sleep(1)
            if command.args.v == 'on' or command.args.v == 'On':
                turtlebot_gazebo = subprocess.check_call("./turtlebot3_gazebo.sh '%s'", cwd="src/benchmark/service", shell=True)
                for i in tqdm(range(8), desc="Launching turtlebot3_gazebo", colour=color, ascii=ASCII, bar_format='{l_bar}{bar:20}{r_bar}{bar:-20b}', leave=False):
                    time.sleep(1)
                turtlebot_nav = subprocess.check_call("./turtlebot3_move_base.sh '%s'", cwd="src/benchmark/service", shell=True)
                for i in tqdm(range(3),  desc="Launching turtlebot3_MobeBase", colour=color, ascii=ASCII, bar_format='{l_bar}{bar:20}{r_bar}{bar:-20b}', leave=False): 
                    time.sleep(1)           
            reval()
            time.sleep(5) 
        cursor_on = subprocess.check_call("tput cvvis", shell=True)


    except:
        if loading == True:
            reval.loader.stop()    
        print(bcolors.WARNING + "Killing rosnodes and roslaunch" + bcolors.ENDC)
        kill_rosnode = subprocess.check_call("./kill_rosnode_turtlebot.sh '%s'", cwd="src/benchmark/service", shell=True)        
        for i in tqdm(range(5),  desc="Closing everything", colour=color, ascii=ASCII, bar_format='{l_bar}{bar:20}{r_bar}{bar:-20b}'):
            time.sleep(1)
        print(bcolors.FAIL + "Mission Failed!" + bcolors.ENDC)    
        cursor_on = subprocess.check_call("tput cvvis", shell=True)  