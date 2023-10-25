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
    battery_collision = subprocess.check_call("./husky_battery_bumper.sh '%s'", cwd="src/benchmark/service", shell=True)
    set_config = subprocess.check_call("python sampling_interface_husky.py '%s'", cwd="src/benchmark/", shell=True)
    for i in tqdm(range(5),  desc="Set config method=random", colour=color, ascii=ASCII, bar_format='{l_bar}{bar:20}{r_bar}{bar:-20b}', leave=False): 
        time.sleep(1)

    rosbag = subprocess.check_call("gnome-terminal -- ./ros_record.sh '%s'", cwd="src/benchmark/service", shell=True)
    calculate_distance = subprocess.check_call("gnome-terminal -- python calculate_distance_traveled.py '%s'", cwd="src/benchmark", shell=True)
    for i in tqdm(range(5),  desc="Data logger", colour=color, ascii=ASCII, bar_format='{l_bar}{bar:20}{r_bar}{bar:-20b}', leave=False): 
        time.sleep(1)
    print("")
    obs_distance = subprocess.check_call("gnome-terminal -- python get_obstacleDistance.py '%s'", cwd="src/benchmark/", shell=True)
    reval.loader = Loader("Mission in progress...", bcolors.CGREEN + "" + bcolors.ENDC, 0.05).start()
    loading = True
    nav2d_goal = subprocess.check_call("python mission_husky.py '%s'", cwd="src/benchmark/", shell=True)
    reval.loader.stop() 
    loading = False
    print(bcolors.CGREEN + "Mission Done!" + bcolors.ENDC)
    for i in tqdm(range(10),  desc="Finishing simulation", colour=color, ascii=ASCII, bar_format='{l_bar}{bar:20}{r_bar}{bar:-20b}', leave=False):
        time.sleep(1)

    kill_rosnode = subprocess.check_call("./kill_rosnode_husky.sh &>/dev/null", cwd="src/benchmark/service", shell=True)

    for i in tqdm(range(5),  desc="Generating logs", colour=color, ascii=ASCII, bar_format='{l_bar}{bar:20}{r_bar}{bar:-20b}', leave=False):
        time.sleep(1)

    reval.loader = Loader("Evaluating logs...", bcolors.HEADER + "" + bcolors.ENDC, 0.05).start()
    loading = True    
    eval = subprocess.check_call("./eval.sh '%s'", cwd="src/benchmark/service", shell=True)
    reval.loader.stop()
    loading = False  
    evaluation_results = subprocess.check_call("python evaluation_results_husky.py '%s'", cwd="src/benchmark/", shell=True)  

    # path = "src/benchmark/log"
    # if os.path.isdir(path):
    #     shutil.rmtree(path)



if __name__== '__main__':
    try:
        signal.signal(signal.SIGINT, KeyboardInterrupt.signal_handler)
        cursor_off = subprocess.check_call("tput civis", shell=True)

        for i in tqdm(range(command.args.e), colour="green", desc="Epoch", bar_format='{l_bar}{bar:20}{r_bar}{bar:-20b}'):
            if command.args.v == 'On' or command.args.v == 'on':
                husky_gazebo = subprocess.check_call("./husky_gazebo.sh '%s'", cwd="src/benchmark/service", shell=True)
                for i in tqdm(range(8), desc="Launching husky_gazebo", colour=color, ascii=ASCII, bar_format='{l_bar}{bar:20}{r_bar}{bar:-20b}', leave=False):
                    time.sleep(1)
                husky_mb = subprocess.check_call("./husky_movebase.sh '%s'", cwd="src/benchmark/service", shell=True)
                for i in tqdm(range(3),  desc="Launching husky_MobeBase", colour=color, ascii=ASCII, bar_format='{l_bar}{bar:20}{r_bar}{bar:-20b}', leave=False): 
                    time.sleep(1)
                husky_rviz = subprocess.check_call("./husky_rviz.sh '%s'", cwd="src/benchmark/service", shell=True)
                for i in tqdm(range(10),  desc="Launching husky_rviz", colour=color, ascii=ASCII, bar_format='{l_bar}{bar:20}{r_bar}{bar:-20b}', leave=False): 
                    time.sleep(1)                    
                reval()    
                time.sleep(5)

            if command.args.v == 'Off' or command.args.v == 'off':
                husky_gazebo = subprocess.check_call("./husky_gazebo_nogui.sh '%s'", cwd="src/benchmark/service", shell=True)
                for i in tqdm(range(8), desc="Launching husky_gazebo", colour=color, ascii=ASCII, bar_format='{l_bar}{bar:20}{r_bar}{bar:-20b}', leave=False):
                    time.sleep(1)
                husky_mb = subprocess.check_call("./husky_movebase.sh '%s'", cwd="src/benchmark/service", shell=True)
                for i in tqdm(range(3),  desc="Launching husky_MobeBase", colour=color, ascii=ASCII, bar_format='{l_bar}{bar:20}{r_bar}{bar:-20b}', leave=False): 
                    time.sleep(1)                    
                reval()    
                time.sleep(5)   
        cursor_on = subprocess.check_call("tput cvvis", shell=True)


    except:
        if loading == True:
            reval.loader.stop()    
        print(bcolors.WARNING + "Killing rosnodes and roslaunch" + bcolors.ENDC)
        kill_rosnode = subprocess.check_call("./kill_rosnode_husky.sh '%s'", cwd="src/benchmark/service", shell=True)        
        for i in tqdm(range(5),  desc="Closing everything", colour=color, ascii=ASCII, bar_format='{l_bar}{bar:20}{r_bar}{bar:-20b}'):
            time.sleep(1)
        print(bcolors.FAIL + "Mission Failed!" + bcolors.ENDC)    
        # path = "src/benchmark/log"
        # if os.path.isdir(path):
        #     shutil.rmtree(path)
        cursor_on = subprocess.check_call("tput cvvis", shell=True)  