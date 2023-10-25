import subprocess
import time
import pandas as pd
import dynamic_reconfigure.client
import rospy
from tqdm import tqdm
from src.Reval.utils.utils import bcolors, Loader, KeyboardInterrupt

color = 'white'
ASCII = '.#'

class TurtlebotSimMeasurement():
    def __init__(self):
        print("[STATUS]: Initialing TurtlebotSimMeasurement class using Reval API")
        rospy.init_node('update_config') 

    def launch_turtlebot3_sim(self, viz:bool):
        if viz:
            turtlebot_gazebo = subprocess.check_call("./turtlebot3_gazebo.sh '%s'", cwd="src/Reval/src/benchmark/service", shell=True)
            for i in tqdm(range(8), desc="Launching turtlebot3_gazebo", colour=color, ascii=ASCII, bar_format='{l_bar}{bar:20}{r_bar}{bar:-20b}', leave=False):
                time.sleep(1)
            turtlebot_nav = subprocess.check_call("./turtlebot3_move_base.sh '%s'", cwd="src/Reval/src/benchmark/service", shell=True)
            for i in tqdm(range(3),  desc="Launching turtlebot3_MobeBase", colour=color, ascii=ASCII, bar_format='{l_bar}{bar:20}{r_bar}{bar:-20b}', leave=False): 
                time.sleep(1)               
        else:         
            turtlebot_gazebo = subprocess.check_call("./turtlebot3_gazebo_nogui.sh '%s'", cwd="src/Reval/src/benchmark/service", shell=True)
            for i in tqdm(range(8), desc="Launching turtlebot3_gazebo", colour=color, ascii=ASCII, bar_format='{l_bar}{bar:20}{r_bar}{bar:-20b}', leave=False):
                time.sleep(1)
            turtlebot_nav = subprocess.check_call("./turtlebot3_move_base_nogui.sh '%s'", cwd="src/Reval/src/benchmark/service", shell=True)
            for i in tqdm(range(3),  desc="Launching turtlebot3_MobeBase", colour=color, ascii=ASCII, bar_format='{l_bar}{bar:20}{r_bar}{bar:-20b}', leave=False): 
                time.sleep(1)        
        battery = subprocess.check_call("gnome-terminal -- python battery_consumer.py '%s'", cwd="src/Reval/src/reval_turtlebot3/src", shell=True)
        rosbag = subprocess.check_call("gnome-terminal -- ./ros_record.sh '%s'", cwd="src/Reval/src/benchmark/service", shell=True)
        calculate_distance = subprocess.check_call("gnome-terminal -- python calculate_distance_traveled.py '%s'", cwd="src/Reval/src/benchmark", shell=True)
        for i in tqdm(range(5),  desc="Data logger", colour=color, ascii=ASCII, bar_format='{l_bar}{bar:20}{r_bar}{bar:-20b}', leave=False): 
            time.sleep(1)
        get_obstacle_distance =  subprocess.check_call("gnome-terminal -- python get_obstacleDistance.py '%s'", cwd="src/Reval/src/benchmark", shell=True)       
        print("")                       
        for i in tqdm(range(5),  desc="Updating config", colour=color, ascii=ASCII, bar_format='{l_bar}{bar:20}{r_bar}{bar:-20b}', leave=False): 
            time.sleep(1) 

    def launch_turtlebot3_task(self):
        loader = Loader("Mission in progress...", bcolors.CGREEN + "" + bcolors.ENDC, 0.05).start()    
        nav2d_goal = subprocess.check_call("python mission_turtlebot.py '%s'", cwd="src/Reval/src/benchmark/", shell=True)    
        loader.stop()          
        kill_rosnode = subprocess.check_call("./kill_rosnode_turtlebot.sh &>/dev/null", cwd="src/Reval/src/benchmark/service", shell=True)
        for i in tqdm(range(5),  desc="Generating logs", colour=color, ascii=ASCII, bar_format='{l_bar}{bar:20}{r_bar}{bar:-20b}', leave=False):
            time.sleep(1) 
        loader = Loader("Evaluating logs...", bcolors.HEADER + "" + bcolors.ENDC, 0.05).start()    
        eval = subprocess.check_call("./eval.sh '%s'", cwd="src/Reval/src/benchmark/service", shell=True)      
        loader.stop()  
        evaluation_results = subprocess.check_call("python MOO_measurement_turtlebot.py '%s'", cwd="src/Reval/src/benchmark/", shell=True)
        time.sleep(5)

    def tbot_dynamic_reconfig(self, MoveBase, DWAPlannerROS, costmap_common, costmap_common_inflation):
        move_base = dynamic_reconfigure.client.Client('/move_base')            
        move_base_DWAPlanner = dynamic_reconfigure.client.Client('/move_base/DWAPlannerROS')
        move_base_global_costmap_inflation = dynamic_reconfigure.client.Client('/move_base/global_costmap/inflation_layer')
        move_base_global_costmap = dynamic_reconfigure.client.Client('/move_base/global_costmap')
        move_base_local_costmap_inflation = dynamic_reconfigure.client.Client('/move_base/local_costmap/inflation_layer')
        move_base_local_costmap = dynamic_reconfigure.client.Client('/move_base/local_costmap')

        move_base.update_configuration(MoveBase)
        move_base_DWAPlanner.update_configuration(DWAPlannerROS)
        move_base_global_costmap.update_configuration(costmap_common)
        move_base_global_costmap_inflation.update_configuration(costmap_common_inflation)
        move_base_local_costmap.update_configuration(costmap_common)
        move_base_local_costmap_inflation.update_configuration(costmap_common_inflation) 

    def tbot_get_measurement(self):
        measurements = pd.read_csv("cure_log/MOBO_measurement_Turtlebot3_sim.csv")
        battery_percentage = measurements['Energy']
        mission_time = measurements['Mission_time']
        traveled_distance = measurements['Traveled_distance']
        planner_failed = measurements['Planner_failed']
        euc_distance = measurements['Positional_error']
        recovery_executed = measurements['Recovery_executed']
        misson_success = measurements['Mission_success']
        obs_distance = measurements['Obstacle_distance']
        task_completion_rate = measurements['RNS']
        pentalty = measurements['Penalty']
        energy = round(19.98 - (19.98 * (battery_percentage/100)), 2)
   
        return (float(energy), float(mission_time), 
                float(traveled_distance), float(planner_failed), 
                float(euc_distance), float(recovery_executed),
                int(misson_success), float(obs_distance), float(task_completion_rate),
                float(pentalty))            
    


class TurtlebotPhyMeasurement():
    def __init__(self):
        print("[STATUS]: Initialing TurtlebotPhyMeasurement class using Reval API")
        rospy.init_node('update_config') 

    def launch_turtlebot3_phy(self, viz:bool):
        # if viz:
        #     turtlebot_nav = subprocess.check_call("./turtlebot3_move_base_phy.sh '%s'", cwd="src/Reval/src/benchmark/service", shell=True)
        #     for i in tqdm(range(3),  desc="Launching turtlebot3_MobeBase", colour=color, ascii=ASCII, bar_format='{l_bar}{bar:20}{r_bar}{bar:-20b}', leave=False): 
        #         time.sleep(1)               
        # else:         
        #     turtlebot_nav = subprocess.check_call("./turtlebot3_move_base_nogui.sh '%s'", cwd="src/Reval/src/benchmark/service", shell=True)
        #     for i in tqdm(range(3),  desc="Launching turtlebot3_MobeBase", colour=color, ascii=ASCII, bar_format='{l_bar}{bar:20}{r_bar}{bar:-20b}', leave=False): 
        #         time.sleep(1)      
        tbot_battery = subprocess.check_call("gnome-terminal -- python get_energy_phy_robot.py '%s'", cwd="src/Reval/src/benchmark", shell=True)
        rosbag = subprocess.check_call("gnome-terminal -- ./ros_record.sh '%s'", cwd="src/Reval/src/benchmark/service", shell=True)
        calculate_distance = subprocess.check_call("gnome-terminal -- python calculate_distance_traveled.py '%s'", cwd="src/Reval/src/benchmark", shell=True)
        for i in tqdm(range(5),  desc="Data logger", colour=color, ascii=ASCII, bar_format='{l_bar}{bar:20}{r_bar}{bar:-20b}', leave=False): 
            time.sleep(1)
        get_obstacle_distance =  subprocess.check_call("gnome-terminal -- python get_obstacleDistance.py '%s'", cwd="src/Reval/src/benchmark", shell=True)       
        print("")                       
        for i in tqdm(range(5),  desc="Updating config", colour=color, ascii=ASCII, bar_format='{l_bar}{bar:20}{r_bar}{bar:-20b}', leave=False): 
            time.sleep(1) 

    def launch_turtlebot3_phy_task(self):
        loader = Loader("Mission in progress...", bcolors.CGREEN + "" + bcolors.ENDC, 0.05).start()    
        nav2d_goal = subprocess.check_call("python mission_turtlebot_phy.py '%s'", cwd="src/Reval/src/benchmark/", shell=True)    
        loader.stop()          
        kill_rosnode = subprocess.check_call("./kill_rosnode_turtlebot_phy.sh &>/dev/null", cwd="src/Reval/src/benchmark/service", shell=True)
        for i in tqdm(range(5),  desc="Generating logs", colour=color, ascii=ASCII, bar_format='{l_bar}{bar:20}{r_bar}{bar:-20b}', leave=False):
            time.sleep(1) 
        loader = Loader("Evaluating logs...", bcolors.HEADER + "" + bcolors.ENDC, 0.05).start()    
        eval = subprocess.check_call("./eval_turtlebot_phy.sh '%s'", cwd="src/Reval/src/benchmark/service", shell=True)      
        loader.stop()  
        evaluation_results = subprocess.check_call("python MOO_measurement_turtlebot_phy.py '%s'", cwd="src/Reval/src/benchmark/", shell=True)
        print("[STATUS]: Switched to default config. If not known, place  the robot to home manually!")       
        subprocess.check_call("rosrun dynamic_reconfigure dynparam load /move_base src/Reval/param/move_base_params.yaml", shell=True)
        subprocess.check_call("rosrun dynamic_reconfigure dynparam load /move_base/DWAPlannerROS src/Reval/param/dwa_local_planner_params_burger.yaml", shell=True)
        subprocess.check_call("rosrun dynamic_reconfigure dynparam load /move_base/global_costmap src/Reval/param/global_costmap_params.yaml", shell=True)
        subprocess.check_call("rosrun dynamic_reconfigure dynparam load /move_base/local_costmap src/Reval/param/local_costmap_params.yaml", shell=True)
        subprocess.check_call("rosrun dynamic_reconfigure dynparam load /move_base/global_costmap/inflation_layer src/Reval/param/costmap_inflation_params.yaml", shell=True)
        subprocess.check_call("rosrun dynamic_reconfigure dynparam load /move_base/local_costmap/inflation_layer src/Reval/param/costmap_inflation_params.yaml", shell=True)
        loader = Loader("Sending the robot to home...", bcolors.CGREEN + "" + bcolors.ENDC, 0.05).start() 
        send_home = subprocess.check_call("python send_home.py '%s'", cwd="src/Reval/src/benchmark/", shell=True)
        loader.stop() 
        #kill_rosnode = subprocess.check_call("./kill_rosnode_turtlebot.sh &>/dev/null", cwd="src/Reval/src/benchmark/service", shell=True)
        time.sleep(5)     

    def tbot_phy_dynamic_reconfig(self, MoveBase, DWAPlannerROS, costmap_common, costmap_common_inflation):
        move_base = dynamic_reconfigure.client.Client('/move_base')            
        move_base_DWAPlanner = dynamic_reconfigure.client.Client('/move_base/DWAPlannerROS')
        move_base_global_costmap_inflation = dynamic_reconfigure.client.Client('/move_base/global_costmap/inflation_layer')
        move_base_global_costmap = dynamic_reconfigure.client.Client('/move_base/global_costmap')
        move_base_local_costmap_inflation = dynamic_reconfigure.client.Client('/move_base/local_costmap/inflation_layer')
        move_base_local_costmap = dynamic_reconfigure.client.Client('/move_base/local_costmap')

        move_base.update_configuration(MoveBase)
        move_base_DWAPlanner.update_configuration(DWAPlannerROS)
        move_base_global_costmap.update_configuration(costmap_common)
        move_base_global_costmap_inflation.update_configuration(costmap_common_inflation)
        move_base_local_costmap.update_configuration(costmap_common)
        move_base_local_costmap_inflation.update_configuration(costmap_common_inflation) 

    def tbot_phy_get_measurement(self):
        measurements = pd.read_csv("cure_log/MOBO_measurement_Turtlebot3_phy.csv")
        battery_percentage = measurements['Energy']
        mission_time = measurements['Mission_time']
        traveled_distance = measurements['Traveled_distance']
        planner_failed = measurements['Planner_failed']
        euc_distance = measurements['Positional_error']
        recovery_executed = measurements['Recovery_executed']
        misson_success = measurements['Mission_success']
        obs_distance = measurements['Obstacle_distance']
        task_completion_rate = measurements['RNS']
        pentalty = measurements['Penalty']
        max_voltage = 12.24 # observed full charge voltage
        cutoff_voltage = 9.0
        nominal_voltage = 11.1
        energy = 19.98 * (battery_percentage/max_voltage)
   
        return (float(energy), float(mission_time), 
                float(traveled_distance), float(planner_failed), 
                float(euc_distance), float(recovery_executed),
                int(misson_success), float(obs_distance), float(task_completion_rate),
                float(pentalty))


class HuskySimMeasurement():
    def __init__(self):
        print("[STATUS]: Initialing HuskySimMeasurement class using Reval API")
        rospy.init_node('update_config') 

    def launch_husky_sim(self, viz:bool):
        if viz:
            husky_gazebo = subprocess.check_call("./husky_gazebo.sh '%s'", cwd="src/Reval/src/benchmark/service", shell=True)
            for i in tqdm(range(8), desc="Launching husky_gazebo", colour=color, ascii=ASCII, bar_format='{l_bar}{bar:20}{r_bar}{bar:-20b}', leave=False):
                time.sleep(1)
            husky_mb = subprocess.check_call("./husky_movebase.sh '%s'", cwd="src/Reval/src/benchmark/service", shell=True)
            for i in tqdm(range(3),  desc="Launching husky_MobeBase", colour=color, ascii=ASCII, bar_format='{l_bar}{bar:20}{r_bar}{bar:-20b}', leave=False): 
                time.sleep(1)
            husky_rviz = subprocess.check_call("./husky_rviz.sh '%s'", cwd="src/Reval/src/benchmark/service", shell=True)
            for i in tqdm(range(10),  desc="Launching husky_rviz", colour=color, ascii=ASCII, bar_format='{l_bar}{bar:20}{r_bar}{bar:-20b}', leave=False): 
                time.sleep(1)
        else:
            husky_gazebo = subprocess.check_call("./husky_gazebo_nogui.sh '%s'", cwd="src/Reval/src/benchmark/service", shell=True)
            for i in tqdm(range(8), desc="Launching husky_gazebo", colour=color, ascii=ASCII, bar_format='{l_bar}{bar:20}{r_bar}{bar:-20b}', leave=False):
                time.sleep(1)
            husky_mb = subprocess.check_call("./husky_movebase.sh '%s'", cwd="src/Reval/src/benchmark/service", shell=True)
            for i in tqdm(range(3),  desc="Launching husky_MobeBase", colour=color, ascii=ASCII, bar_format='{l_bar}{bar:20}{r_bar}{bar:-20b}', leave=False): 
                time.sleep(1)
        battery_collision = subprocess.check_call("./husky_battery_bumper.sh '%s'", cwd="src/Reval/src/benchmark/service", shell=True)
        rosbag = subprocess.check_call("gnome-terminal -- ./ros_record.sh '%s'", cwd="src/Reval/src/benchmark/service", shell=True)
        calculate_distance = subprocess.check_call("gnome-terminal -- python calculate_distance_traveled.py '%s'", cwd="src/Reval/src/benchmark", shell=True)
        for i in tqdm(range(5),  desc="Data logger", colour=color, ascii=ASCII, bar_format='{l_bar}{bar:20}{r_bar}{bar:-20b}', leave=False): 
            time.sleep(1)
        print("")
        obs_distance = subprocess.check_call("gnome-terminal -- python get_obstacleDistance.py '%s'", cwd="src/Reval/src/benchmark/", shell=True)                
        for i in tqdm(range(5),  desc="Updating config", colour=color, ascii=ASCII, bar_format='{l_bar}{bar:20}{r_bar}{bar:-20b}', leave=False): 
            time.sleep(1)                 

    def launch_husky_task(self):
        loader = Loader("Mission in progress...", bcolors.CGREEN + "" + bcolors.ENDC, 0.05).start()
        loading = True
        nav2d_goal = subprocess.check_call("python mission_husky.py '%s'", cwd="src/Reval/src/benchmark/", shell=True)
        loader.stop() 
        loading = False
        print(bcolors.CGREEN + "Mission Done!" + bcolors.ENDC)
        for i in tqdm(range(10),  desc="Finishing simulation", colour=color, ascii=ASCII, bar_format='{l_bar}{bar:20}{r_bar}{bar:-20b}', leave=False):
            time.sleep(1)
        kill_rosnode = subprocess.check_call("./kill_rosnode_husky.sh &>/dev/null", cwd="src/Reval/src/benchmark/service", shell=True)
        for i in tqdm(range(5),  desc="Generating logs", colour=color, ascii=ASCII, bar_format='{l_bar}{bar:20}{r_bar}{bar:-20b}', leave=False):
            time.sleep(1)
        loader = Loader("Evaluating logs...", bcolors.HEADER + "" + bcolors.ENDC, 0.05).start()
        loading = True    
        eval = subprocess.check_call("./eval.sh '%s'", cwd="src/Reval/src/benchmark/service", shell=True)
        loader.stop()
        loading = False  
        evaluation_results = subprocess.check_call("python MOO_measurement_husky.py '%s'", cwd="src/Reval/src/benchmark/", shell=True) 
        time.sleep(5)

    def husky_dynamic_reconfig(self, MoveBase, DWAPlannerROS, costmap_common, costmap_common_inflation):
        move_base = dynamic_reconfigure.client.Client('/move_base')            
        move_base_DWAPlanner = dynamic_reconfigure.client.Client('/move_base/DWAPlannerROS')
        move_base_global_costmap_inflation = dynamic_reconfigure.client.Client('/move_base/global_costmap/inflation')
        move_base_global_costmap = dynamic_reconfigure.client.Client('/move_base/global_costmap')
        move_base_local_costmap_inflation = dynamic_reconfigure.client.Client('/move_base/local_costmap/inflation')
        move_base_local_costmap = dynamic_reconfigure.client.Client('/move_base/local_costmap')

        move_base.update_configuration(MoveBase)
        move_base_DWAPlanner.update_configuration(DWAPlannerROS)
        move_base_global_costmap.update_configuration(costmap_common)
        move_base_global_costmap_inflation.update_configuration(costmap_common_inflation)
        move_base_local_costmap.update_configuration(costmap_common)
        move_base_local_costmap_inflation.update_configuration(costmap_common_inflation) 

    def husky_get_measurement(self):
        measurements = pd.read_csv("cure_log/MOBO_measurement_Husky_sim.csv")
        battery_percentage = measurements['Energy']
        mission_time = measurements['Mission_time']
        traveled_distance = measurements['Traveled_distance']
        planner_failed = measurements['Planner_failed']
        euc_distance = measurements['Positional_error']
        recovery_executed = measurements['Recovery_executed']
        misson_success = measurements['Mission_success']
        obs_distance = measurements['Obstacle_distance']
        task_completion_rate = measurements['RNS']
        pentalty = measurements['Penalty']
        energy = round(400 - (400 * (battery_percentage/100)), 2)

        return (float(energy), float(mission_time), 
                float(traveled_distance), int(planner_failed), 
                float(euc_distance), int(recovery_executed),
                int(misson_success), float(obs_distance), float(task_completion_rate),
                float(pentalty))       