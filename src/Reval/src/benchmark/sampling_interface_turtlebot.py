import os
import dynamic_reconfigure.client
import rospy
import pandas as pd
import time
from sampling_method import CarotSampling

def turtlebot():
    rospy.init_node('turtlebot3_config_py', anonymous=True)
    turtlebot_config = '../../config/turtlebot3_config_random-constant.yaml'

    move_base = dynamic_reconfigure.client.Client('/move_base')
    move_base_global_costmap_inflation = dynamic_reconfigure.client.Client('/move_base/global_costmap/inflation_layer')
    move_base_global_costmap = dynamic_reconfigure.client.Client('/move_base/global_costmap')
    move_base_local_costmap_inflation = dynamic_reconfigure.client.Client('/move_base/local_costmap/inflation_layer')
    move_base_local_costmap = dynamic_reconfigure.client.Client('/move_base/local_costmap')

    load_config = CarotSampling()
    load_config.load_config(turtlebot_config, 'Turtlebot3')
    move_base.update_configuration(load_config.node_MoveBase)
    time.sleep(1)
    move_base_DWAPlanner = dynamic_reconfigure.client.Client('/move_base/DWAPlannerROS')
    move_base_DWAPlanner.update_configuration(load_config.node_DWAPlannerROS)
    move_base_global_costmap.update_configuration(load_config.node_costmap_common)
    move_base_global_costmap_inflation.update_configuration(load_config.node_costmap_common_inflation)
    move_base_local_costmap.update_configuration(load_config.node_costmap_common)
    move_base_local_costmap_inflation.update_configuration(load_config.node_costmap_common_inflation)

    # Logging config
    config_dict = {**load_config.node_MoveBase,
                   **load_config.node_DWAPlannerROS,
                   **load_config.node_costmap_common,
                   **load_config.node_costmap_common_inflation}
    log_config = pd.DataFrame([config_dict])
    log_config.to_csv("log/config_turtlebot.csv", index=False, header=True)

if __name__ == '__main__':
    turtlebot()   
