import dynamic_reconfigure

move_base_global_costmap_inflation = dynamic_reconfigure.client.Client('/move_base/global_costmap/inflation_layer')
move_base_global_costmap = dynamic_reconfigure.client.Client('/move_base/global_costmap')
move_base_global_costmap_obstacles = dynamic_reconfigure.client.Client('/move_base/global_costmap/obstacle_layer')


move_base_local_costmap_inflation = dynamic_reconfigure.client.Client('/move_base/local_costmap/inflation_layer')
move_base_local_costmap = dynamic_reconfigure.client.Client('/move_base/local_costmap')
move_base_local_costmap_obstacles = dynamic_reconfigure.client.Client('/move_base/local_costmap/obstacle_layer') 

move_base_DWAPlanner = dynamic_reconfigure.client.Client('/move_base/DWAPlannerROS') 
