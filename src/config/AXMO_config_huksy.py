params = [
                {
                    "name": "controller_frequency",
                    "type": "range",
                    "bounds": [3.0, 7.0],
                },
                {
                    "name": "planner_patience",
                    "type": "range",
                    "bounds": [3.0, 7.0],  
                },
                {
                    "name": "controller_patience",
                    "type": "range",
                    "bounds": [3.0, 7.0],
                },
                {
                    "name": "conservative_reset_dist",
                    "type": "range",
                    "bounds": [1.0, 5.0],  
                },
                {
                    "name": "planner_frequency",
                    "type": "fixed",
                    "value": 0.0,
                },                
                {
                    "name": "oscillation_timeout",
                    "type": "fixed",
                    "value": 5.0,
                },
                {
                    "name": "oscillation_distance",
                    "type": "fixed",
                    "value": 0.5,  
                },
                {
                    "name": "acc_lim_theta",
                    "type": "range",
                    "bounds": [1.5, 5.2],
                },
                {
                    "name": "acc_lim_trans",
                    "type": "range",
                    "bounds": [0.1, 0.5],
                },
                {
                    "name": "acc_lim_x",
                    "type": "range",
                    "bounds": [1.0, 5.0],
                },                
                {
                    "name": "acc_lim_y",
                    "type": "fixed",
                    "value": 0.0,   
                },
                {
                    "name": "angular_sim_granularity",
                    "type": "fixed",
                    "value": 0.1,  
                },
                {
                    "name": "forward_point_distance",
                    "type": "range",
                    "bounds": [0.225, 0.725],  
                },                
                {
                    "name": "goal_distance_bias",
                    "type": "range",
                    "bounds": [5.0, 40.0],  
                },
                {
                    "name": "max_scaling_factor",
                    "type": "range",
                    "bounds": [0.1, 0.5],  
                },
                {
                    "name": "max_vel_theta",
                    "type": "range",
                    "bounds": [0.5, 2.0],  
                },
                {
                    "name": "max_vel_trans",
                    "type": "range",
                    "bounds": [0.3, 0.75],  
                },
                {
                    "name": "max_vel_x",
                    "type": "range",
                    "bounds": [0.3, 0.75],  
                },
                {
                    "name": "max_vel_y",
                    "type": "fixed",
                    "value": 0.0,  
                },
                {
                    "name": "min_vel_theta",
                    "type": "range",
                    "bounds": [1.5, 3.0],  
                },
                {
                    "name": "min_vel_trans",
                    "type": "range",
                    "bounds": [0.1, 0.2],  
                },
                {
                    "name": "min_vel_x",
                    "type": "range",
                    "bounds": [-0.3, 0.0],  
                },
                {
                    "name": "min_vel_y",
                    "type": "fixed",
                    "value": 0.0,  
                },
                {
                    "name": "occdist_scale",
                    "type": "range",
                    "bounds": [0.05, 0.5],  
                },

                {
                    "name": "oscillation_reset_angle",
                    "type": "range",
                    "bounds": [0.1, 0.5],  
                },

                {
                    "name": "oscillation_reset_dist",
                    "type": "fixed",
                    "value": 0.25,  
                },

                {
                    "name": "path_distance_bias",
                    "type": "range",
                    "bounds": [10.0, 50.0],  
                },

                {
                    "name": "scaling_speed",
                    "type": "range",
                    "bounds": [0.15, 0.35],
                },

                {
                    "name": "sim_granularity",
                    "type": "range",
                    "bounds": [0.015, 0.045],  
                },

                {
                    "name": "sim_time",
                    "type": "range",
                    "bounds": [0.5, 3.5],
                },

                {
                    "name": "stop_time_buffer",
                    "type": "range",
                    "bounds": [0.1, 1.5],
                },

                {
                    "name": "theta_stopped_vel",
                    "type": "range",
                    "bounds": [0.05, 0.15],
                },

                {
                    "name": "trans_stopped_vel",
                    "type": "range",
                    "bounds": [0.05, 0.15],  
                },

                {
                    "name": "twirling_scale",
                    "type": "fixed",
                    "value": 0,  
                },

                {
                    "name": "vth_samples",
                    "type": "range",
                    "bounds": [10, 30],  
                },

                {
                    "name": "vx_samples",
                    "type": "range",
                    "bounds": [3, 10],  
                },

                {
                    "name": "vy_samples",
                    "type": "range",
                    "bounds": [0, 15],  
                },

                {
                    "name": "xy_goal_tolerance",
                    "type": "fixed",
                    "value": 0.2,  
                },
                {
                    "name": "yaw_goal_tolerance",
                    "type": "fixed",
                    "value": 0.1,  
                },
                {
                    "name": "publish_frequency",
                    "type": "range",
                    "bounds": [1.0, 6.0],  
                },
                {
                    "name": "resolution",
                    "type": "range",
                    "bounds": [0.02, 0.15] ,  
                },
                {
                    "name": "transform_tolerance",
                    "type": "range",
                    "bounds": [0.2, 2.0],  
                },
                {
                    "name": "update_frequency",
                    "type": "range",
                    "bounds": [1.0, 6.0],  
                },
                {
                    "name": "cost_scaling_factor",
                    "type": "range",
                    "bounds": [1.0, 20.0],  
                },

                {
                    "name": "inflation_radius",
                    "type": "range",
                    "bounds": [0.3, 1.5],  
                },
            ]

MoveBase = {'controller_frequency',
            'planner_patience',
            'controller_patience',
            'conservative_reset_dist',
            'planner_frequency',
            'oscillation_timeout',
            'oscillation_distance'}

DWAPlannerROS = {'acc_lim_theta',
                'acc_lim_trans',
                'acc_lim_x',
                'acc_lim_y',
                'angular_sim_granularity',
                'forward_point_distance',
                'goal_distance_bias',
                'max_scaling_factor',
                'max_vel_theta',
                'max_vel_trans',
                'max_vel_x',
                'max_vel_y',
                'min_vel_theta',
                'min_vel_trans',
                'min_vel_x',
                'min_vel_y',
                'occdist_scale',
                'oscillation_reset_angle',
                'oscillation_reset_dist',
                'path_distance_bias',
                'scaling_speed',
                'sim_granularity',
                'sim_time',
                'stop_time_buffer',
                'theta_stopped_vel',
                'trans_stopped_vel',
                'twirling_scale',
                'vth_samples',
                'vx_samples',
                'vy_samples',
                'xy_goal_tolerance',
                'yaw_goal_tolerance'}

costmap_common = {'publish_frequency',
                'resolution',
                'transform_tolerance',
                'update_frequency'}

costmap_common_inflation = {'cost_scaling_factor',
                            'inflation_radius'}