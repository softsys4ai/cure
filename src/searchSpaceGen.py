import pickle
import src.pythonTranslation as pythongen

class SearchSpaceGen():
    def __init__(self, robot):
        global params, DWAPlannerROS, MoveBase, costmap_common, costmap_common_inflation
        print("[STATUS]: Initialing SearchSpaceGen class")
        if robot == "Turtlebot3_sim" or robot == "Turtlebot3_phy":
            from src.config.AXMO_config_turtlebot import params, DWAPlannerROS, MoveBase, costmap_common, costmap_common_inflation
        if robot == "Husky_sim":
            from src.config.AXMO_config_huksy import params, DWAPlannerROS, MoveBase, costmap_common, costmap_common_inflation

    def get_configSpcae(self, configs:list): 
        parameters = []
        nodes = {"MoveBase":[], "DWAPlannerROS":[],
                "costmap_common":[], "costmap_common_inflation":[]}
        search_space = [] 
        for config in range(len(configs)):
            # Move base
            if configs[config] in list(MoveBase):
                for i in range(len(params)):
                    if params[i]['name'] == configs[config]:
                        if "bounds" in params[i]:         
                            parameters.append(configs[config])
                            search_space.append({"name": configs[config], "type": "range", "bounds": params[i]['bounds']})
                            nodes["MoveBase"].append("'"+configs[config]+"'"+":"+configs[config])
    
            # DWAPlannerROS           
            if configs[config] in list(DWAPlannerROS):
                for i in range(len(params)):
                    if params[i]['name'] == configs[config]:
                        if "bounds" in params[i]:         
                            parameters.append(configs[config])
                            search_space.append({"name": configs[config], "type": "range", "bounds": params[i]['bounds']})
                            nodes["DWAPlannerROS"].append("'"+configs[config]+"'"+":"+configs[config])
                        else:
                            pass
                    else:
                        pass
            # costmap_common           
            if configs[config] in list(costmap_common):
                for i in range(len(params)):
                    if params[i]['name'] == configs[config]:
                        if "bounds" in params[i]:       
                            parameters.append(configs[config])  
                            search_space.append({"name": configs[config], "type": "range", "bounds": params[i]['bounds']})
                            nodes["costmap_common"].append("'"+configs[config]+"'"+":"+configs[config])
                        else:
                            pass
                    else:
                        pass
            # costmap_common_inflation           
            if configs[config] in list(costmap_common_inflation):
                for i in range(len(params)):
                    if params[i]['name'] == configs[config]:
                        if "bounds" in params[i]:       
                            parameters.append(configs[config])  
                            search_space.append({"name": configs[config], "type": "range", "bounds": params[i]['bounds']})
                            nodes["costmap_common_inflation"].append("'"+configs[config]+"'"+":"+configs[config])
                        else:
                            pass
                    else:
                        pass
            else:
                pass

        with open('cure_log/parameters.ob', 'wb') as fp:
            pickle.dump(parameters, fp)
            fp.close()  
        with open('cure_log/nodes.ob', 'wb') as fp:
            pickle.dump(nodes, fp)
            fp.close() 
        fname = 'src/config/AXMO_config_dynamic.py'
        with open(fname, "w") as myfile:
            myfile.write(f"params = {search_space}")
        pythongen.srcipt_gen()               
        print("[STATUS]: Search space generated!")  
        return parameters, nodes, search_space