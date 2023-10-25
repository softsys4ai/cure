import yaml
import random
import time

class CarotSampling():
    def config_method(self, config_file):
        self.applied_config = {}
        for node, param in config_file.items():
            for value in param:
                # index 0-> method: value
                if list(param[value].values())[0] == 'random':
                    # index 1-> value: [value_1, value_2]
                    if type(list(param[value].values())[1][0]) == bool:
                        val = random.choice([True, False])    
                    if type(list(param[value].values())[1][0]) == float:
                        val = round(random.uniform(list(param[value].values())[1][0],
                                            list(param[value].values())[1][1]), 2)
                    if type(list(param[value].values())[1][0]) == int:
                        val = random.randint(list(param[value].values())[1][0],
                                            list(param[value].values())[1][1])                 
                if list(param[value].values())[0] == 'constant':       
                    val = list(param[value].values())[1]
                if list(param[value].values())[0] == 'causal-intervention':
                    pass    
                self.applied_config.setdefault(node, {}).setdefault(value, val)  

        return self.applied_config

    # def ymal_gen(self, node, node_name:str, platform:str):
    #     with open(f'../../config/tmp/{node_name}_{platform}.yaml', 'w') as yaml_file:
    #         yaml_file.write( yaml.dump(node, default_flow_style=False, sort_keys=False))
    #         self.filename = f'{node_name}_{platform}.yaml'
    #         return self.filename

    def load_config(self, load_config, platform:str):
        self.node_DWAPlannerROS = {}
        with open(load_config, 'r') as file:
            self.get_config = yaml.safe_load(file)
        self.set_config = self.config_method(self.get_config)
        self.node_MoveBase = self.set_config['move_base']
        self.node_DWAPlannerROS = self.set_config['DWAPlannerROS'] 
        self.node_costmap_common = self.set_config['costmap_common']
        self.node_costmap_common_inflation = self.set_config['costmap_common_inflation']
