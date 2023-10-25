import numpy as np
import operator
from sklearn.linear_model import RidgeCV

class FeatureSelection():
    def __init__(self):
        print("[STATUS]: Intitializing FeatureSelection class")
        # self.GP()    

    def get_rootCause(self, config_columns:list, config:np.array, 
                objective_name:str, objective:np.array, top_k:int):
        ridge = RidgeCV().fit(config, objective)
        importance = np.abs(ridge.coef_)
        feature_names = np.array(config_columns)
        feature_importance = sorted(list(zip(feature_names, importance)), key=operator.itemgetter(1), reverse=True)
        return {objective_name: (feature_importance[0:top_k])}
        