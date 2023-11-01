from typing import List, Tuple, Dict
import numpy as np
import pandas as pd
from sklearn.linear_model import RidgeCV, Ridge
from joblib import dump, load
from sklearn.inspection import permutation_importance

class FeatureSelection():
    def __init__(self):
        print("[STATUS]: Initializing FeatureSelection class")

    def train(self, config: np.array, objectives: np.array, model_name: str):
        """This function fit the RidgeCV on the observational data"""
        self.ridge = RidgeCV().fit(config, objectives)
        self.save_model(self.ridge, model_name)        
        
    def save_model(self, model, model_name):
        filename = f"{model_name}.joblib"
        dump(model, filename)
        print(f"[STATUS]: Model saved as {filename}")

    def load_model(self, model_name):
        filename = f"{model_name}.joblib"
        return load(filename)
    
    def get_rootCause(self, outlier_data: pd.DataFrame, model_name: str, config_columns:pd.DataFrame.columns, 
                      obj_columns: pd.DataFrame.columns, config: np.array, objective_name: str, top_k: int) -> Dict[str, List[Tuple[str, float]]]:
        """This function computes the ridge coefficients and rank the configs"""
        loaded_model = self.load_model(model_name)
        objective_index = obj_columns.to_list().index(objective_name)
        # Extracting the model for the specific objective
        ridge = Ridge(alpha=loaded_model.alpha_)
        ridge.coef_ = loaded_model.coef_[objective_index]
        ridge.intercept_ = loaded_model.intercept_[objective_index]
        result = permutation_importance(ridge, config, outlier_data[f"{objective_name}"].values, n_repeats=1000)
        # Ranking features based on permutation importance
        ranked_features = sorted(list(zip(config_columns, result.importances_mean)), key=lambda pair: pair[1], reverse=True)
        return {objective_name: (ranked_features[0:top_k])}     