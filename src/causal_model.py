import pandas as pd
import numpy as np
import math
import operator
import collections
import networkx as nx
from dowhy import gcm
from causallearn.search.ConstraintBased.FCI import fci
from causallearn.utils.cit import fisherz
from causallearn.utils.GraphUtils import GraphUtils
from causallearn.utils.PCUtils.BackgroundKnowledge import BackgroundKnowledge
from collections import defaultdict
from ananke.estimation import CausalEffect
import warnings
warnings.filterwarnings('ignore')


class CausalModel:
    def __init__(self, columns):
        print("[STATUS]: Initializing CausalModel class")      
        self.colmap={}
        for i in range(len(columns)):
            self.colmap[i] = columns[i]    

    def get_tabu_edges_care_remove(self, options, objectives, metrics):
        """This function is used to exclude edges which are not possible"""
        tabu_edges=[]

        # constraint on config - config
        # for opt in options:
        #     for cur_elem in options:
        #         if cur_elem != opt:
        #             tabu_edges.append((opt, cur_elem))
        
        # constraint on met - config
        for met in metrics:
            for cur_elem in options:
                if cur_elem != met:
                    tabu_edges.append((met, cur_elem))

        # constraint on obj - met
        for obj in objectives:
            for cur_elem in metrics:
                if cur_elem != obj:
                    tabu_edges.append((obj, cur_elem))        
        
        # constraints on config - obj
        for opt in options:
            for cur_elem in objectives:
                if cur_elem != opt:
                    tabu_edges.append((opt, cur_elem))
                    tabu_edges.append((cur_elem, opt))
   
        return tabu_edges

    def care_fci(self, df, tabu_edges_remove, alpha:float, verbose:bool):
            """This function is used to learn model using FCI"""
            df_arr = np.array(df)
            G, edges = fci(df_arr, fisherz, alpha, verbose=verbose)  
            nodes = G.get_nodes()   
            bk = BackgroundKnowledge()  
            for ce in tabu_edges_remove:
                f = list(self.colmap.keys())[list(self.colmap.values()).index(ce[0])]
                s = list(self.colmap.keys())[list(self.colmap.values()).index(ce[1])]     
                bk.add_forbidden_by_node(nodes[f], nodes[s])            
            try:    
                G, edges = fci(df_arr, fisherz, alpha, verbose=verbose, background_knowledge=bk)
            except:
                G, edges = fci(df_arr, fisherz, alpha, verbose=verbose)    

            fci_edges = []
            for edge in edges:
                fci_edges.append(str(edge))
        
            # Viz
            pdy = GraphUtils.to_pydot(G, edges,labels=list(df.columns))          
            pdy.write_png('fig/FCI_label.png')
            pdy = GraphUtils.to_pydot(G, edges)          
            pdy.write_png('fig/FCI.png')
            
            return fci_edges

    def resolve_edges(self, DAG, PAG, columns, tabu_edges, objectives):
        """This function is used to resolve fci (PAG) edges"""
        print("[STATUS]: Resolving edges using entropy")
        bi_edge = "<->"
        directed_edge = "-->"
        undirected_edge = "o-o"
        trail_edge = "o->"
        #  entropy only contains directed edges.
        options = {}
        for opt in columns:
            options[opt]= {}
            options[opt][directed_edge]= []
            options[opt][bi_edge]= []
        # add DAG edges to current graph
        for edge in DAG:
            if edge[0] or edge[1] is None:
                options[edge[0]][directed_edge].append(edge[1])    
        # replace trail and undirected edges with single edges using entropic policy
        for i in range (len(PAG)):
            if trail_edge in PAG[i]:
                PAG[i]=PAG[i].replace(trail_edge, directed_edge)
            elif undirected_edge in PAG[i]:
                    PAG[i]=PAG[i].replace(undirected_edge, directed_edge)
            else:
                continue
        # update causal graph edges
        for edge in PAG:
            cur = edge.split(" ")
            if cur[1]==directed_edge:
                
                node_one = self.colmap[int(cur[0].replace("X", ""))-1]               
                node_two = self.colmap[int(cur[2].replace("X", ""))-1]
                options[node_one][directed_edge].append(node_two)
            elif cur[1]==bi_edge:
                node_one = self.colmap[int(cur[0].replace("X", ""))-1]
                node_two = self.colmap[int(cur[2].replace("X", ""))-1]
                
                options[node_one][bi_edge].append(node_two)
            else: print ("[ERROR]: unexpected edges")
        # extract mixed graph edges 
        single_edges=[]
        double_edges=[]
        
        for i in options:
            options[i][directed_edge]=list(set(options[i][directed_edge]))
            options[i][bi_edge]=list(set(options[i][bi_edge]))
        for i in options:
            for m in options[i][directed_edge]:
                single_edges.append((i,m))
            for m in options[i][bi_edge]:
                double_edges.append((i,m))
        s_edges=list(set(single_edges)-set(tabu_edges))
        single_edges = []
        for e in s_edges: 
            if e[0]!=e[1]:
                single_edges.append(e)
        
        for i in range(int(len(s_edges)/2)):
             for obj in objectives:
                 if s_edges[i][0]!=s_edges[i][1]:
                     single_edges.append((s_edges[i][0],obj))
       
        double_edges=list(set(double_edges)-set(tabu_edges))
        # print ("--------------------------------------------------------------")
        # print ("Connections discovered by the causal graph")
        # print (single_edges)
        # print ("--------------------------------------------------------------")
        return single_edges, double_edges

    def get_causal_paths(self, columns, di_edges,
                         bi_edges, objectives):
        """This function is used to discover causal paths from an objective node"""
        CG = Graph(columns)
        causal_paths={}
        for edge in di_edges:
            CG.add_edge(edge[1], edge[0])
     
        for edge in bi_edges:
            CG.add_edge(edge[1], edge[0])
        for obj in objectives:
            CG.get_all_paths(obj)
            causal_paths[obj] = CG.path
    
        return causal_paths 

    def compute_path_causal_effect(self, df, paths, 
                                   G, k):
        """This function is used to compute P_ACE for each path"""
        ACE = {}
        for path in paths:
            for i in range(0, len(path)):
                try:
                    if i > 0:
                        obj= CausalEffect(graph=G, treatment=path[i],outcome=path[0])
                        ace, Ql, Qu = obj.compute_effect(df, "gformula", n_bootstraps = 5, alpha=0.05) # computing the effect
                        ACE[str(path)] = [ace, Ql, Qu]
                        print({"Path": str(path), "ACE": ace, "Ql": Ql, "Qu": Qu})
                except:
                    continue      
        df_result = pd.DataFrame.from_dict(ACE)
        (df_result.T).to_csv("cure_log/care_ace_result.csv", header=False)
        return paths 
    
    def DowhyCausalEffect(self, obj_isnot_config:list, di_edges:list, bi_edges:list, 
                          normal_data, anomaly_data, target_node:str,
                          n_bootstrap:int, top_k:int, verbose:bool):
        # print("[STATUS]: Computing Causal Effects")
        causal_model = gcm.StructuralCausalModel(nx.DiGraph(di_edges))
        gcm.auto.assign_causal_mechanisms(causal_model, normal_data)
        if verbose == True:
            gcm.config.enable_progress_bars() # to enable print statements when computing Shapley values
        if verbose == False:
            gcm.config.disable_progress_bars() # to disable print statements when computing Shapley values
        median_attribs, uncertainty_attribs = gcm.confidence_intervals(
            gcm.fit_and_compute(gcm.attribute_anomalies,
                                causal_model,
                                normal_data,
                                target_node=target_node,
                                anomaly_samples=anomaly_data),
                        num_bootstrap_resamples=n_bootstrap)
        print(median_attribs)
        for config in obj_isnot_config:
            median_attribs.pop(config, "pass")

        causal_effect = sorted(median_attribs.items(), key=operator.itemgetter(1), reverse=True)[0:top_k]
        return{target_node: (causal_effect[0:top_k])}

    def CausalEffect(self, G, data, config_col, target, top_k:int,
                     n_bootstraps:int, alpha:float, verbose:bool):
        CE ={}
        if verbose:
            print(f"[STATUS]: Estimating average causal effect on {target}\n")
            print("{:<25} {:<10}".format('Option', 'ACE'))
        for i in range(len(config_col.columns)):
            try:
                ace_obj = CausalEffect(graph=G, treatment=config_col.columns[i], outcome=target) 
                ace, Ql, Qu = ace_obj.compute_effect(data, "aipw", n_bootstraps=n_bootstraps, alpha=alpha) 
            except:
                continue
            if verbose:
                print("{:<25} {:<10}".format(config_col.columns[i], abs(ace)))
                # print(target, config_col.columns[i], abs(ace))
            if math.isnan(abs(ace)) or math.isinf(abs(ace)): 
                continue
            CE[config_col.columns[i]] = np.float64(abs(ace))
        causal_effect = sorted(CE.items(), key=operator.itemgetter(1), reverse=True)[0:top_k]
        if verbose:
            print("")
        return{target: (causal_effect[0:top_k])}

class Graph:
    def __init__(self, vertices):
        # No. of vertices
        self.V = vertices
        # default dictionary to store graph
        self.graph = defaultdict(list)

    def add_edge(self, u, v):
        self.graph[u].append(v)

    def get_all_paths_util(self, u, visited, path):

        visited[u]= True
        path.append(u)
        # If current vertex is same as destination, then print
        if self.graph[u] == []:
            try:
                if self.path:
                    self.path.append(path[:])
            except AttributeError:
                    self.path=[path[:]]
        else:
            for i in self.graph[u]:
                if visited[i]== False:
                    self.get_all_paths_util(i, visited, path)

        # remove current vertex from path[] and mark it as unvisited
        path.pop()
        visited[u]= False

    def get_all_paths(self, s):
        # mark all the vertices as not visited
        visited = {}
        for i in self.V:
            visited[i] = False
        # create an array to store paths
        path = []
        # call the recursive helper function to print all paths
        self.get_all_paths_util(s, visited, path)