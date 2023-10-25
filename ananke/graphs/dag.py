"""
Class for Directed Acyclic Graphs (DAGs).
"""
import logging
from .admg import ADMG
from .cg import CG
from .ug import UG
import itertools

logger = logging.getLogger(__name__)


class DAG(ADMG, CG):

    def __init__(self, vertices=[], di_edges=set(), **kwargs):
        """
        Constructor.

        :param vertices: iterable of names of vertices.
        :param di_edges: iterable of tuples of directed edges i.e. (X, Y) = X -> Y.
        """

        super().__init__(vertices=vertices, di_edges=di_edges, **kwargs)
        logger.debug("DAG")

    def d_separated(self, vertex1, vertex2, given):
        """
        Determine whether two vertices are d-separated given other vertices.
        
        :param vertex1: first vertex
        :param vertex2: second vertex
        :param given: list of given vertices
        
        :return: boolean result of d-separation
        """
        ancestral_vars = [vertex1, vertex2] + list(given)
            
        # create a new subgraph of the ancestors of vertex1, vertex2, and given vertices
        ancestral_subgraph = self.subgraph(self.ancestors(ancestral_vars)) 
        self_vertices = list(ancestral_subgraph.vertices)
        self_edges = list(ancestral_subgraph.di_edges)
        
        # retrieves all combinations of the graph's vertices
        pairs_of_vertices = [list(pair) for pair in itertools.combinations(self_vertices, 2)]
        
        # checks for common children between any pairs of vertices
        # if a pair of vertices has common children, an undirected edge connects the vertices
        for Vi, Vj in pairs_of_vertices:
            children_i = set(ancestral_subgraph.children(Vi))
            children_j = set(ancestral_subgraph.children(Vj))
            common_children = children_i.intersection(children_j)
            if len(common_children) > 0:
                self_edges.append((Vi, Vj))

        # removes given vertices from the graph
        for vertex in given: 
            self_vertices.remove(vertex)
            
        # removes any edges from the graph that include any of the given vertices
        for edge in self_edges[:]:
                if edge[0] in given or edge[1] in given:
                    self_edges.remove(edge)
                    
        # creates a new undirected graph from the updated vertices and edges
        augmented_graph = UG(self_vertices, self_edges)
        
        #checks if vertex 2 is in the block of vertex 1
        vertex1_block = augmented_graph.block(vertex1)
        return vertex2 not in vertex1_block
