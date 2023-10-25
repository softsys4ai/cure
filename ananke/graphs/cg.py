"""
Class for Lauritzen-Wermuth-Frydenberg chain graphs (LWF-CGs/CGs).
"""

import logging
from .sg import SG

logger = logging.getLogger(__name__)


class CG(SG):

    def __init__(self, vertices=[], di_edges=set(), ud_edges=set(), **kwargs):
        """
        Constructor.

        :param vertices: iterable of names of vertices.
        :param di_edges: iterable of tuples of directed edges i.e. (X, Y) = X -> Y.
        :param ud_edges: iterable of tuples of undirected edges i.e. (X, Y) = X - Y.
        """

        # initialize vertices
        super().__init__(vertices=vertices, di_edges=di_edges, ud_edges=ud_edges, **kwargs)
        logger.debug("CG")

    def boundary(self, vertices):
        """
        Get the boundary of a set of vertices defined as the block and parents of the block.

        :param vertices: iterable of vertex names.
        :return: set corresponding to the boundary.
        """

        boundary = set()
        for v in vertices:
            boundary = boundary.union(self.block(v))
        boundary = boundary.union(self.parents(boundary))
        return boundary - set(vertices)
