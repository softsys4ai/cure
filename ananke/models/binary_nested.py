#!/usr/bin/env python3
"""
Implements the Nested Markov parameterization for acyclic directed mixed graphs over binary variables.

The code relies on the following papers:

[ER12] Evans, R. J., & Richardson, T. S. (2012). Maximum likelihood fitting of acyclic directed mixed graphs to binary data. arXiv preprint arXiv:1203.3479.
[ER19] Evans, R. J., & Richardson, T. S. (2019). Smooth, identifiable supermodels of discrete DAG models with latent variables. Bernoulli, 25(2), 848-876.

"""
import logging
import numpy as np
from scipy import optimize

from collections import ChainMap, namedtuple
from ..utils import powerset

from collections import OrderedDict

logger = logging.getLogger(__name__)


def permutations(n, k=2):
    """
    Computes tuples of all permutations of n variables with cardinality k.

    :param n: number of variables
    :param k: cardinality of each variable
    :return:
    """
    return [tuple(x) for x in np.rollaxis(np.mgrid[(slice(0, k, None),) * n], 0, n + 1).reshape(
        (k ** n, n))]


def recursive_partition_heads(B, intrinsic_dict):
    """
    Partition an arbitrary (sub)set of vertices B (in V) into recursive heads

    :param B: arbitrary possibly empty subset of vertices
    :param intrinsic_dict: map of intrinsic sets to heads and tails of that set

    """
    if len(B) == 0:  # if B is null set
        heads = []
    else:
        rH = get_recursive_heads(intrinsic_dict)
        B_star = B
        heads = []

        while len(B_star) > 0:  # some empty set condition

            pB = [set(element) for element in list(set(powerset(B_star, 0)))]
            rH_intersection_pB = [element for element in pB if element in rH]

            tau = maximal_heads(rH_intersection_pB, intrinsic_dict)
            heads.append(tau)

            tau_list = [item for sublist in [list(i) for i in tau] for item in sublist]
            B_star = B_star.difference(set(tau_list))

        heads = [item for sublist in [list(i) for i in heads] for item in sublist]
        heads = {frozenset(i) for i in heads}  # convert to frozen set so it can be used as a key

    return heads


def maximal_heads(list_H, intrinsic_dict):
    """
    Returns maximal heads for a set of heads. Only defined if all heads in list_H (list) are in IHT, ie, they are all heads of intrinsic sets
    """
    # create a dictionary that is the reverse mapping of heads:intrinsic sets
    reverse_dict = {frozenset(head_tail[0]): iset for iset, head_tail in intrinsic_dict.items()}
    list_iset = [reverse_dict[frozenset(x)] for x in list_H]  # the only sets we care about

    # order both lists by number of elements in each intrinsic set (descending)
    list_iset_nelem = [len(i) for i in list_iset]
    ord_iset = [x for _, x in sorted(zip(list_iset_nelem, list_iset), reverse=True)]
    ord_H = [x for _, x in sorted(zip(list_iset_nelem, list_H), reverse=True)]

    max_heads = []

    for ix, iset in enumerate(ord_iset):
        bigger_lst = ord_iset[:ix]  # all intrinsic sets "larger" than this one; does not include self.
        if len(bigger_lst) == 0:
            max_heads.append(ord_H[ix])
        else:
            temp = [iset.issubset(big_set) for big_set in
                    bigger_lst]  # can make this better. If iset is a subset of one bigger set we dont care about the others
            if not any(temp):
                max_heads.append(ord_H[ix])

    return max_heads


def get_recursive_heads(intrinsic_dict):
    """
    Compute all heads of intrinsic sets

    :param intrinsic_dict: mapping of intrinsic sets of some graph to heads and tails
    :return: list of all heads
    """

    return [v[0] for k, v in intrinsic_dict.items()]


def initialize_q_vector(intrinsic_dict):
    """
    Generates the q_vector, a dictionary mapping (heads, tails, value of tail) to parameter. Default q parameter values are
    1/2^#(head), for each head.

    :param intrinsic_dict: mapping of intrinsic sets of some graph to heads and tails
    :return q_vector:
    """
    q_vector = OrderedDict()
    ordered_values = sorted([tuple(tuple(sorted(y)) for y in x) for x in intrinsic_dict.values()])
    for head, tail in ordered_values:
        n_head = len(head)
        n_tail = len(tail)
        if n_tail == 0:
            q_vector[(frozenset(head), tuple(tail), ())] = 1 / (2 ** n_head)
        else:

            for permutation in permutations(n_tail):
                q_vector[(frozenset(head), tuple(tail), tuple(permutation))] = 1 / (2 ** n_head)

    return q_vector


def get_heads_tails_map(intrinsic_dict):
    """
    Get mapping of heads to tails from a mapping of intrinsic sets

    :param intrinsic_dict: mapping of intrinsic sets of some graph to heads and tails
    :return:
    """
    return {frozenset(value[0]): tuple(sorted(value[1])) for key, value in intrinsic_dict.items()}


def compute_likelihood_district(nu_dict, q_vector, district, heads_tails_map, intrinsic_dict):
    """
    Compute likelihood directly using Equation (1) of [ER12].

    This likelihood is not recommended for use in maximization as it is more efficiently expressed using M and P
    computations.

    :param nu_dict: a dictionary representing a single observation of variables to value
    :param q_vector: dictionary mapping the head, tail, tail value to parameter value
    :param district: district of graph
    :param heads_tails_map: mapping of heads to tails
    :param intrinsic_dict: mapping of intrinsic set to heads and tails
    :return:
    """
    """
    """
    # find all subsets of that district -  some of these will be B's (based on what nu is)
    pset = [frozenset(i) for i in
            list(powerset(district, 0))]  # convert to frozen set so you can set a dictionary with these keys later
    ## maybe for each district send in the powerset as a dictionary? 

    # compute partition of heads for each of these subsets
    partition_head_dict = {B: recursive_partition_heads(B, intrinsic_dict) for B in pset}

    O_j = [key for key, value in nu_dict.items() if (value == 0 and key in district)]
    B_j = [set(pset_item) for pset_item in pset if
           all(element in list(pset_item) for element in O_j)]  # subset of D_j but superset of O_j

    lld = 0
    for setB in B_j:
        partition = partition_head_dict[frozenset(setB)]
        term = 1
        for head in partition:  # if setB is empty, partition is also empty and term = 1 (defined)
            if len(heads_tails_map[head]) == 0:
                term = term * q_vector[(head, (), ())]
            else:
                tail = heads_tails_map[head]
                nu_tail = tuple([nu_dict[tail_element] for tail_element in tail])
                term = term * q_vector[head, tail, nu_tail]

        lld = lld + (-1) ** (len(setB) - len(O_j)) * term

    return lld


def compute_M(G, partition_head_dict, district, heads_tails_map, terms):
    """
    Compute M matrix as given in Section 3.1 of [ER12]

    :param G: ADMG graph
    :param partition_head_dict: dictionary mapping every subset of given district to constituent heads
    :param district: a district of the graph
    :param heads_tails_map: dictionary mapping heads to tails
    :param terms: list of terms of given district
    :return:
    """
    district = sorted(district)
    pset = [tuple(sorted(i)) for i in
            list(powerset(district, 0))]  # convert to frozen set so you can set a dictionary with these keys later
    all_nu = [OrderedDict(zip(sorted(list(G.vertices)), tuple(permutation))) for permutation in
              permutations(len(G.vertices))]
    all_term_dicts = []

    for nu_dict in all_nu:

        O_j = [key for key, value in nu_dict.items() if (value == 0 and key in district)]
        B_j = [(pset_item) for pset_item in pset if
               all(element in list(pset_item) for element in O_j)]  # subset of D_j but superset of O_j
        terms_dict = OrderedDict(zip(terms, [0] * len(terms)))
        for setB in B_j:
            all_tails = list()
            for item in partition_head_dict[setB]:
                tail = heads_tails_map[item]
                if len(tail) > 0:
                    all_tails.extend(tail)
            all_tails = tuple(sorted(list(set(all_tails))))
            values = tuple(nu_dict[x] for x in all_tails)

            terms_dict[(frozenset(setB), all_tails, values)] = (-1) ** (len(setB) - len(O_j))

        assert list(terms_dict.keys()) == terms, terms

        all_term_dicts.append(list(terms_dict.values()))

    return np.array(all_term_dicts)


def compute_P(partition_head_dict, q_vector_keys, heads_tails_map, terms):
    """
    Computes P matrix as given in Section 3.1 of [ER12]

    :param partition_head_dict: dictionary mapping every subset of given district to constituent heads
    :param q_vector_keys: list of parameter names
    :param heads_tails_map: dictionary mapping heads to tails
    :param terms: list of all terms for a given district
    :return:
    """
    all_P = []
    for term in terms:
        P_dict = OrderedDict(zip(q_vector_keys, [0] * len(q_vector_keys)))

        head = tuple(sorted(term[0]))
        all_tails = term[1]
        all_values = term[2]
        for item in partition_head_dict[head]:
            ix = [i for i in range(len(all_tails)) if all_tails[i] in heads_tails_map[item]]
            tail = tuple([all_tails[i] for i in ix])
            value = tuple([all_values[i] for i in ix])
            P_dict[(frozenset(item), tail, value)] = 1
        all_P.append(list(P_dict.values()))

    return np.array(all_P)


def compute_all_M_and_P(G, intrinsic_dict):
    """
    Computes for each district of a graph, the corresponding M and P matrices required to compute the expression

    p(q) = \prod_j M_j \exp(P_j \log q_j)

    See Section 3.1 of [ER12] for further details.

    :param G: ADMG
    :param intrinsic_dict: mapping of intrinsic set to heads and tails
    :return:
    """
    heads_tails_map = get_heads_tails_map(intrinsic_dict=intrinsic_dict)
    q_vector_keys = list(initialize_q_vector(intrinsic_dict).keys())
    Ms = []
    Ps = []
    districts = [frozenset(x) for x in G.districts]

    for D_j in districts:
        terms = compute_terms(D_j, intrinsic_dict, heads_tails_map)
        partition_head_dict = compute_partition_head_dict(intrinsic_dict=intrinsic_dict, district=D_j)
        M = compute_M(G=G,
                      partition_head_dict=partition_head_dict, district=D_j,
                      heads_tails_map=heads_tails_map,
                      terms=terms)
        # Select q parameters whose heads are in district D_j
        selected_q_vector_keys = [k for k in q_vector_keys if set(k[0]).issubset((D_j))]
        P = compute_P(partition_head_dict=partition_head_dict,
                      q_vector_keys=selected_q_vector_keys,
                      heads_tails_map=heads_tails_map,
                      terms=terms)
        Ms.append(M)
        Ps.append(P)
    return Ms, Ps


def compute_district_bool_map(q_vector_keys, districts):
    """

    :param q_vector_keys:
    :param districts:
    :return:
    """
    selected_keys = []
    districts = [frozenset(x) for x in districts]
    for D_j in districts:
        selected_keys.append([True if set(k[0]).issubset(D_j) else False for k in q_vector_keys])
    return OrderedDict(zip(districts, selected_keys))


def compute_partition_head_dict(intrinsic_dict, district):
    """
    Compute partition head dictionary. Maps every subset of a district to its constituent maximal recursive heads

    :param intrinsic_dict: dictionary mapping intrinsic sets to (heads, tails) of that set
    :param district: district of graph
    :return:
    """
    B_j = sorted([tuple(sorted(i)) for i in
                  list(powerset(district,
                                0))])  # convert to frozen set so you can set a dictionary with these keys later
    partition_head_dict = {B: recursive_partition_heads(frozenset(B), intrinsic_dict) for B in B_j}

    return partition_head_dict


def compute_terms(district, intrinsic_dict, heads_tails_map):
    """
    Computes list of terms (product of q parameters) and the partition head dictionary.
    Each term is a tuple of (frozenset(all heads), tuple(all tails), tuple(values of all tails)).

    :param district: district of graph
    :param intrinsic_dict: dictionary mapping intrinsic sets to (heads, tails) of that set
    :param heads_tails_map: dictionary mapping heads to tails
    :return:
    """
    # find all subsets of that district -  some of these will be B's (based on what nu is)
    B_j = sorted([tuple(sorted(i)) for i in
                  list(powerset(district,
                                0))])  # convert to frozen set so you can set a dictionary with these keys later

    # compute partition of heads for each of these subsets
    partition_head_dict = compute_partition_head_dict(intrinsic_dict=intrinsic_dict, district=district)
    heads_tails_values = list()
    for B in B_j:
        all_tails = list()
        for item in partition_head_dict[B]:
            tail = heads_tails_map[item]
            if len(tail) > 0:
                all_tails.extend(tail)
        all_tails = sorted(list(set(all_tails)))
        n_tail = len(all_tails)
        tail_permutations = permutations(n=n_tail)
        for value in tail_permutations:
            heads_tails_values.append((frozenset(B), tuple(all_tails), tuple(value)))

    return sorted(heads_tails_values)


def _compute_prob_single_district(q, M, P):
    """
    Computes the probability of a single district as per Eqn (3) of [ER12]

    :param q: q parameters involved in district
    :param M: M matrix for district
    :param P: P matrix for district
    :return:
    """
    prob = M @ np.exp(P @ np.log(q))
    return prob


def compute_q_indices_by_district(q_vector_keys, districts):
    """
    Computes a boolean indexing array that indicates which q parameters are involved in which districts.

    :param q_vector_keys:
    :param districts:
    :return:
    """
    indices = [[True if k[0].issubset(district) else False for k in q_vector_keys] for district in districts]
    return indices


def _compute_prob(q, q_indices, districts, Ms, Ps):
    """
    Compute full probability using matrix operations

    :param q: numpy array of q parameters (obtained from q_vector)
    :param q_indices: numpy array of booleans to select subset of active q for each district
    :param districts: list of districts
    :param Ms: list of M matrices
    :param Ps: list of P matrices
    :return:
    """
    prob = 1
    for i, (district, M, P) in enumerate(zip(districts, Ms, Ps)):
        new_q = q[q_indices[i]]
        prob *= _compute_prob_single_district(q=new_q, M=M,
                                              P=P)  # M @ np.exp(P @ np.log(new_q))
    return prob


def _compute_likelihood(q, counts, q_indices, districts, Ms, Ps):
    """
    Computes full log-likelihood

    :param q: numpy array of q parameters (obtained from q_vector)
    :param counts: n(V=v) for each value assignment v of variables V, obtained in sorted order
    :param q_indices: numpy array of booleans to select subset of active q for each district
    :param districts: list of districts
    :param Ms: list of M matrices
    :param Ps: list of P matrices
    :return:
    """
    prob = _compute_prob(q, q_indices, districts, Ms, Ps)
    lld = np.sum(counts * np.log(prob))
    return lld


def _compute_partial_neg_log_likelihood(x, counts, A, b):
    """
    Returns negative partial log-likelihood for optimization.
    This function is implicitly defined w.r.t. a variable v, such that the parameter x consists of all
    parameters in q whose heads contain v. This selected variable defines the corresponding A, b matrices.
    The probability vector is then computed as prob = A @ x - b

    :param x: subset of q
    :param counts: n(V=v) for each value assignment v of variables V, obtained in sorted order
    :param A: A matrix as per Eqn (4) of [ER12]
    :param b: b vector as per Eqn (4) of [ER12]
    :return:
    """
    lld = np.sum(counts * np.log(A @ x - b))
    return -lld


def _compute_jac_neg_log_partial_likelihood(x, counts, A, b):
    """
    Returns negative jacobian of partial log-likelihood for optimization

    :param x: subset of q
    :param counts: n(V=v) for each value assignment v of variables V, obtained in sorted order
    :param A: A matrix as per Eqn (4) of [ER12]
    :param b: b vector as per Eqn (4) of [ER12]
    :return:
    """
    jac = np.sum(A * (counts * 1 / (A @ x - b)).reshape(-1, 1), axis=0)
    return -jac


def _compute_hess_neg_log_partial_likelihood(x, counts, A, b):
    """
    Compute hessian of partial log-likelihood. Because the partial likelihood is a linear function, the hessian is zero.

    :param x: subset of q
    :param counts: n(V=v) for each value assignment v of variables V, obtained in sorted order
    :param A: A matrix as per Eqn (4) of [ER12]
    :param b: b vector as per Eqn (4) of [ER12]
    :return:
    """
    return np.zeros((x.shape[0], x.shape[0]))


def compute_theta_reindexed_bool_map(q_vector_keys, districts):
    """
    Creates a mapping from a variable to a boolean indexing vector.

    This boolean vector indexes only parameters whose heads are involved in the district of that variable.
    It selects parameters which have heads containing that variable.

    Used to construct A and b matrices for partial likelihood.

    :param q_vector_keys: list of q_vector keys
    :param districts: list of districts
    :return:
    """
    theta_reindexed_bool_map = dict()
    for district in districts:
        selected_q_vector_keys = [k for k in q_vector_keys if set(k[0]).issubset((district))]
        for v in district:
            theta_reindexed_bool_map[v] = np.array(
                [True if v in item[0] else False for i, item in enumerate(selected_q_vector_keys)])
    return theta_reindexed_bool_map


def compute_theta_bool_map(q_vector_keys, variables):
    """
    Compute map from variable to boolean indexing vector of q_parameters which have heads containing that variable.
    In this map, the indices are not reindexed by district. The boolean vector selects parameters which have heads containing that variable.

    :param q_vector_keys: list of q_vector keys
    :param variables: list of variables
    :return:
    """
    theta_bool_map = {v: np.array([True if v in item[0] else False for i, item in enumerate(q_vector_keys)]) for v in
                      variables}
    return theta_bool_map


def construct_A_b(variable, q, theta_reindexed_bool_map, M, P):
    """
    Constructs A and b matrices (eqn 4, Evans and Richardson 2013) for constraining parameters of given variable,
    in district which admits matrices M, P

    :param variable: name of variable
    :param q_vector: q_vector in OrderedDict format
    :param M: M matrix
    :param P: P matrix
    :return:
    """
    theta_bool_ix = theta_reindexed_bool_map[variable]
    theta_ix = np.argwhere(theta_bool_ix).ravel()

    comp_bool_ix = ~theta_bool_ix
    comp_ix = np.argwhere(comp_bool_ix).ravel()

    comp_val = q[comp_ix]
    P_theta = P[:, theta_ix]
    P_comp = P[:, comp_ix]

    b = -M[:, np.sum(P_theta, axis=1) == 0] @ np.exp(
        P_comp[np.sum(P_theta, axis=1) == 0, :] @ np.log(comp_val))

    A = np.concatenate([M[:, P[:, theta_ix[i]] == 1] @ np.exp(
        P[np.ix_(P[:, theta_ix[i]] == 1, comp_bool_ix)] @ np.log(comp_val)).reshape(-1, 1) for i in
                        range(len(theta_ix))], axis=1)

    return A, b



def process_data(df):
    """
    :param data: pandas DataFrame columns of variables and rows of observations
    :return: a vector of counts, ordered in ascending order of v for p(V=v)
    """
    count_dict = OrderedDict()
    colnames = list(df.columns)

    Counts = namedtuple('Counts', colnames)

    for permutation in permutations(len(colnames)):
        count_dict[(Counts(**dict(zip(colnames, permutation))))] = 0

    for tup in df.itertuples(index=False):
        permutation_tuple = Counts(**dict(zip(tup._fields, list(tup))))
        count_dict[permutation_tuple] += 1

    counts = np.array(list(count_dict.values()))

    return counts


class BinaryNestedModel:
    """
    Estimates parameters of nested binary model using the iterative maximum likelihood algorithm (Algorithm 1) of [ER12].
    Performs scipy.minimize constrained minimization of negative log-likelihood.
    """

    def __init__(self, graph, tol=1e-8):
        self.graph = graph
        self.tol = tol

        self.fitted_params = None


    def fit(self, X, q_vector=None, *args, **kwargs):
        """

        :param X: N x M pandas DataFrame
        :param args:
        :param kwargs:
        :return:
        """

        counts = process_data(X)

        return self._fit(counts, q_vector, *args, **kwargs)



    def _fit(self, counts, q_vector=None):
        intrinsic_dict, _ = self.graph.get_intrinsic_sets_and_heads()
        districts = [frozenset(x) for x in self.graph.districts]
        variables = list(self.graph.vertices)
        if q_vector is None:
            q_vector = initialize_q_vector(intrinsic_dict)
        Ms, Ps = compute_all_M_and_P(G=self.graph, intrinsic_dict=intrinsic_dict)
        variable_district_map = dict(ChainMap(*[{v: district for v in district} for district in districts]))
        Ms_map = dict(zip(districts, Ms))
        Ps_map = dict(zip(districts, Ps))
        district_bool_map = compute_district_bool_map(q_vector_keys=list(q_vector.keys()), districts=districts)
        theta_bool_map = compute_theta_bool_map(q_vector_keys=list(q_vector.keys()), variables=variables)
        theta_reindexed_bool_map = compute_theta_reindexed_bool_map(q_vector_keys=list(q_vector.keys()),
                                                                    districts=districts)
        q_indices = compute_q_indices_by_district(q_vector_keys=list(q_vector.keys()), districts=districts)

        q = np.array(list(q_vector.values()))
        diff = np.inf

        while diff > self.tol:
            lld_old = _compute_likelihood(q=q, counts=counts, q_indices=q_indices, districts=districts, Ms=Ms, Ps=Ps)
            logger.debug('current likelihood: {}'.format(lld_old))
            for v in variables:
                district = variable_district_map[v]
                q_district = q[district_bool_map[district]]
                M = Ms_map[district]
                P = Ps_map[district]
                A, b = construct_A_b(variable=v, q=q_district, theta_reindexed_bool_map=theta_reindexed_bool_map, M=M,
                                     P=P)
                theta = q[theta_bool_map[v]]
                constraint = optimize.LinearConstraint(A=A, lb=b, ub=np.inf, keep_feasible=False)
                res = optimize.minimize(_compute_partial_neg_log_likelihood, x0=theta, args=(counts, A, b),
                                        constraints=[constraint], method='trust-constr',
                                        jac=_compute_jac_neg_log_partial_likelihood,
                                        hess=_compute_hess_neg_log_partial_likelihood)
                q[theta_bool_map[v]] = res.x
            lld_new = _compute_likelihood(q=q, counts=counts, q_indices=q_indices, districts=districts, Ms=Ms, Ps=Ps)
            diff = lld_new - lld_old
            logger.debug('q: {}'.format(str(q)))
            logger.debug('likelihood increase: {}'.format(str(diff)))

        self.fitted_params = OrderedDict(zip(q_vector.keys(), q))

        return self
