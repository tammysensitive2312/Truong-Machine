"""
SM Harwood
16 October 2022
"""
import logging
import numpy as np
from scipy import sparse
from scipy.special import softmax
from ..routing_problem import RoutingProblem
try:
    import cplex
except ImportError:
    pass

logger = logging.getLogger(__name__)

class PathBasedRoutingProblem(RoutingProblem):
    """
    Class to implement the path-based formulation of a VRPTW
    Base math program is a 0-1 Integer Linear Program (ILP)
    which can be transformed to a
    Quadratic Unconstrained Binary Optimization (QUBO) problem

    ILP:

    min_x ∑_r c_r x_r
      s.t.
      ∑_r δ_k,r x_r = 1, for each node k,
      x_r ∈ {0,1}, for all routes r

    where
        x_r = 1 if route r is chosen (0 otherwise)
        c_r = cost of route r
        δ,r = 1 if route r visits node k

    The interpretation is that we wish to choose vehicle routes,
    so that each (non-depot) node is visited exactly once,
    while minimizing the total cost of traveling those routes.

    A valid route begins and ends at the "depot" node,
    and visits other nodes at most once while traversing available arcs.
    In addition, there are demand and delivery time window constraints that
    are encoded in what is a valid or feasible route
    """

    def __init__(self, vrptw=None):
        super().__init__(vrptw)
        self.routes = []
        self.route_costs = []
        self.route_node_visited = []

    def get_num_variables(self):
        return len(self.route_costs)

    def check_route(self, candidate_route):
        """ Check to see if this path is an allowed route; does it use valid arcs, visit nodes
        within allowed time windows, and satisfy cumulative demand

        args:
        candidate_route (list of int or list of strings): A route to potentially add, defined either
            as a list of node indexes or node names

        Return:
        feasible (bool): whether the route is a viable route
        cost (float): associated cost of route
        visits_node (list of int): List indicating whether
            this route visits Node[i] (visits_node[i] = 1)
            or not (visits_node[i] = 0)
        """
        feasible = True
        cost = 0
        visits_node = [0] * (len(self.nodes))
        if len(candidate_route) < 2:
            # must be at least 2 stops long
            return False, cost, visits_node

        route_indices = candidate_route
        # Convert from list of string node names to indices if necessary
        # (rest are done on the fly)
        check_indexes = (0, 1, -1)
        for index in check_indexes:
            if isinstance(route_indices[index], str):
                route_indices[index] = self.get_node_index(route_indices[index])

        # First check:
        # are first and last nodes the depot?
        if route_indices[0] != self.depot_index or route_indices[-1] != self.depot_index:
            return False, cost, visits_node
        # Make sure these are valid arcs,
        # visit each node at most once (besides depot)
        # satisfy capacity constraints
        # and time window constraints
        time = 0
        loading = self.initial_loading
        for i in range(len(route_indices) - 1):
            # have we already visited this node?
            if visits_node[route_indices[i]] == 1:
                return False, cost, visits_node
            else:
                visits_node[route_indices[i]] = 1

            # valid arc?
            # first, convert to index
            if isinstance(route_indices[i + 1], str):
                route_indices[i + 1] = self.get_node_index(route_indices[i + 1])
            a = (route_indices[i], route_indices[i + 1])
            # will increment time and loading appropriately
            feas_arc, time, loading = self.check_arc(time, loading, a)
            if feas_arc:
                cost += self.arcs[a].get_cost()
            else:
                return False, cost, visits_node
        # end for loop
        return feasible, cost, visits_node

    def check_arc(self, time, load, arc_key):
        """ Check if an arc is part of a valid route

        args:
        time (float): Time accumulated so far on route (time leaving origin of arc)
        load (float): Load on vehicle
        arc_key (tuple of int): The pair of node indices of potential arc

        return:
        feasibleArc (bool): Whether arc is feasible/part of valid route
        time (float): Updated time/arrival at destination of arc
        load (float): Updated load
        """
        # is the key even valid?
        try:
            arc = self.arcs[arc_key]
            dest = arc.get_destination()
        except KeyError:
            return False, time, load
        # Do we arrive before or during the next node's time window?
        # add travel time of current arc,
        # but if we arrive at a node before its time window, we have to wait
        time += arc.get_travel_time()
        time = max(time, dest.get_window()[0])
        if time > dest.get_window()[1]:
            logger.debug("Check arc %s: time window bad", arc_key)
            return False, time, load
        # Is the load physical (nonnegative and within capacity)?
        load += dest.get_load()
        if load > self.vehicle_cap or load < 0:
            logger.debug("Check arc %s: capacity bad", arc_key)
            return False, time, load
        return True, time, load

    def get_route_names(self, route):
        """ Given a sequence of indices, return the corresponding node names """
        return list(map(lambda n: self.node_names[n], route))

    def generate_route(self, vf=None, explore=1, node_costs=None, time_costs=None, unvisited=None):
        """ Generate a route
        Can view this as one iteration of an approximate dynamic programming method
        The goal is to find a good candidate route, and we always have exploration,
            so it is not "true" DP to find an optimal route

        args:
        vf (list-like of float): value function over nodes
        explore (float): Parameter controlling exploration/sampling;
            explore=0 : sample tightly around mode
            explore=np.inf : uniform sampling over keys
        node_costs (list-like of float): Cost of visiting a node (to add to stage costs)
            Zero if node_costs is None
        time_costs (function): Cost of arrival time at a node; takes a float and returns a float
        unvisited (list): List of unvisited node indices. Generated route will only contain nodes
            from this list. If None, all nodes are possible to visit

        Return:
        r (list of int): A route
        vf (list of float): updated (in place) value function
        """

        if vf is None:
            vf = [0] * len(self.nodes)
        else:
            assert len(vf) == len(self.nodes), 'Value function incorrect size'

        if unvisited is None:
            unvisited = list(range(len(self.nodes)))

        # all routes start at depot
        currNode = self.depot_index
        r = [currNode]
        time = 0
        load = self.initial_loading
        maxLegs = 2 + len(self.nodes)
        # Build up a route
        for _ in range(maxLegs):
            PotentialNodesAndVals = dict()
            # Loop over arcs to unvisited nodes,
            # get stage cost plus value function at each node
            for n in unvisited:
                nc = 0.0 if node_costs is None else node_costs[n]
                a = (currNode, n)
                # valid arc?
                feas_arc, new_time, _ = self.check_arc(time, load, a)
                tc = 0.0 if time_costs is None else time_costs(new_time)
                if feas_arc:
                    stageCost = self.arcs[a].get_cost()
                    PotentialNodesAndVals[n] = stageCost + nc + tc + vf[n]
                else:
                    continue
            # end for

            # Get a node to go to;
            # Minimize value function
            # Maybe add some randomization, proportional to this objective
            try:
                sampledNode, minNode = get_sampled_key(PotentialNodesAndVals, explore)
                a = (currNode, sampledNode)
                feas_arc, time, load = self.check_arc(time, load, a)
                # Update value function estimate:
                # ACTUAL minimizing cost-to-go: stageCost(a) + vf[minNode]
                vf[currNode] = PotentialNodesAndVals[minNode]
                currNode = sampledNode
            except AssertionError:
                # We might hit this if PotentialNodesAndVals is empty,
                # which might happen if unvisited doesn't have anything connected to depot
                break
            r.append(currNode)
            # if we have returned to the depot, we are done
            if currNode == self.depot_index:
                break
        # end loop over building up route
        return r, vf

    def add_route(self, route):
        """ If route is feasible, save its data

        args:
        route (list of int or list of string): A route to potentially add, defined either as a list
            of node indexes or node names

        Return:
        feas (bool): Is this route feasible/valid
        added (bool): whether the route was added
            (cost ?)
        """
        # If route r is feasible:
        #  cost = c_r
        #  visits_node[k] = \delta_k,r
        # However, with make_feasible heuristic, we might add nodes later
        # The route is just a list of indices, and doesn't need to know how
        #   many total node there are.
        # However, visits_node is an indicator array,
        #   and will be the incorrect length if nodes are added.
        # Convert to a list of node indices as well
        feas, cost, visits_node = self.check_route(route)
        visits_node_indices = np.flatnonzero(visits_node)
        added = False
        if feas and route not in self.routes:
            self.routes.append(list(route))
            self.route_costs.append(cost)
            self.route_node_visited.append(visits_node_indices)
            added = True
        return feas, added

    def add_routes_better(self, explore, node_costs, time_costs):
        """ Add routes in a more constructive way

        args:
        explore (float): Parameter controlling exploration/sampling;
            explore=0 : sample tightly around mode
            explore=np.inf : uniform sampling over keys
        node_costs (list-like of float): Cost of visiting a node (to add to stage costs)
            Zero if node_costs is None
        time_costs (function): Cost of arrival time at a node; takes a float and returns a float

        return:
        unvisited_indices (list): List of indices of still unvisited nodes after
            this construction heuristic
        """
        num_vehicles = self.estimate_max_vehicles()
        unvisited_indices = list(range(len(self.nodes)))
        routes = []

        # The number of routes that can be added is limited by the number of vehicles
        for _ in range(num_vehicles):
            r, _ = self.generate_route(None, explore, node_costs, time_costs, unvisited_indices)
            feas, _ = self.add_route(r)
            if not feas:
                continue
            # else, route is feasible/valid,
            # whether or not it was added, update unvisited indices
            routes.append(r)
            for n in r:
                if n == self.depot_index:
                    continue
                unvisited_indices.remove(n)
        # end loop
        return unvisited_indices, routes

    def make_feasible(self, high_cost):
        """
        Some sort of greedy construction heuristic to make sure the problem is
        feasible. We add dummy node/arcs as necessary to emulate more
        vehicles being available.
        """
        # Add routes greedily (no/little exploration),
        # but keep the list of unvisited nodes.
        exit_penalty = np.max(self.route_costs)
        time_penalty = 10
        node_costs = [0]*len(self.nodes)
        node_costs[self.depot_index] = exit_penalty
        time_costs = lambda t: time_penalty*t
        unvisited_indices, routes = self.add_routes_better(0, node_costs, time_costs)
        unvisited_indices.remove(self.depot_index)

        # Add arcs and construct routes through the nodes that remain unvisited
        # Note that we will add a node, which also adds a new CONSTRAINT to the
        # problem - but the constructed route will automatically satisfy the new
        # constraint.
        # Make new arcs as costly as most expensive (regular) route
        # high_cost = np.max(self.route_costs)
        depot_name = self.node_names[self.depot_index]
        for u in unvisited_indices:
            # Add arcs and a route through the unvisited node;
            # to be a valid arc/route, the loading constraints must be satisfied
            # (see check_arc)
            # Add a dummy node to ensure loading is satisfied
            # initial + new_node + unvisited \in [0, vehicle_cap]
            loading = self.initial_loading + self.nodes[u].get_load()
            if loading < 0:
                new_node_loading = -loading
            elif loading > self.vehicle_cap:
                new_node_loading = self.vehicle_cap - loading
            else:
                new_node_loading = 0
            logger.debug(f"Unvisited node: {self.node_names[u]}, "+\
                         f"loading: {self.nodes[u].get_load()}")
            new_node = f"mf_Dum_{u}"
            # Add node - remember, node is defined by DEMAND = -LOADING
            self.add_node(new_node, -new_node_loading)
            new_node_index = self.node_names.index(new_node)
            # Add arcs and route through unvisited node
            # cost of route will be (at least?) twice high_cost
            self.add_arc(depot_name, new_node, 0, high_cost)
            self.add_arc(new_node, self.node_names[u], 0, high_cost)
            # do we need an arc from the unvisited node?
            try:
                self.arcs[(u, self.depot_index)]
            except KeyError:
                self.add_arc(self.node_names[u], depot_name, 0, 0)
            r = [self.depot_index, new_node_index, u, self.depot_index]
            routes.append(r)
            feas, _ = self.add_route(r)
            logger.info("New feasibility route: %s", r)
            assert feas, "Something not right in make_feasible"

        # construct and save feasible solution
        feas_sol = np.zeros(len(self.route_costs))
        for r in routes:
            i = self.routes.index(r)
            feas_sol[i] = 1
        self.feasible_solution = feas_sol
        return

    def get_math_program_data(self):
        """
        Get the data of an integer linear program formulation of the problem
        """
        num_nodes = len(self.nodes)
        num_vars = len(self.route_costs)
        mp_cost = np.array(self.route_costs)
        # constraints are: visit all nodes (EXCEPT depot) exactly once
        # route_node_visited is a list of the nodes each route visits;
        # use this to construct constraint matrix sparsely,
        # then get rid of row corresponding to depot
        mp_constraints_rhs = np.ones(num_nodes - 1)
        mp_constraints_matrix = np.zeros((num_nodes, num_vars))
        for j, node_indices in enumerate(self.route_node_visited):
            col_indices = j*np.ones(len(node_indices), dtype=int)
            mp_constraints_matrix[node_indices, col_indices] = 1
        mask = np.ones(num_nodes, dtype=bool)
        mask[self.depot_index] = False
        mp_constraints_matrix = sparse.csr_array(mp_constraints_matrix[mask, :])
        return mp_cost, mp_constraints_matrix, mp_constraints_rhs

    def get_objective_data(self):
        """
        Return objective information in a consistent way
        objective(x) = cᵀx + xᵀ Q x

        Parameters:

        Return:
            c (array): 1-d array defining linear part of objective
            Q (array): 2-d array defining quadratic part of objective
        """
        n = self.get_num_variables()
        return np.asarray(self.route_costs), sparse.csr_array((n,n))

    def get_constraint_data(self):
        """
        Return constraints in a consistent way
        A_eq * x = b_eq
        xᵀ * Q_eq * x = r_eq

        Parameters:

        Return:
            A_eq (array): 2-d array of linear equality constraints
            b_eq (array): 1-d array of right-hand side of equality constraints
            Q_eq (array): 2-d array of a single quadratic equality constraint
                (potentially all zeros if there are no nontrivial quadratic constraints)
            r_eq (float): Right-hand side of the quadratic constraint
        """
        _, A_eq, b_eq = self.get_math_program_data()
        n = self.get_num_variables()
        return A_eq, b_eq, sparse.csr_array((n,n)), 0

    def get_sufficient_penalty(self, feasibility):
        """
        Return a threshhold value of the penalty parameter that is sufficient
        for penalizing the constraints when constructing a QUBO representation of
        this problem

        Parameters:
            feasibility (bool): Whether this is for a feasibility version of the
                problem. Sufficient penalty value can be zero

        Return:
            sufficient_pp (float): Penalty parameter value
        """
        if feasibility:
            sufficient_pp = 0.0
        else:
            sufficient_pp = sum(np.fabs(cost) for cost in self.route_costs)
        return sufficient_pp

    # def get_cplex_prob(self):
    #     """ Get a CPLEX object containing the BLEC/MIP representation
    #
    #     args:
    #     None
    #
    #     Return:
    #     cplex_prob (cplex.Cplex): A CPLEX object defining the MIP problem
    #     """
    #     cplex_prob = cplex.Cplex()
    #
    #     # Get BLEC/MIP data (but convert to lists for CPLEX)
    #     mp_cost, mp_constraints_matrix, mp_constraints_rhs = self.get_math_program_data()
    #     # Variables: all binary
    #     # constraints: all equality
    #     var_types = [cplex_prob.variables.type.binary] * len(mp_cost)
    #     con_types = ['E'] * len(mp_constraints_rhs)
    #     # (rows, cols) = np.nonzero(mp_constraints_matrix)
    #     # vals = mp_constraints_matrix[(rows, cols)]
    #     # rows = rows.tolist()
    #     # cols = cols.tolist()
    #     # vals = vals.tolist()
    #     constraints_mat = sparse.coo_array(mp_constraints_matrix)
    #     rows = constraints_mat.row.tolist()
    #     cols = constraints_mat.col.tolist()
    #     vals = constraints_mat.data.tolist()
    #
    #     # Variable names: node index sequence
    #     # Given a route (a list of indices), convert to a single string of those indices
    #     route_namer = lambda r: 'r_' + '_'.join(map(lambda i: f'{i}', r))
    #     vnames = list(map(route_namer, self.routes))
    #     # Constraint names: name after node index
    #     cnames = list(map(lambda i: f"cNode_{i}", range(len(self.nodes))))
    #     cnames.pop(self.depot_index)
    #
    #     # define object
    #     cplex_prob.objective.set_sense(cplex_prob.objective.sense.minimize)
    #     cplex_prob.variables.add(obj=mp_cost.tolist(), types=var_types, names=vnames)
    #     cplex_prob.linear_constraints.add(rhs=mp_constraints_rhs.tolist(), senses=con_types,
    #                                       names=cnames)
    #     cplex_prob.linear_constraints.set_coefficients(zip(rows, cols, vals))
    #     return cplex_prob

    def get_routes(self, solution):
        """ Get the list of list of nodes corresponding to a solution """
        soln_var_indices = np.flatnonzero(solution)
        routes = []
        for i in soln_var_indices:
            routes.append(self.get_route_names(self.routes[i]))
        return routes

def get_sampled_key(key_val, explore):
    """ Sample the keys of a dictionary inversely proportional to the real values

    The idea is to transform the values into a probability distribution with probability inversely
    proportional to the values, so that the mode corresponds to the smallest value. To do this, use
    a softmax on the negative of the values

    args:
    explore (float): Parameter controlling how much we explore
        explore = 0      : samples more tightly around mode      (makes MORE sensitive to scale)
        explore = np.inf : gives more randomization/exploration  (makes LESS sensitive to scale)

    return:
    sampled_key: A (potentially randomly) sampled key of the dictionary
    minimum_key: The key corresponding to the actual minimum value
    """
    assert key_val, 'Dictionary to sample is empty'
    assert explore >= 0, 'explore must be non-negative'

    # Get and return true minimum anyway
    minimum_key = min(key_val, key=key_val.get)

    # Sample keys according to probabilities obtained from softmax,
    # using appropriate scaling
    scaled_vals = -np.fromiter(key_val.values(), dtype=float) / (1e-4 + explore)
    pmf = softmax(scaled_vals)
    try:
        sampled_key = np.random.choice(list(key_val.keys()), p=pmf)
    except ValueError:
        # make sure pmf is really a pmf
        pmf /= np.sum(pmf)
        sampled_key = np.random.choice(list(key_val.keys()), p=pmf)
    return sampled_key, minimum_key
