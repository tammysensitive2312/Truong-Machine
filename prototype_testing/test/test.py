import time

from vrp_solver_2 import main


def benchmark():
    test_cases = [
        ('small_instance.json', 10, 2),
        ('medium_instance.json', 50, 5),
        ('large_instance.json', 100, 10)
    ]

    results = []
    for case, n_customers, n_vehicles in test_cases:
        start = time.time()
        main(case, 'output.h5')
        duration = time.time() - start
        results.append((n_customers, n_vehicles, duration))

    print("Benchmark Results:")
    for r in results:
        print(f"Customers: {r[0]}, Vehicles: {r[1]}, Time: {r[2]:.2f}s")

def make_feasible(self, high_cost):
    """
    Some sort of greedy construction heuristic to make sure the problem is
    feasible. We add dummy nodes/arcs as necessary to emulate more
    vehicles being available.
    """
    # Initialize list of unvisited node indices
    # remove depot
    unvisited_indices = list(range(len(self.nodes)))
    unvisited_indices.remove(0)

    # starting from depot,
    # go through unvisited nodes,
    # greedily visit first unvisited node that we can (satisfying timing constraints)
    # repeat until outgoing arcs from depot are exhausted
    used_arcs = []
    max_vehicles = self.estimate_max_vehicles()
    for _ in range(max_vehicles):
        # start from depot, build a route
        current_node = 0  # depot is first node
        current_time = self.time_points[0]
        building_route = True
        while building_route:
            # go through unvisited nodes, choose first available
            best_node = None
            best_arrival = np.inf
            for n in unvisited_indices:
                arc = (current_node, n)
                if self.check_arc(arc):
                    # this is a potentially allowed arc- need to check timing
                    t_w = self.nodes[n].get_window()
                    arrival_actual, in_tp = self.get_arrival_time(current_time, arc)
                    if not in_tp:
                        # arrival time is beyond current time_points
                        continue
                    if arrival_actual <= min(t_w[1], best_arrival):
                        # this timing is valid AND we arrive earlier than all others yet
                        best_node = n
                        best_arrival = arrival_actual
            if best_node is not None:
                # record arc used, update position + time
                used_arcs.append((current_node, current_time, best_node, best_arrival))
                current_node = best_node
                current_time = best_arrival
                unvisited_indices.remove(best_node)
            else:
                # route cannot be continued
                # Break if already at depot
                # Add arc back to depot if possible
                building_route = False
                if current_node == 0:
                    break
                arc = (current_node, 0)
                assert self.check_arc(arc), f"No arcs back to depot from {current_node}"
                t_w = self.nodes[0].get_window()
                arrival_actual, in_tp = self.get_arrival_time(current_time, arc)
                assert in_tp, f"No arcs back to depot from {current_node} within time horizon"
                used_arcs.append((current_node, current_time, 0, arrival_actual))
                # We could potentially add an arc back to depot,
                # but I think this is messy and an indicator of a malspecified
                # problem...
                # self.check_and_add_exit_arc(current_node)
                # # Make sure that we can get back to depot with the discrete
                # # time points available
                # arrival_actual, in_tp = self.get_arrival_time(current_time, arc)
                # if not in_tp:
                #     # if arrival time is not in time_points, add it in
                #     self.time_points = np.append(self.time_points, arrival_actual)
                # used_arcs.append((current_node,current_time, 0,arrival_actual))
        # end building route
    # end construction over all routes

    # NOW, if there are unvisited nodes, construct expensive dummy arcs from depot
    # and record these dummy routes
    for n in unvisited_indices:
        # Since we are adding arcs, we need to re-construct/enumerate stuff
        self.variables_enumerated = False
        self.constraints_built = False
        self.objective_built = False

        arc = (0, n)
        depot_nm = self.node_names[0]
        node_nm = self.node_names[n]
        assert not self.check_arc(arc), \
            f"We should have been able to construct a route through node {node_nm}"
        logger.info("Adding entry arc to %s", node_nm)
        added = self.add_arc(depot_nm, node_nm, 0, high_cost)
        assert added, "Something is wrong in construction heuristic"
        current_time = self.time_points[0]
        arrival, in_tp = self.get_arrival_time(current_time, arc)
        # Since the arc we added has zero travel time,
        # arrival time should be in time_points already...
        assert in_tp, f"Arriving at {arrival}: not in time_points??"
        used_arcs.append((0, current_time, n, arrival))
        current_time = arrival

        # Now, exit back to depot
        self.check_and_add_exit_arc(n, high_cost)
        arc = (n, 0)
        arrival, in_tp = self.get_arrival_time(current_time, arc)
        assert in_tp, f"Arriving at {arrival}: not in time_points??"
        used_arcs.append((n, current_time, 0, arrival))
    # done fixing

    # construct and save feasible solution
    self.enumerate_variables()
    self.feasible_solution = np.zeros(self.num_variables)
    for a in used_arcs:
        self.feasible_solution[self.get_var_index(*a)] = 1
    return


def extract_routes(best_sample, data, points):
    """
    Trích xuất hành trình của từng xe từ nghiệm best_sample.

    Args:
        best_sample (dict): Nghiệm tốt nhất từ QUBO solver, chứa các biến x_{depot_id}_{v}_{i}_{j}.
        data (dict): Dữ liệu đầu vào bao gồm thông tin depot, khách hàng, và xe.
        points (list): Danh sách tất cả các điểm (depot + khách hàng).

    Returns:
        dict: Dictionary với key là (depot_id, vehicle_id) và value là danh sách các điểm trong hành trình.
    """
    routes = {}
    depots = data['depots']
    vehicles_per_depot = data['vehicles']['vehicles_per_depot']

    # Duyệt qua từng depot và xe
    for depot in depots:
        depot_id = depot['id']
        for v in range(vehicles_per_depot):
            # Thu thập các cạnh được chọn (x = 1) cho xe v từ depot này
            edges = []
            for var_name in best_sample:
                if (var_name.startswith(f'x_{depot_id}_{v}_') and
                        best_sample[var_name] == 1):
                    parts = var_name.split('_')
                    i, j = int(parts[3]), int(parts[4])
                    edges.append((i, j))

            # Xây dựng hành trình từ depot
            route = [depot_id]
            current = depot_id
            visited = {depot_id}  # Theo dõi các điểm đã ghé thăm để tránh lặp

            while edges:
                next_point = None
                for edge in edges:
                    if edge[0] == current and edge[1] not in visited:
                        next_point = edge[1]
                        edges.remove(edge)
                        break
                if next_point is None:
                    break  # Không còn cạnh nào hợp lệ
                route.append(next_point)
                visited.add(next_point)
                current = next_point

            # Đảm bảo hành trình kết thúc tại depot nếu có thể
            if current != depot_id and any(edge[0] == current and edge[1] == depot_id for edge in edges):
                route.append(depot_id)
            elif current != depot_id:
                route.append(depot_id)  # Thêm depot nếu hành trình chưa khép kín

            # Lưu hành trình vào dictionary
            routes[(depot_id, v)] = route

    return routes