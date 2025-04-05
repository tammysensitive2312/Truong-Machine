import h5py
import json
import sys
import time
import vrp_log as log

import numpy as np
from dimod import Binary, quicksum
from dwave.samplers import SimulatedAnnealingSampler
from scipy.sparse import csr_matrix, lil_matrix


def load_input_data(json_path):
    data_object = {}

    """Đọc dữ liệu đầu vào từ file JSON theo định dạng pragmatic"""
    with open(json_path, 'r') as f:
        data = json.load(f)

    # Xử lý cấu trúc dữ liệu
    depots = data['depots']
    customers = sorted(data['customers'], key=lambda x: x['id'])
    vehicles = data['vehicles']

    # Chuyển đổi ma trận dạng list of lists thành dictionary
    distance_matrix = {}
    time_matrix = {}
    for i, row in enumerate(data['distance_matrix']):
        for j, val in enumerate(row):
            distance_matrix[(i, j)] = val
    # for i, row in enumerate(data['time_matrix']):
    #     for j, val in enumerate(row):
    #         time_matrix[(i, j)] = val

    data_object['parameters'] = data['parameters']
    data_object['depots'] = depots
    data_object['customers'] = customers
    data_object['vehicles'] = vehicles
    data_object['distance_matrix'] = distance_matrix
    # data_object['time_matrix'] = time_matrix
    data_object['sa_params'] = data['sa_params']

    print("Dữ liệu đầu vào:")
    print(data_object)
    return data_object


def save_qubo_hdf5(qubo_matrix, filename):
    """Lưu ma trận QUBO dạng CSR vào file HDF5"""
    sparse_qubo = csr_matrix(qubo_matrix)
    with h5py.File(filename, 'w') as f:
        f.create_dataset('data', data=sparse_qubo.data)
        f.create_dataset('indices', data=sparse_qubo.indices)
        f.create_dataset('indptr', data=sparse_qubo.indptr)
        f.create_dataset('shape', data=sparse_qubo.shape)


def build_qubo(data, penalty_ratio=1e5):
    """Xây dựng mô hình QUBO tập trung vào ràng buộc trọng tải và depot"""
    start_time = time.time()

    # Trích xuất dữ liệu
    depots = data['depots']
    customers = data['customers']
    vehicles_per_depot = data['vehicles']['vehicles_per_depot']
    capacity = data['vehicles']['capacity']
    distance_matrix = data['distance_matrix']

    depot_ids = [d['id'] for d in depots]
    customer_ids = [c['id'] for c in customers]

    # Tạo danh sách điểm và id của khách hàng
    all_points = depot_ids + customer_ids
    demand = {c['id']: c['demand'] for c in customers}  # Dictionary ánh xạ id -> demand

    valid_edges = []

    for d_id in depot_ids:
        valid_edges += [(d_id, c_id) for c_id in customer_ids]
        valid_edges += [(c_id, d_id) for c_id in customer_ids]

    # Customer -> Customer
    valid_edges += [
        (c1['id'], c2['id'])
        for c1 in customers
        for c2 in customers
        if c1 != c2
    ]

    # Khởi tạo biến QUBO
    x = {}
    var_index = {}  # Ánh xạ (depot_id, v, i, j) -> index
    idx = 0

    for depot_id in depot_ids:
        for v in range(vehicles_per_depot):
            for (i, j) in valid_edges:
                if i == j:
                    continue
                x_key = (depot_id, v, i, j)
                x[x_key] = Binary(f"x_{depot_id}_{v}_{i}_{j}")
                var_index[x_key] = idx
                idx += 1
    num_vars = len(x)
    print(f"Số biến QUBO: {num_vars}")

    # Hàm mục tiêu: Tổng quãng đường

    constraints = {
        'depot': {'coeffs': [], 'rows': [], 'cols': []},
        'capacity': {'coeffs': [], 'rows': [], 'cols': []},
        'customer': {'coeffs': [], 'rows': [], 'cols': []},
        'flow': {'coeffs': [], 'rows': [], 'cols': []}
    }

    # Ràng buộc 1: Xe xuất phát và trở về depot
    for depot_id in depot_ids:
        for v in range(vehicles_per_depot):
            # Xuất phát từ depot
            out_flow = quicksum(
                x[(depot_id, v, depot_id, j)]
                for j in customer_ids if (depot_id, j) in valid_edges
            )
            # Kết thúc tại depot
            in_flow = quicksum(
                x[(depot_id, v, i, depot_id)]
                for i in customer_ids if (i, depot_id) in valid_edges
            )
            constraint = (1 - out_flow) ** 2 + (1 - in_flow) ** 2
            # Chuyển đổi thành hệ số QUBO
            for u, v_var, coeff in constraint.iter_quadratic():
                if u in var_index and v_var in var_index:
                    constraints['depot']['coeffs'].append(coeff * penalty_ratio)
                    constraints['depot']['rows'].append(var_index[u])
                    constraints['depot']['cols'].append(var_index[v_var])
                else:
                    print(f"Cảnh báo: Biến {u} hoặc {v_var} không tồn tại. Bỏ qua.")

    # Ràng buộc 2: Công suất xe
    for depot_id in depot_ids:
        for v in range(vehicles_per_depot):
            total_demand = quicksum(
                demand[j] * x[(depot_id, v, i, j)]
                for (i, j) in valid_edges if j in customer_ids
            )
            constraint = (total_demand - capacity) ** 2
            for u, v_var, coeff in constraint.iter_quadratic():
                if u in var_index and v_var in var_index:
                    constraints['depot']['coeffs'].append(coeff * penalty_ratio)
                    constraints['depot']['rows'].append(var_index[u])
                    constraints['depot']['cols'].append(var_index[v_var])
                else:
                    print(f"Cảnh báo: Biến {u} hoặc {v_var} không tồn tại. Bỏ qua.")

    # Ràng buộc 3: Mỗi khách hàng được phục vụ đúng 1 lần
    for j in customer_ids:
        service_count = quicksum(
            x[(depot_id, v, i, j)]
            for depot_id in depot_ids
            for v in range(vehicles_per_depot)
            for i in all_points if (i, j) in valid_edges
        )
        constraint = (1 - service_count) ** 2
        for u, v_var, coeff in constraint.iter_quadratic():
            if u in var_index and v_var in var_index:
                constraints['depot']['coeffs'].append(coeff * penalty_ratio)
                constraints['depot']['rows'].append(var_index[u])
                constraints['depot']['cols'].append(var_index[v_var])
            else:
                print(f"Cảnh báo: Biến {u} hoặc {v_var} không tồn tại. Bỏ qua.")

    # Ràng buộc 4: Tính liên tục của hành trình
    for node in customer_ids:
        for depot_id in depot_ids:
            for v in range(vehicles_per_depot):
                inflow = quicksum(
                    x[(depot_id, v, i, node)]
                    for i in all_points if (i, node) in valid_edges
                )
                outflow = quicksum(
                    x[(depot_id, v, node, j)]
                    for j in all_points if (node, j) in valid_edges
                )
                constraint = (inflow - outflow) ** 2
                for u, v_var, coeff in constraint.iter_quadratic():
                    if u in var_index and v_var in var_index:
                        constraints['depot']['coeffs'].append(coeff * penalty_ratio)
                        constraints['depot']['rows'].append(var_index[u])
                        constraints['depot']['cols'].append(var_index[v_var])
                    else:
                        print(f"Cảnh báo: Biến {u} hoặc {v_var} không tồn tại. Bỏ qua.")

    # Tổng hợp QUBO
    QUBO = lil_matrix((num_vars, num_vars))

    # Thêm hàm mục tiêu
    for (depot_id, v, i, j) in x:
        idx = var_index[(depot_id, v, i, j)]
        QUBO[idx, idx] += distance_matrix[(i, j)]

    # Thêm các ràng buộc
    for ctype in constraints:
        for coeff, row, col in zip(
            constraints[ctype]['coeffs'],
            constraints[ctype]['rows'],
            constraints[ctype]['cols']
        ):
            QUBO[row, col] += coeff

    print(f"Xây dựng QUBO trong {time.time() - start_time:.2f} giây")
    return QUBO.tocoo(), x, var_index


def two_opt_optimize(routes, distance_matrix):
    optimized_routes = {}
    for (depot_id, v), route in routes.items():
        best_route = route
        best_cost = sum(distance_matrix[(best_route[i], best_route[i+1])]
                       for i in range(len(best_route)-1))
        improved = True
        while improved:
            improved = False
            for i in range(1, len(route)-2):
                for j in range(i+1, len(route)-1):
                    new_route = route[:i] + route[i:j+1][::-1] + route[j+1:]
                    new_cost = sum(distance_matrix[(new_route[k], new_route[k+1])]
                                  for k in range(len(new_route)-1))
                    if new_cost < best_cost:
                        best_route = new_route
                        best_cost = new_cost
                        improved = True
        optimized_routes[(depot_id, v)] = best_route
    return optimized_routes


def solve_qubo(Q, var_index, initial_solution, sa_params, offset=0):
    """Giải QUBO với nghiệm ban đầu và trả về nghiệm tối ưu."""
    start_time = time.time()

    # Khởi tạo sampler và chuyển đổi nghiệm ban đầu
    sampler = SimulatedAnnealingSampler()
    initial_state = {var: initial_solution[idx] for var, idx in var_index.items()}

    # Gọi solver QUBO với tham số
    response = sampler.sample_qubo(
        Q,
        initial_state=initial_state,
        num_reads=sa_params['num_reads'],
        num_sweeps=sa_params['num_sweeps'],
        beta_range=sa_params['beta_range'],
    )

    # Xử lý kết quả
    best_sample = response.first.sample
    energy = response.first.energy + offset  # Thêm offset (nếu có)

    # Chuyển đổi nghiệm thành vector
    var_names = sorted(best_sample.keys())
    solution_vector = [best_sample[var] for var in var_names]

    print(f"Giải QUBO trong {time.time() - start_time:.2f} giây")
    return best_sample, energy, solution_vector, var_names


def extract_routes(best_sample, data):
    """
    Trích xuất hành trình của từng xe từ nghiệm best_sample, áp dụng kỹ thuật từ arc_based_rp.
    """
    routes = {}
    depots = data['depots']
    vehicles_per_depot = data['vehicles']['vehicles_per_depot']
    customer_ids = [c['id'] for c in data['customers']]
    depot_ids = [d['id'] for d in depots]

    # Duyệt qua từng depot và xe
    for depot in depots:
        depot_id = depot['id']
        for v in range(vehicles_per_depot):
            # Thu thập các cạnh được chọn (x = 1) cho xe v từ depot này
            edges = []
            for var_name, value in best_sample.items():
                if value == 1 and var_name.startswith(f'x_{depot_id}_{v}_'):
                    parts = var_name.split('_')
                    i, j = int(parts[3]), int(parts[4])
                    edges.append((i, j))

            # Xây dựng edge mapping để theo dõi node tiếp theo
            edge_map = {}
            for (i, j) in edges:
                edge_map[i] = j

            # Xây dựng route từ depot
            route = [depot_id]
            current_node = depot_id
            visited = set([current_node])

            while current_node in edge_map:
                next_node = edge_map[current_node]
                # Kiểm tra node hợp lệ và tránh lặp
                if next_node in visited or next_node not in (customer_ids + [depot_id]):
                    break
                route.append(next_node)
                visited.add(next_node)
                current_node = next_node

            # Đảm bảo kết thúc tại depot
            if route[-1] != depot_id:
                route.append(depot_id)

            # Kiểm tra tính hợp lệ của route
            if not is_valid_route(route, depot_id, customer_ids):
                route = [depot_id, depot_id]  # Đánh dấu route không hợp lệ

            routes[(depot_id, v)] = route

    # Xử lý khách hàng chưa được phục vụ bằng dummy routes
    all_served = set()
    for route in routes.values():
        all_served.update(route[1:-1])  # Bỏ qua depot ở đầu và cuối

    unserved_customers = set(customer_ids) - all_served
    for c in unserved_customers:
        depot_id = depots[0]['id']
        vehicle_id = len(routes)
        routes[(depot_id, vehicle_id)] = [depot_id, c, depot_id]

    return routes


def is_valid_route(route, depot_id, customer_ids):
    """
    Kiểm tra tính hợp lệ của route:
    - Bắt đầu và kết thúc tại depot.
    - Mỗi customer chỉ xuất hiện một lần.
    """
    if len(route) < 2 or route[0] != depot_id or route[-1] != depot_id:
        return False
    for node in route[1:-1]:
        if node not in customer_ids or route.count(node) > 1:
            return False
    return True


def check_constraints(best_sample, data, points):
    """Kiểm tra tính hợp lệ của nghiệm với các ràng buộc mở rộng"""
    start_time = time.time()
    valid = True

    # Trích xuất hành trình (KHÔNG tối ưu hóa)
    routes = extract_routes(best_sample, data)

    # Trích xuất thông tin từ dữ liệu
    depots = data['depots']
    customers = [c['id'] for c in data['customers']]
    vehicles_per_depot = data['vehicles']['vehicles_per_depot']
    capacity = data['vehicles']['capacity']
    demand = {c['id']: c['demand'] for c in data['customers']}

    # Ràng buộc 1: Xe xuất phát và kết thúc tại depot
    for depot in depots:
        depot_id = depot['id']
        for v in range(vehicles_per_depot):
            route = routes.get((depot_id, v), [])
            if not route or route[0] != depot_id or route[-1] != depot_id:
                print(f"Xe {v} từ depot {depot_id} không xuất phát/kết thúc tại depot!")
                valid = False

    # Ràng buộc 2: Mỗi khách hàng được phục vụ đúng một lần
    served_customers = set()
    for route in routes.values():
        for point in route[1:-1]:  # Bỏ depot ở đầu và cuối
            if point in served_customers:
                print(f"Khách hàng {point} được phục vụ nhiều lần!")
                valid = False
            if point in customers:
                served_customers.add(point)
    if set(customers) != served_customers:
        print("Không phải tất cả khách hàng đều được phục vụ!")
        valid = False

    # Ràng buộc 3: Kiểm tra công suất
    for (depot_id, v), route in routes.items():
        total_demand = sum(demand.get(point, 0) for point in route[1:-1])
        if total_demand > capacity:
            print(f"Xe {v} từ depot {depot_id} vượt công suất: {total_demand}/{capacity}")
            valid = False

    print(f"Kiểm tra ràng buộc trong {time.time() - start_time:.2f} giây")
    return valid


def generate_initial_routes(data):
    depots = data['depots']
    customers = data['customers']
    vehicles_per_depot = data['vehicles']['vehicles_per_depot']
    capacity = data['vehicles']['capacity']
    distance_matrix = data['distance_matrix']

    routes = {}
    unvisited = [c['id'] for c in customers]

    for depot in depots:
        depot_id = depot['id']
        for v in range(vehicles_per_depot):
            current_load = 0
            route = [depot_id]
            current_node  = depot_id

            while unvisited and current_load < capacity:
                # Tìm khách hàng gần nhất chưa được thăm và đủ công suất
                nearest = None
                min_dist = float('inf')
                for c in unvisited:
                    if (current_node, c) in distance_matrix:
                        dist = distance_matrix[(current_node, c)]
                        demand = next(cust['demand'] for cust in customers if cust['id'] == c)

                        if dist < min_dist and current_load + demand <= capacity:
                            nearest = c
                            min_dist = dist

                if nearest is None:
                    break

                route.append(nearest)
                current_load += next(cust['demand'] for cust in customers if cust['id'] == nearest)
                unvisited.remove(nearest)
                current_node = nearest

            route.append(depot_id)
            routes[(depot_id, v)] = route

    for c in unvisited:
        depot_id = depots[0]['id']  # Giả sử lấy depot đầu tiên
        # Kiểm tra tồn tại cung (depot_id, c) và (c, depot_id)
        if (depot_id, c) not in distance_matrix:
            # Thêm dummy arc với chi phí cao (ví dụ: 1e6)
            distance_matrix[(depot_id, c)] = 1e6
        if (c, depot_id) not in distance_matrix:
            distance_matrix[(c, depot_id)] = 1e6

        # Tạo route mới (giả sử có đủ xe)
        vehicle_id = len(routes)
        if vehicle_id >= vehicles_per_depot * len(depots):
            raise ValueError("Không đủ xe để phục vụ tất cả khách hàng")
        routes[(depot_id, vehicle_id)] = [depot_id, c, depot_id]

    return routes


def routes_to_qubo_solution(routes, var_index):
    """Chuyển đổi routes thành vector nghiệm QUBO."""
    if var_index is None:
        raise ValueError("var_index không được là None. Hãy gọi build_qubo trước.")

    solution = np.zeros(len(var_index), dtype=int)

    for (depot_id, v), route in routes.items():
        for i in range(len(route) - 1):
            from_node = route[i]
            to_node = route[i + 1]
            key = (depot_id, v, from_node, to_node)
            if key in var_index:
                solution[var_index[key]] = 1
            else:
                # Xử lý trường hợp biến không tồn tại (có thể log cảnh báo)
                print(f"Cảnh báo: Biến {key} không tồn tại trong var_index. Bỏ qua.")

    return solution


def main(input_file, output_file):
    """Hàm chính thực thi toàn bộ quy trình giải VRP"""
    # Khởi tạo logger và đọc dữ liệu
    logger = log.VRPLogger()
    start_time = time.time()
    data = load_input_data(input_file)
    logger.log_input_data(data)
    params = data['parameters']

    # Khởi tạo tham số
    A = params['initial_penalty']
    best_sample = None
    best_energy = float('inf')
    sa_params = data['sa_params']

    QUBO, x, var_index = build_qubo(data, A)

    # Tạo initial routes để khởi tạo nghiệm ban đầu
    initial_routes = generate_initial_routes(data)
    initial_solution = routes_to_qubo_solution(initial_routes, var_index)

    while time.time() - start_time < params['max_time']:
        # Xây dựng QUBO với hệ số phạt hiện tại
        QUBO, x, var_index = build_qubo(data, A)

        # Lưu ma trận QUBO (tùy chọn)
        qubo_matrix = np.zeros((len(x), len(x)))
        for (var1, var2), coeff in QUBO.to_qubo()[0].items():
            i = int(var1.split('_')[-1])
            j = int(var2.split('_')[-1])
            qubo_matrix[i, j] += coeff
        save_qubo_hdf5(qubo_matrix, output_file)

        # Giải QUBO với nghiệm ban đầu từ initial routes
        sample, energy, solution_vector, var_names = solve_qubo(
            QUBO,
            var_index,
            initial_solution,  # Sử dụng initial solution
            sa_params
        )

        # Kiểm tra ràng buộc (KHÔNG áp dụng two_opt_optimize)
        constraints_satisfied = check_constraints(sample, data, var_index)

        # Cập nhật nghiệm tốt nhất và điều chỉnh hệ số phạt
        if constraints_satisfied:
            # Giảm hệ số phạt nếu nghiệm hợp lệ
            A = max(A * 0.8, params['initial_penalty'])
            best_sample = sample
            best_energy = energy
            break  # Dừng nếu tìm thấy nghiệm hợp lệ
        else:
            # Tăng hệ số phạt nếu nghiệm không hợp lệ
            if energy < best_energy:
                best_sample = sample
                best_energy = energy
            A *= 1.5
            A += params['penalty_increment']

    # Tối ưu hóa routes bằng 2-opt sau khi có nghiệm hợp lệ
    if best_sample is not None:
        routes = extract_routes(best_sample, data)
        optimized_routes = two_opt_optimize(routes, data['distance_matrix'])
        logger.log_solution(optimized_routes, best_energy, var_names, var_index, data)
    else:
        print("Không tìm thấy nghiệm hợp lệ!")

    return best_sample, best_energy, optimized_routes if best_sample else None


if __name__ == "__main__":
    if len(sys.argv) != 3:
        print("Usage: python vrp_solver.py <input.json> <output.h5>")
        sys.exit(1)

    main(sys.argv[1], sys.argv[2])