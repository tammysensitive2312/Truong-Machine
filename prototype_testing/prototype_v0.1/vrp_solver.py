from dimod import Binary, quicksum
import numpy as np
from dwave.samplers import SimulatedAnnealingSampler
import time, json, sys, h5py, vrp_log as log
from scipy.sparse import csr_matrix


def load_input_data(json_path):
    data_object = {}

    """Đọc dữ liệu đầu vào từ file JSON theo định dạng pragmatic"""
    with open(json_path, 'r') as f:
        data = json.load(f)

    # Xử lý cấu trúc dữ liệu
    depot = data['depot']
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
    data_object['depot'] = depot
    data_object['customers'] = customers
    data_object['vehicles'] = vehicles
    data_object['distance_matrix'] = distance_matrix
    # data_object['time_matrix'] = time_matrix

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


def build_qubo(data, A):
    """Xây dựng mô hình QUBO từ dữ liệu đầu vào"""
    start_time = time.time()

    # Trích xuất dữ liệu
    depot_id = data['depot']['id']
    customers = data['customers']['id']
    vehicles_count = data['vehicles']['count']
    capacity = data['vehicles']['capacity']
    distance_matrix = data['distance_matrix']
    # time_matrix = data['time_matrix']

    # Tạo danh sách điểm
    points = [depot_id] + [c['id'] for c in customers]

    # Khởi tạo biến QUBO
    x = {
        (v, i, j): Binary(f'x_{v}_{i}_{j}')
        for v in range(vehicles_count)
        for i in points
        for j in points
        if i != j
    }

    # Hàm mục tiêu: Tổng quãng đường
    H_obj = quicksum(
        distance_matrix[(i, j)] * x[v, i, j]
        for v in range(vehicles_count)
        for i in points
        for j in points
        if i != j
    )

    # Ràng buộc 1: Mỗi khách hàng được phục vụ đúng 1 lần
    # H_visit = A * quicksum(
    #     (1 - quicksum(
    #         x[v, j, i]
    #         for v in range(vehicles_count)
    #         for j in points
    #         if j != i)) ** 2
    #     for i in points if i != depot_id
    # )

    # Ràng buộc 2: Xe xuất phát và trở về depot
    H_depot = A * quicksum(
        (1 - quicksum(x[v, depot_id, j] for j in points if j != depot_id)) ** 2 +
        (1 - quicksum(x[v, i, depot_id] for i in points if i != depot_id)) ** 2
        for v in range(vehicles_count)
    )

    # Ràng buộc 3: Công suất xe
    # demand = {c['id']: c['demand'] for c in customers}
    # H_capacity = A * quicksum(
    #     (quicksum(demand[j] * x[v, i, j] for i in points for j in points if j != depot_id and i != j) - capacity) ** 2
    #     for v in range(vehicles_count)
    # )

    # Tổng hợp QUBO
    QUBO = H_obj + H_depot

    print(f"Xây dựng QUBO trong {time.time() - start_time:.2f} giây")
    return QUBO, x, points


def solve_qubo(QUBO):
    """Giải bài toán QUBO và trả về nghiệm"""
    start_time = time.time()

    qubo_dict, offset = QUBO.to_qubo()
    sampler = SimulatedAnnealingSampler()
    response = sampler.sample_qubo(qubo_dict)

    # Lấy nghiệm tốt nhất và chuyển thành vector/mảng
    best_sample = response.first.sample
    energy = response.first.energy + offset

    # Trích xuất thông tin biến từ nghiệm
    var_names = sorted(best_sample.keys())
    solution_vector = [best_sample[var] for var in var_names]

    print(f"Giải QUBO trong {time.time() - start_time:.2f} giây")
    return best_sample, energy, solution_vector, var_names


def check_constraints(best_sample, data, points):
    """Kiểm tra tính hợp lệ của nghiệm"""
    start_time = time.time()
    valid = True
    depot_id = data['depot']['id']
    customers = [c['id'] for c in data['customers']]
    vehicles_count = data['vehicles']['count']
    capacity = data['vehicles']['capacity']
    demand = {c['id']: c['demand'] for c in data['customers']}

    # Kiểm tra ràng buộc 1
    # for c in customers:
    #     count = sum(best_sample[f'x_{v}_{i}_{c}']
    #                 for v in range(vehicles_count)
    #                 for i in points if i != c)
    #     if count != 1:
    #         print(f"Khách hàng {c} được phục vụ {count} lần!")
    #         valid = False

    # Kiểm tra ràng buộc 2
    for v in range(vehicles_count):
        start = sum(best_sample[f'x_{v}_{depot_id}_{j}'] for j in points if j != depot_id)
        end = sum(best_sample[f'x_{v}_{i}_{depot_id}'] for i in points if i != depot_id)
        if start != 1 or end != 1:
            print(f"Xe {v} vi phạm ràng buộc depot!")
            valid = False
            break

    # Kiểm tra ràng buộc 3
    # for v in range(vehicles_count):
    #     total = sum(demand[j] * best_sample[f'x_{v}_{i}_{j}']
    #                 for i in points
    #                 for j in customers
    #                 if i != j)
    #     if total > capacity:
    #         print(f"Xe {v} vượt công suất: {total}/{capacity}")
    #         valid = False

    print(f"Kiểm tra ràng buộc trong {time.time() - start_time:.2f} giây")
    return valid


def main(input_file, output_file):
    """Hàm chính thực thi toàn bộ quy trình"""
    # Khởi tạo logger
    logger = log.VRPLogger()
    start_time = time.time()

    # Đọc dữ liệu đầu vào
    data = load_input_data(input_file)
    logger.log_input_data(data)
    params = data['parameters']

    A = params['initial_penalty']
    best_sample = None
    best_energy = float('inf')
    best_vector = None
    best_var_names = None

    while time.time() - start_time < params['max_time']:
        # Xây dựng QUBO
        QUBO, x, points = build_qubo(data, A)
        logger.log_qubo_build(time.time() - start_time, len(x))

        # Lưu ma trận QUBO
        qubo_matrix = np.zeros((len(x), len(x)))
        for (var1, var2), coeff in QUBO.to_qubo()[0].items():
            i = int(var1.split('_')[-1])
            j = int(var2.split('_')[-1])
            qubo_matrix[i, j] += coeff
        save_qubo_hdf5(qubo_matrix, output_file)

        # Giải QUBO
        sample, energy, solution_vector, var_names = solve_qubo(QUBO)

        # Kiểm tra ràng buộc
        constraints_satisfied = check_constraints(sample, data, points)
        # logger.log_iteration(A, energy, sample, constraints_satisfied)

        if constraints_satisfied:
            best_sample = sample
            best_energy = energy
            best_vector = solution_vector
            best_var_names = var_names
            break
        else:
            if energy < best_energy:
                best_sample = sample
                best_energy = energy
                best_vector = solution_vector
                best_var_names = var_names
            A += params['penalty_increment']

    # Xuất kết quả và ghi log
    logger.log_solution(best_sample, best_energy, best_var_names, points, data)

    return best_sample, best_energy, best_vector


if __name__ == "__main__":
    if len(sys.argv) != 3:
        print("Usage: python vrp_solver.py <input.json> <output.h5>")
        sys.exit(1)

    main(sys.argv[1], sys.argv[2])