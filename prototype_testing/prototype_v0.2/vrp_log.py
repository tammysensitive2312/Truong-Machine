import logging
import time
import os
import json
from datetime import datetime
import matplotlib.pyplot as plt


class VRPLogger:
    def __init__(self, log_file="output/vrp_solver.log", log_level=logging.INFO, console_output=True):
        """Khởi tạo logger cho bài toán VRP"""
        self.start_time = time.time()

        # Cấu hình logging
        self.logger = logging.getLogger("VRPSolver")
        self.logger.setLevel(log_level)

        # Tạo file handler
        file_handler = logging.FileHandler(log_file, encoding='utf-8')
        file_formatter = logging.Formatter('%(asctime)s - %(levelname)s - %(message)s')
        file_handler.setFormatter(file_formatter)
        self.logger.addHandler(file_handler)

        self.log_file = log_file
        self.iteration = 0
        self.best_energy = float('inf')

    def log_input_data(self, data):
        """Ghi thông tin dữ liệu đầu vào"""
        self.logger.info(f"Số điểm: {len(data['customers']) + 1}")
        self.logger.info(f"Số xe: {data['vehicles']['vehicles_per_depot']}")
        self.logger.info(f"Số khách hàng: {len(data['customers'])}")
        self.logger.info(f"Tải trọng xe: {data['vehicles']['capacity']}")
        self.logger.info(f"Tham số giải thuật: {json.dumps(data['parameters'], indent=2)}")

    def log_qubo_build(self, duration, variables_count):
        """Ghi thông tin xây dựng QUBO"""
        self.logger.info(f"=== Xây dựng QUBO ===")
        self.logger.info(f"Thời gian xây dựng: {duration:.4f} giây")
        self.logger.info(f"Số biến QUBO: {variables_count}")

    def log_iteration(self, penalty, energy, solution, constraints_satisfied):
        """Ghi thông tin mỗi lần lặp"""
        self.iteration += 1
        self.logger.info(f"\n=== Lần lặp {self.iteration} ===")
        self.logger.info(f"Hệ số phạt: {penalty}")
        self.logger.info(f"Năng lượng: {energy}")
        self.logger.info(f"Thỏa mãn ràng buộc: {constraints_satisfied}")

        if energy < self.best_energy:
            self.best_energy = energy
            self.logger.info("Tìm thấy nghiệm tốt hơn!")

    def log_solution(self, sample, energy, var_names, points, data):
        """Ghi thông tin nghiệm cuối cùng cho bài toán nhiều depot"""
        self.logger.info("\n=== Kết quả cuối cùng ===")
        self.logger.info(f"Năng lượng tốt nhất: {energy}")

        # Chuyển đổi nghiệm thành lộ trình dễ đọc
        self.logger.info("\nLộ trình các xe:")
        vehicles_per_depot = data['vehicles']['vehicles_per_depot']
        depots = data['depots']

        # Theo dõi tổng thống kê
        total_route_distance = 0
        total_customers_served = 0

        # Duyệt qua từng depot
        for depot in depots:
            depot_id = depot['id']
            self.logger.info(f"\n--- Depot {depot_id} ---")

            # Duyệt qua từng xe của depot
            for v in range(vehicles_per_depot):
                # Tìm tất cả các cạnh được sử dụng bởi xe v tại depot này
                edges = []
                for var_idx, var_name in enumerate(var_names):
                    if var_name.startswith(f"x_{depot_id}_{v}_") and sample[var_name] == 1:
                        parts = var_name.split('_')
                        i, j = int(parts[3]), int(parts[4])
                        edges.append((i, j))

                # Xây dựng lộ trình
                route = [depot_id]
                current = depot_id
                total_distance = 0
                total_demand = 0
                vehicle_customers = 0

                while edges:
                    found = False
                    for edge in edges:
                        if edge[0] == current:
                            route.append(edge[1])
                            if edge[1] != depot_id:
                                # Tìm yêu cầu của khách hàng
                                for customer in data['customers']:
                                    if customer['id'] == edge[1]:
                                        total_demand += customer['demand']
                                        vehicle_customers += 1
                                        break

                            total_distance += data['distance_matrix'].get((edge[0], edge[1]), 0)
                            current = edge[1]
                            edges.remove(edge)
                            found = True
                            break

                    if not found:
                        break  # Không tìm thấy cạnh tiếp theo

                # Chỉ hiển thị thông tin của các xe có lộ trình
                if len(route) > 1:
                    self.logger.info(f"Xe {v} (Depot {depot_id}):")
                    self.logger.info(f"   Lộ trình: {route}")
                    self.logger.info(f"   Tổng quãng đường: {total_distance}")
                    self.logger.info(f"   Tổng yêu cầu: {total_demand}/{data['vehicles']['capacity']}")
                    self.logger.info(f"   Số khách hàng phục vụ: {vehicle_customers}")

                # Cập nhật thống kê tổng
                total_route_distance += total_distance
                total_customers_served += vehicle_customers

        # Thống kê tổng quát
        self.logger.info("\n=== Thống kê tổng quát ===")
        self.logger.info(f"Tổng quãng đường: {total_route_distance}")
        self.logger.info(f"Tổng khách hàng được phục vụ: {total_customers_served}")
        self.logger.info(f"Tổng số depot: {len(depots)}")
        self.logger.info(f"Xe mỗi depot: {vehicles_per_depot}")

        # Thời gian chạy
        elapsed_time = time.time() - self.start_time
        self.logger.info(f"\nTổng thời gian chạy: {elapsed_time:.2f} giây")

    def plot_convergence(self):
        plt.plot(self.energies)
        plt.title("Energy Convergence")
        plt.xlabel("Iteration")
        plt.ylabel("Energy")
        plt.savefig("convergence.png")