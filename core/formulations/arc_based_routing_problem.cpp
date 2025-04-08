//
// Created by ACER on 03/04/2025.
//

#include "../../header/arc_based_routing_problem.h"
#include "iostream"
#include "chrono"
#include "Eigen/Dense"

ArcBasedRoutingProblem::ArcBasedRoutingProblem(VehicleRoutingProblemWithTimeWindows* vrptw_ptr)
        : RoutingProblem(vrptw_ptr) {

}

int ArcBasedRoutingProblem::get_num_variables() {
    if (!variables_enumerated) {
        enumerate_variables();
    }
    return num_variables;
}

void ArcBasedRoutingProblem::make_feasible(float high_cost) {
    // Initialize list of unvisited node indices
    // remove depot
    std::vector<int> unvisited_indices;
    for (int i = 1; i < get_nodes().size(); ++i) {
        unvisited_indices.push_back(i);
    }

    std::cout << "Unvisited indices: ";
    for (int index : unvisited_indices) {
        std::cout << index << " ";
    }
    std::cout << std::endl;

    std::cout<< "Starting to used arcs..." << std::endl;

    // starting from depot,
    // go through unvisited nodes,
    // greedily visit first unvisited node that we can (satisfying timing constraints)
    // repeat until outgoing arcs from depot are exhausted
    std::vector<std::tuple<int, float, int, float>> used_arcs;
    int max_vehicles = estimate_max_vehicles();
    std::cout << "Max vehicles: " << max_vehicles << std::endl;
    for (int vehicle = 0; vehicle < max_vehicles; ++vehicle) {
        std::cerr << "\n===== Processing vehicle " << vehicle + 1 << "/" << max_vehicles << " =====" << std::endl;

        // Bắt đầu từ depot
        int current_node = 0;
        float current_time = time_points.at(0); // Sử dụng at() để kiểm tra range
        bool building_route = true;
        std::cerr << "Start from depot (node 0), current_time = " << current_time << std::endl;

        while (building_route) {
            std::cerr << "\n-- Current node: " << current_node
                      << ", current time: " << current_time
                      << ", unvisited node: " << unvisited_indices.size()
                      << std::endl;

            // Tìm nút tiếp theo
            std::optional<int> best_node = std::nullopt;
            float best_arrival = std::numeric_limits<float>::infinity();

            // Debug: In danh sách các nút chưa thăm
            std::cerr << "unvisited node list: ";
            for (int n : unvisited_indices) std::cerr << n << " ";
            std::cerr << std::endl;

            for (int n : unvisited_indices) {
                std::pair<int, int> arc = {current_node, n};
                std::cerr << "  Investigate arc " << arc.first << "->" << arc.second;

                try {
                    // Kiểm tra arc tồn tại
                    if (!check_arc(arc)) {
                        std::cerr << " (Non-existent)" << std::endl;
                        continue;
                    }

                    // Lấy thời gian đến
                    auto [arrival_actual, in_tp] = get_arrival_time(current_time, get_arcs().at(arc)); // Sử dụng at() để kiểm tra key
                    std::cerr << ", arrival_actual = " << arrival_actual
                              << ", in_tp = " << std::boolalpha << in_tp;

                    // Kiểm tra time window
                    if (!in_tp) continue;
                    auto [t_w_start, t_w_end] = get_nodes().at(n)->get_window(); // Sử dụng at() để kiểm tra index
                    std::cerr << ", time window [" << t_w_start << ", " << t_w_end << "]";

                    if (arrival_actual <= t_w_end && arrival_actual < best_arrival) {
                        best_node = n;
                        best_arrival = arrival_actual;
                        std::cerr << " --> Update best_node!";
                    }
                    std::cerr << std::endl;

                } catch (const std::exception& e) {
                    std::cerr << "\nERROR WHILE PROCESS NODE " << n << ": " << e.what() << std::endl;
                    throw;
                }
            }

            if (best_node.has_value()) {
                int n = best_node.value();
                std::cerr << "Choose next node: " << n
                          << ", arrival_time = " << best_arrival
                          << std::endl;

                // Thêm arc vào lộ trình
                used_arcs.emplace_back(current_node, current_time, n, best_arrival);

                // Cập nhật trạng thái
                current_node = n;
                current_time = best_arrival;

                // Xóa nút khỏi danh sách chưa thăm
                auto it = std::find(unvisited_indices.begin(), unvisited_indices.end(), n);
                if (it != unvisited_indices.end()) {
                    unvisited_indices.erase(it);
                    std::cerr << "Deleted node " << n << " from unvisited node list" << std::endl;
                } else {
                    std::cerr << "Can't find node " << n << " from unvisited node list!" << std::endl;
                }

            } else {
                std::cerr << "Can't find next node acceptend!" << std::endl;
                building_route = false;

                // Xử lý quay về depot
                if (current_node != 0) {
                    std::pair<int, int> arc_back = {current_node, 0};
                    std::cerr << "Try back to depot from node : " << current_node << std::endl;

                    try {
                        if (!check_arc(arc_back)) {
                            throw std::runtime_error("There is no supply to the depot");
                        }

                        auto [arrival_actual, in_tp] = get_arrival_time(current_time, get_arcs().at(arc_back));
                        std::cerr << "Arrival time to depot: " << arrival_actual
                                  << ", in_tp = " << std::boolalpha << in_tp << std::endl;

                        if (in_tp) {
                            used_arcs.emplace_back(current_node, current_time, 0, arrival_actual);
                            std::cerr << "Added supply node to depot successfully" << std::endl;
                        } else {
                            throw std::runtime_error("The time to arrive at the depot is not included in the time window");
                        }

                    } catch (const std::exception& e) {
                        std::cerr << "ERROR WHEN RETURNING TO DEPOT: " << e.what() << std::endl;
                        throw std::runtime_error(
                                "Unable to complete the route for the vehicle " + std::to_string(vehicle) +
                                ". Reason: " + e.what()
                        );
                    }
                }
                std::cerr << "End of roadmap for vehicle " << vehicle << std::endl;
            }
        }
    }
    std::cout << "End of route for all vehicles." << std::endl;
    std::cout << "Used arcs: ";
    for (const auto& arc_tuple : used_arcs) {
        auto [i, s, j, t] = arc_tuple;
        std::cout << "(" << i << ", " << s << ", " << j << ", " << t << ") ";
    }

    // Thêm cung giả cho các nút chưa được thăm
    for (int n : unvisited_indices) {
        variables_enumerated = false;
        constraints_built = false;
        objective_built = false;

        std::pair<int, int> arc = {0, n};
        std::string depot_nm = get_node_names().at(0);
        std::string node_nm = get_node_names().at(n);
        if (!check_arc(arc)) {
            bool added = add_arc(depot_nm, node_nm, 0, high_cost);
            if (!added) {
                throw std::runtime_error("Không thể thêm cung giả đến nút " + node_nm);
            }
        }

        float current_time = time_points[0];
        auto [arrival, in_tp] = get_arrival_time(current_time, get_arcs().at(arc));
        if (!in_tp) {
            throw std::runtime_error("Thời gian đến không hợp lệ cho cung giả");
        }
        used_arcs.emplace_back(0, current_time, n, arrival);
        current_time = arrival;

        // Thêm cung về depot
        check_and_add_exit_arc(n, high_cost);
        std::pair<int, int> arc_back = {n, 0};
        auto [arrival_back, in_tp_back] = get_arrival_time(arrival, get_arcs().at(arc_back));
        if (!in_tp_back) {
            throw std::runtime_error("Thời gian quay về depot không hợp lệ");
        }
        used_arcs.emplace_back(n, arrival, 0, arrival_back);
    }

    // Xây dựng feasible_solution
    enumerate_variables();
    feasible_solution = Eigen::VectorXf::Zero(num_variables);
    for (const auto& arc_tuple : used_arcs) {
        auto [i, s, j, t] = arc_tuple;
        auto idx_opt = get_variable_index(i, s, j, t);
        if (idx_opt.has_value()) {
            feasible_solution[idx_opt.value()] = 1.0f;
        } else {
            throw std::runtime_error("Không tìm thấy biến trong variables_mapping");
        }
    }

}

std::tuple<Eigen::VectorXf, Eigen::SparseMatrix<float>>
ArcBasedRoutingProblem::get_objective_data() {
    /**
     * Return objective information in a consistent way
     * objective(x) = cᵀx + xᵀ Q x
     *
     * Return:
     *   c (Eigen::VectorXf): 1-d vector defining linear part of objective
     *   Q (Eigen::SparseMatrix<float>): 2-d matrix defining quadratic part of objective
     */
    build_objective(); // Gọi hàm xây dựng objective
    int n = get_num_variables();

    // Trả về tuple gồm objective và ma trận thưa rỗng kích thước n x n
    Eigen::SparseMatrix<float> Q(n, n); // Ma trận rỗng, mặc định tất cả phần tử bằng 0
    return std::make_tuple(objective, Q);
}

std::tuple<Eigen::SparseMatrix<float>, Eigen::VectorXf, Eigen::SparseMatrix<float>, float>
ArcBasedRoutingProblem::get_constraint_data() {
    /**
     * Return constraints in a consistent way
     * A_eq * x = b_eq
     * xᵀ * Q_eq * x = r_eq
     *
     * Return:
     *   A_eq (Eigen::SparseMatrix<float>): 2-d array of linear equality constraints
     *   b_eq (Eigen::VectorXf): 1-d array of right-hand side of equality constraints
     *   Q_eq (Eigen::SparseMatrix<float>): 2-d array of a single quadratic equality constraint
     *       (potentially all zeros if there are no nontrivial quadratic constraints)
     *   r_eq (float): Right-hand side of the quadratic constraint
     */
    build_constraints();

    Eigen::SparseMatrix<float> A_eq = constraints_matrix;
    Eigen::VectorXf b_eq = constraints_rhs;
    int n = get_num_variables();

    //giữ A_eq là sparse ngay cả khi b_eq rỗng
    Eigen::SparseMatrix<float> Q_eq(n, n);
    float r_eq = 0.0f;
    return std::make_tuple(A_eq, b_eq, Q_eq, r_eq);
}
/**
    Return a threshhold value of the penalty parameter that is sufficient
    for penalizing the constraints when constructing a QUBO representation of
    this problem

    @param feasibility (bool): Whether this is for a feasibility version of the
                problem. Sufficient penalty value can be zero
    @return sufficient_pp (float): Penalty parameter value
 */
float ArcBasedRoutingProblem::get_sufficient_penalty(bool feasibility) const {
    double sufficient_pp;
    if (feasibility) sufficient_pp = 0.0;
    else {
        double sum_arc_cost = 0.0;
        for (const auto& arc_pair : get_arcs()) {
            sum_arc_cost += std::abs(arc_pair.second->get_cost());
        }
        sufficient_pp = sum_arc_cost * time_points.size() * time_points.size();
    }
    return sufficient_pp;
}

std::unique_ptr<IloCplex> ArcBasedRoutingProblem::get_cplex_problem() {
    IloEnv* env = new IloEnv();
    IloModel model(*env);
    auto cplex = std::make_unique<IloCplex>(model);

    build_objective();
    build_constraints();

    std::unordered_map<float, int> t_index;
    for (size_t i = 0; i < time_points.size(); ++i) {
        t_index[time_points[i]] = static_cast<int>(i);
    }

    std::vector<std::string> names;
    names.reserve(variables_mapping.size());
    for (const auto& mapping : variables_mapping) {
        const auto& isjt = mapping.first;
        int i = std::get<0>(isjt);
        float s = std::get<1>(isjt);
        int j = std::get<2>(isjt);
        float t = std::get<3>(isjt);
        names.push_back("n" + std::to_string(i) + "t" + std::to_string(t_index[s]) +
                        "_n" + std::to_string(j) + "t" + std::to_string(t_index[t]));
    }

    IloNumVarArray vars(*env, objective.size(), 0, 1, ILOBOOL);
    for (size_t i = 0; i < names.size(); ++i) {
        vars[i].setName(names[i].c_str());
    }

    IloObjective obj = IloMinimize(*env);
    IloNumArray obj_coeffs(*env, objective.size());
    for (int i = 0; i < objective.size(); ++i) {
        obj_coeffs[i] = objective[i];
    }

    obj.setLinearCoefs(vars, obj_coeffs);
    model.add(obj);

    // Thêm ràng buộc đẳng thức
    IloRangeArray constraints(*env);
    IloNumArray rhs(*env, constraints_rhs.size());
    for (int i = 0; i < constraints_rhs.size(); ++i) {
        rhs[i] = constraints_rhs[i];
        constraints.add(IloRange(*env, constraints_rhs[i], constraints_rhs[i])); // Ràng buộc đẳng thức
        constraints[i].setName(constraint_names[i].c_str());
    }
    model.add(constraints);

    // Đặt hệ số cho ma trận ràng buộc
    for (int k = 0; k < constraints_matrix.outerSize(); ++k) {
        for (Eigen::SparseMatrix<float>::InnerIterator it(constraints_matrix, k); it; ++it) {
            int row = it.row();
            int col = it.col();
            float val = it.value();
            constraints[row].setLinearCoef(vars[col], val);
        }
    }

    return cplex;
}

//std::vector<double> ArcBasedRoutingProblem::solve_cplex_problem(const std::string &solutionFilename) const {
//    return {};
//}

void ArcBasedRoutingProblem::add_time_points(const std::vector<float>& new_time_points) {
    time_points = new_time_points;
    std::sort(time_points.begin(), time_points.end());
}

bool ArcBasedRoutingProblem::check_arc(const std::pair<int, int>& arc_key) {
    const auto& arcs = get_arcs();
    return arcs.find(arc_key) != arcs.end();
}

bool ArcBasedRoutingProblem::check_node_time_compat(int nodeIdx, float timePoint) const {
    auto t_w = get_nodes().at(nodeIdx)->get_window();
    return std::get<0>(t_w) <= timePoint && timePoint <= std::get<1>(t_w);
}

void ArcBasedRoutingProblem::enumerate_variables() {
    if (variables_enumerated) {
        return;
    }

    auto start = std::chrono::high_resolution_clock::now();
    enumerate_variables_quicker();
    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
    float seconds = duration.count() / 1000000.0f;
    std::cout << "Variable enumeration took " << seconds << " seconds" << std::endl;
}

void ArcBasedRoutingProblem::enumerate_variables_quicker() {
    if (variables_enumerated) return;

    auto& nodes = get_nodes();
    num_variables = 0;
    variables_mapping.clear();

    for (const auto& arcPair : get_arcs()) {
        int i = arcPair.first.first;
        int j = arcPair.first.second;
        float travelTime = arcPair.second->get_travel_time();

        auto [iStart, iEnd] = nodes.at(i)->get_window();

        for (float s : time_points) {
            if (s < iStart) continue;
            if (s > iEnd) break;

            auto [jStart, jEnd] = nodes.at(j)->get_window();
            for (float t : time_points) {
                if (t < jStart) continue;
                if (t > jEnd) break;
                if (s + travelTime > t) continue;

                // Biến (i, s, j, t) hợp lệ, thêm vào varMapping
                variables_mapping[std::make_tuple(i, s, j, t)] = num_variables++;
            }
        }
    }

    variables_enumerated = true;
}

const std::optional<int> ArcBasedRoutingProblem::get_variable_index(int node1Idx, float time1, int node2Idx,
                                                                    float time2) const{
    auto target = std::make_tuple(node1Idx, time1, node2Idx, time2);
    auto it = variables_mapping.find(target);
    if (it != variables_mapping.end()) {
        return it->second;
    }
    return std::nullopt;
}

const std::optional<std::tuple<int, float, int, float>>
ArcBasedRoutingProblem::get_variable_tuple_index(int variableIdx) const {
    for (const auto& [tuple, idx] : variables_mapping) {
        if (idx == variableIdx) {
            return tuple;
        }
    }
    return std::nullopt;
}

void ArcBasedRoutingProblem::build_objective() {
    if (objective_built) return;
    enumerate_variables();
    objective.resize(get_num_variables());

    for (const auto& [tuple, k] : variables_mapping) {
        auto [i, s, j, t] = tuple;
        objective[k] = get_arcs().at({i, j})->get_cost();
    }
    objective_built = true;
}

void ArcBasedRoutingProblem::build_constraints() {
    if (constraints_built) return;
    auto start = std::chrono::high_resolution_clock::now();
    build_constraints_quicker();
    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
    float seconds = duration.count() / 1000000.0f;
    std::cout << "Constraint construction took " << seconds << " seconds" << std::endl;
}

void ArcBasedRoutingProblem::build_constraints_quicker() {
    /**
     * Buidl up linear equality constraints of BLEC
     * A*x = b
     */

    enumerate_variables();

    std::vector<Eigen::Triplet<float>> triple_list;
    std::vector<float> brhs;
    std::vector<std::string> constraint_names;
    int row_index = 0;

    // Flow conservation constraints (for each (i,s))
    // EXCEPT DEPOT
    // see above- Depot is first in node list
    // First, index the non-trivial constraints
    std::vector<std::pair<int, float>> flow_conservation_mapping;
    for (int i = 1; i < get_nodes().size(); ++i) {
        for (int s_index = 0; s_index < time_points.size(); ++s_index) {
            float s = time_points[s_index];
            // Constraint:
            // sum_jt x_jtis - sum_jt x_isjt = 0

            // is s in the time window of i?
            // (if not, this is a vacuous constraint)
            auto [start, end] = get_nodes()[i] ->get_window();
            if (s < start) continue;
            // timePoints is sorted, so s will keep increasing in this loop
            // This condition will continue to be satisfied
            // (and so s NOT in time window)
            if (s > end) break;

            flow_conservation_mapping.emplace_back(i, s);
            brhs.push_back(0);
            constraint_names.push_back("cflow_" + std::to_string(i) + "," + std::to_string(s_index));
            row_index++;
        }
    }

    // NOW, go through variables
    // Note: each variable is an arc, and participates in (at most) TWO constraints:
    // once for INflow to a node, and once for OUTflow from a node
    for (const auto& [tuple, col] : variables_mapping) {
        auto [i, s, j, t] = tuple;

        auto out_flow_it = std::find(flow_conservation_mapping.begin(),
                                     flow_conservation_mapping.end(),
                                     std::make_pair(i, s));
        if (out_flow_it != flow_conservation_mapping.end()) {
            int row = std::distance(flow_conservation_mapping.begin(), out_flow_it);
            triple_list.emplace_back(row, col, -1.0f);
        }

        auto in_flow_it = std::find(flow_conservation_mapping.begin(),
                                    flow_conservation_mapping.end(),
                                    std::make_pair(j, t));
        if (in_flow_it != flow_conservation_mapping.end()) {
            int row = std::distance(flow_conservation_mapping.begin(), in_flow_it);
            triple_list.emplace_back(row, col, 1.0f);
        }
    }

    int num_nodes = static_cast<int>(get_nodes().size());
    for (int j = 1; j < num_nodes; ++j) {
        brhs.push_back(1);
        constraint_names.push_back("cnode" + std::to_string(j));
    }

    for (const auto& [tuple, col] : variables_mapping) {
        auto [i, s, j, t] = tuple;
        if (j == 0) continue;
        triple_list.emplace_back(row_index + (j - 1), col, 1.0f);
    }
    row_index += num_nodes -1;
    constraints_matrix.resize(row_index, get_num_variables());
    constraints_matrix.setFromTriplets(triple_list.begin(), triple_list.end());

    constraints_rhs = Eigen::VectorXf::Map(brhs.data(), brhs.size());

    this->constraint_names = constraint_names;
    constraints_built = true;
}

const std::tuple<float, bool> ArcBasedRoutingProblem::get_arrival_time(float departure_time, Arc* arc) const {
    /**
     * Get actual arrival time, leaving at <departure_time> on <arc>
     * @return
     *   arrival_time (float): Arrival time \n
     *   in_time_points (bool): whether this arrival time is in time_points set
     */
    // Recall: cannot arrive before time window
    auto t_w = arc->get_destination()->get_window(); // Lấy cửa sổ thời gian đích
    float arrival = std::max(std::get<0>(t_w), departure_time + arc->get_travel_time());

    // Tìm điểm thời gian nhỏ nhất >= arrival trong time_points
    auto it = std::lower_bound(time_points.begin(), time_points.end(), arrival);
    if (it == time_points.end()) {
        // Không có điểm thời gian nào >= arrival
        return std::make_tuple(arrival, false);
    }

    // Có điểm thời gian >= arrival
    float arrival_actual = *it;
    return std::make_tuple(arrival_actual, true);
}

const std::vector<float> &ArcBasedRoutingProblem::get_time_points() const {
    return time_points;
}
/**
  If exit arc from nodes[node_index] to depot does not exist, \n
  add it with zero travel time BUT cost of cost
 * @param n
 * @param cost
 */
void ArcBasedRoutingProblem::check_and_add_exit_arc(int node_index, float cost) {
    std::pair<int, int> arc = {node_index, 0};
    if (!check_arc(arc)) {
        std::string node_nm = get_node_names().at(node_index);
        std::string depot_nm = get_node_names().at(0);

        std::cout<< "Adding exit arc from " << node_nm << std::endl;
        bool added = add_arc(node_nm, depot_nm, 0, cost);
        if (!added) {
            std::string error_msg = "Something is wrong with exit arcs: (" +
                                    std::to_string(arc.first) + ", " +
                                    std::to_string(arc.second) + ") not added";
            throw std::runtime_error(error_msg);
        }
        variables_enumerated = false;
        constraints_built = false;
        objective_built = false;
    }
}

std::vector<std::vector<std::pair<int, float>>>
ArcBasedRoutingProblem::get_routes(const Eigen::VectorXf& solution) const {
    /**
     * Get a representation of the paths/ vehicle routes in a solution
     *
     * solution: binary vector corresponding to a solution
     */
    // Trích xuất các chỉ số của các cung được chọn (solution[i] > 0)
    std::vector<int> soln_var_indices;
    for (int i = 0; i < solution.size(); ++i) {
        if (solution[i] > 0.5f) { // Giả định nhị phân: 1 nếu > 0.5, 0 nếu nhỏ hơn
            soln_var_indices.push_back(i);
        }
    }

    // Chuyển chỉ số thành tuple (i, s, j, t)
    std::vector<std::tuple<int, float, int, float>> soln_var_tuples;
    for (int idx : soln_var_indices) {
        auto tuple_opt = get_variable_tuple_index(idx);
        if (tuple_opt) {
            soln_var_tuples.push_back(tuple_opt.value());
        } else {
            throw std::runtime_error("Invalid variable index in solution");
        }
    }

    // Sắp xếp lexicographically để các cung từ depot (nút 0) đứng đầu
    std::sort(soln_var_tuples.begin(), soln_var_tuples.end(),
              [](const auto& a, const auto& b) {
                  return std::tie(std::get<0>(a), std::get<1>(a), std::get<2>(a), std::get<3>(a)) <
                         std::tie(std::get<0>(b), std::get<1>(b), std::get<2>(b), std::get<3>(b));
              });

    // Xây dựng các tuyến đường và kiểm tra
    std::vector<float> visited(get_nodes().size(), 0.0f);
    std::vector<std::vector<std::pair<int, float>>> routes;

    while (!soln_var_tuples.empty()) {
        routes.emplace_back(); // Thêm tuyến mới
        auto arc = soln_var_tuples.front();
        soln_var_tuples.erase(soln_var_tuples.begin());
        bool route_finished = false;

        while (!route_finished) {
            routes.back().emplace_back(std::get<0>(arc), std::get<1>(arc)); // Thêm (i, s)
            std::pair<int, float> node_to_find{std::get<2>(arc), std::get<3>(arc)};
            visited[std::get<2>(arc)] += 1.0f;

            if (!check_node_time_compat(std::get<2>(arc), std::get<3>(arc))) {
                throw std::runtime_error("Node time window not satisfied");
            }

            bool node_found = false;
            for (size_t i = 0; i < soln_var_tuples.size(); ++i) {
                auto& a = soln_var_tuples[i];
                if (node_to_find.first == std::get<0>(a) && node_to_find.second == std::get<1>(a)) {
                    arc = soln_var_tuples[i];
                    soln_var_tuples.erase(soln_var_tuples.begin() + i);
                    node_found = true;
                    break;
                }
            }

            if (!node_found) {
                routes.back().push_back(node_to_find); // Thêm (j, t) cuối cùng
                route_finished = true;
            }
        }
    }

    // Kiểm tra mỗi nút (trừ depot) được thăm đúng một lần
    for (size_t i = 1; i < visited.size(); ++i) {
        if (visited[i] != 1.0f) {
            throw std::runtime_error("Solution does not obey node visitation constraints");
        }
    }

    return routes;
}

