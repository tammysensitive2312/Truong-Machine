//
// Created by Truong on 31/03/2025.
//

#ifndef VEHICLE_ROUTING_PROBLEM_AS_QUBO_ROUTING_PROBLEM_H
#define VEHICLE_ROUTING_PROBLEM_AS_QUBO_ROUTING_PROBLEM_H

#include <string>
#include <vector>
#include <map>
#include <tuple>
#include <limits>
#include <memory>
#include "Eigen/Sparse"
#include "vehicle_routing_problem_with_time_window.h"
#include "ilcplex/ilocplex.h"


class RoutingProblem {
    /**
    A base class intended to enforce a consistent interface among the different
            formulations of the Vehicle Routing Problem with Time Windows (VRPTW)
    */
protected:
    std::unique_ptr<VehicleRoutingProblemWithTimeWindows> vrptw;
    Eigen::VectorXf feasible_solution;
public:
    /**
     * Constructor
     *
     * @param vrptw_ptr pointer to VRPTW object to hold the underlying graph data. \n
     * if NULL, a new object is created.
     */
    RoutingProblem(VehicleRoutingProblemWithTimeWindows* vrptw_ptr = nullptr);

    virtual ~RoutingProblem() = default;

    const std::vector<Node*>& get_nodes() const;
    const std::vector<std::string>& get_node_names() const;
    const std::map<std::pair<int, int>, Arc*>& get_arcs() const;
    int get_depot_index() const;
    float get_vehicle_capacity() const;
    float get_initial_loading() const;
    void set_vehicle_capacity(float capacity);
    void set_initial_loading(float loading);

    /**
     * Add a node to the problem
     *
     * @param node_name
     * @param demand
     * @param time_window Time window, default is (0, infinity)
     */
    void add_node(const std::string& node_name, float demand,
                  const std::tuple<float, float>& time_window = std::make_pair(0.0f, std::numeric_limits<float>::infinity()));

    // Get the index of the node `node_name`
    int get_node_index(const std::string& node_name) const;

    // The nodes of the underlying VRPTW problem
    Node* get_node(const std::string& node_name) const;

    // Set node with name `depot_name` as depot node
    void set_depot(const std::string& depot_name);

    bool add_arc(const std::string& origin_name, const std::string& destination_name,
                 float travel_time, float cost = 0);

    // Estimate maximum number of vehicles
    int estimate_max_vehicles() const;

    // Get the number of variables in this formulation
    virtual int get_num_variables() = 0;

    /**
    Some sort of greedy construction heuristic to make sure the problem is
    feasible. We add dummy node/arcs as necessary to emulate more
    vehicles being available.
    */
    virtual void make_feasible(float high_cost) = 0;

    /**
     * Trả về dữ liệu hàm mục tiêu theo cách nhất quán
     * objective(x) = cᵀx + xᵀ Q x
     *
     * @return Tuple hold vector c and Q matrix
     */
    virtual std::tuple<Eigen::VectorXf, Eigen::SparseMatrix<float>> get_objective_data() = 0;

    /**
     * Return constraints in a consistent way
     * A_eq * x = b_eq
     * x^T * Q_eq * x = r_eq
     *
     * @return Tuple holf A_eq matrix, vector b_eq, Q_eq matrix and r_eq value
     */
    virtual std::tuple<
            Eigen::SparseMatrix<float>,
            Eigen::VectorXf,
            Eigen::SparseMatrix<float>,
            float>
    get_constraint_data() = 0;

    /**
     * Return a threshhold value of the penalty parameter that is sufficient
     * for penalizing the constraints when constructing a QUBO representation of
     * this problem
     *
     * @param feasibility : Whether this is for a feasibility version of the
     * problem. Sufficient penalty value can be zero
     *
     * @return sufficient_pp : Penalty parameter value
     */
    virtual float get_sufficient_penalty(bool feasibility) const = 0;

    /**
     * Get the Quadratic Unconstrained Binary Optimization (QUBO) problem reformulation.
        This assumes that the routing problem may be formulated as \n

            minₓ cᵀx + xᵀQx
            st
                Ax = b
                xᵀRx = 0
                x ∈ {0,1}ⁿ
     * \n
     * in which case an equivalent QUBO is \n
            min_x cᵀx + xᵀQx + ρ(||Ax - b||² + xᵀRx) \n
        for some sufficiently large parameter ρ, and appropriate handling of linear terms

     * @param feasibility Có chỉ xét tính khả thi (bỏ qua hàm mục tiêu) hay không
     * @param penalty_parameter Tham số penalty (nếu NULL, sẽ được tính tự động)
     * @return Tuple gồm ma trận Q định nghĩa QUBO và hằng số c
     */
    std::tuple<Eigen::SparseMatrix<float>, float> get_qubo(bool feasibility = false,
                                                           float penalty_parameter = -1);

    /**
     * Lấy đối tượng CPLEX chứa biểu diễn Integer Programming gốc
     * Lưu ý: Phụ thuộc vào việc sử dụng thư viện CPLEX, có thể thay thế bằng thư viện khác
     */
    virtual std::unique_ptr<IloCplex> get_cplex_problem() = 0;

    /**
     * Xuất mô hình Mixed Integer Programming ra file
     */
    void export_mip(const std::string& filename = "");

    /**
     * Solve constrained integer formulation with CPLEX
     */
//    virtual std::vector<double> solve_cplex_problem(const std::string& solution_filename = "") const = 0;

    // const std::vector<int> &get_feasible_solution() const;
};


#endif //VEHICLE_ROUTING_PROBLEM_AS_QUBO_ROUTING_PROBLEM_H