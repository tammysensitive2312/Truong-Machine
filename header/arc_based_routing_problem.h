//
// Created by Truong on 03/04/2025.
//

#ifndef VEHICLE_ROUTING_PROBLEM_AS_QUBO_ARC_BASED_ROUTING_PROBLEM_H
#define VEHICLE_ROUTING_PROBLEM_AS_QUBO_ARC_BASED_ROUTING_PROBLEM_H

#include "routing_problem.h"
#include "optional"

namespace std {
    template <>
    struct hash<std::tuple<int, float, int, float>> {
        size_t operator()(const std::tuple<int, float, int, float>& t) const {
            size_t seed = 0;
            seed ^= std::hash<int>{}(std::get<0>(t)) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
            seed ^= std::hash<float>{}(std::get<1>(t)) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
            seed ^= std::hash<int>{}(std::get<2>(t)) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
            seed ^= std::hash<float>{}(std::get<3>(t)) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
            return seed;
        }
    };
}


class ArcBasedRoutingProblem : public RoutingProblem {
private:
    std::vector<float> time_points;
    std::unordered_map<std::tuple<int, float, int, float>, int> variables_mapping;
    int num_variables;
    bool variables_enumerated = false;
    bool constraints_built = false;
    bool objective_built = false;
    std::vector<std::string> constraint_names;

    Eigen::VectorXf objective;
    Eigen::SparseMatrix<float> constraints_matrix;
    Eigen::VectorXf constraints_rhs;

    void enumerate_variables_quicker();

    // Build all constraints of math program
    void build_constraints();
    /**
       Build up Linear equality constraints of BLEC
       A*x = b
     */
    void build_constraints_quicker();

    void check_and_add_exit_arc(int node_index, float cost);
public:
    ArcBasedRoutingProblem();
    ArcBasedRoutingProblem(VehicleRoutingProblemWithTimeWindows* vrptw_ptr = nullptr);

    ~ArcBasedRoutingProblem() override = default;

    int get_num_variables() override;

    /**
        Some sort of greedy construction heuristic to make sure the problem is
        feasible. We add dummy nodes/arcs as necessary to emulate more
        vehicles being available.
     * @param high_cost
     */
    void make_feasible(float high_cost) override;

    std::tuple<Eigen::VectorXf, Eigen::SparseMatrix<float>> get_objective_data() override;

    std::tuple<Eigen::SparseMatrix<float>, Eigen::VectorXf, Eigen::SparseMatrix<float>, float>
    get_constraint_data() override;

    std::unique_ptr<IloCplex> get_cplex_problem() override;

//    std::vector<double> solve_cplex_problem(const std::string &solution_filename) const override;

    float get_sufficient_penalty(bool feasibility) const;

    void add_time_points(const std::vector<float>& new_time_points);

    bool check_arc(const std::pair<int, int>& arc_key);

    bool check_node_time_compat(int node_idx, float time_point) const;

    // Build up linear objective of base BLEC formulation
    void build_objective();

    void enumerate_variables();

    const std::vector<float> &get_time_points() const;

    /**
    Get the unique id/index of the binary variable given the "tuple" indexing
    Return of None means the tuple does not correspond to a variable
    */
    const std::optional<int> get_variable_index(int node1_idx, float time1, int node2_idx, float time2) const;

    const std::optional<std::tuple<int, float, int, float>> get_variable_tuple_index(int variable_idx) const;

    /**
     Get actual arrival time, leaving at <departure_time> on <arc>

     @return
            arrival_time (float): Arrival time \n
            in_time_points (bool): whether this arrival time is in time_points set
     */
    const std::tuple<float, bool> get_arrival_time(float departure_time, Arc* arc) const;

    std::vector<std::vector<std::pair<int, float>>> get_routes(const Eigen::VectorXf &solution) const;
};

#endif //VEHICLE_ROUTING_PROBLEM_AS_QUBO_ARC_BASED_ROUTING_PROBLEM_H