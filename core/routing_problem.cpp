//
// Created by ACER on 31/03/2025.
//

#include "../header/routing_problem.h"

RoutingProblem::RoutingProblem(VehicleRoutingProblemWithTimeWindows *vrptw_ptr) {
    if (vrptw_ptr == nullptr) {
        vrptw = std::make_unique<VehicleRoutingProblemWithTimeWindows>();
    } else {
        vrptw = std::make_unique<VehicleRoutingProblemWithTimeWindows>(*vrptw_ptr);
    }
}

const std::vector<Node*>& RoutingProblem::get_nodes() const {
    return vrptw->get_nodes();
}

const std::vector<std::string>& RoutingProblem::get_node_names() const {
    return vrptw->get_node_names();
}

const std::map<std::pair<int, int>, Arc*> & RoutingProblem::get_arcs() const {
    return vrptw->get_arcs();
}

int RoutingProblem::get_depot_index() const {
    return vrptw->get_depot_index();
}

float RoutingProblem::get_vehicle_capacity() const {
    return vrptw->get_vehicle_capacity();
}

float RoutingProblem::get_initial_loading() const {
    return vrptw->get_initial_loading();
}

void RoutingProblem::set_vehicle_capacity(float capacity) {
    return vrptw->set_vehicle_capacity(capacity);
}

void RoutingProblem::set_initial_loading(float loading) {
    return vrptw->set_initial_loading(loading);
}

void RoutingProblem::add_node(const std::string &node_name, float demand, const std::tuple<float, float> &time_window) {
    return vrptw->add_node(node_name, demand, time_window);
}

int RoutingProblem::get_node_index(const std::string &node_name) const {
    return vrptw->get_node_index(node_name);
}

Node* RoutingProblem::get_node(const std::string &node_name) const {
    return vrptw->get_node(node_name);
}

void RoutingProblem::set_depot(const std::string &depot_name) {
    return vrptw->set_depot(depot_name);
}

bool RoutingProblem::add_arc(const std::string &origin_name, const std::string &destination_name, float travel_time,
                             float cost) {
    return vrptw->add_arc(origin_name, destination_name, travel_time, cost);
}

int RoutingProblem::estimate_max_vehicles() const {
    return vrptw->estimate_max_vehicles();
}

const Eigen::VectorXf &RoutingProblem::get_feasible_solution() const {
    return feasible_solution;
}

std::tuple<Eigen::SparseMatrix<float>, float> RoutingProblem::get_qubo(bool feasibility, float penalty_parameter) {
    float sufficient_pp = get_sufficient_penalty(feasibility);

    float pp = (penalty_parameter > 0) ? penalty_parameter : (sufficient_pp + 1.0f);
    if (pp <= sufficient_pp) {
        std::cout << "Penalty parameter might not be big enough... (> " << sufficient_pp << ")" << std::endl;
    }

    Eigen::SparseMatrix<float> A_eq = std::get<0>(get_constraint_data());
    Eigen::VectorXf b_eq = std::get<1>(get_constraint_data());
    Eigen::SparseMatrix<float> Q_eq = std::get<2>(get_constraint_data());
    float r_eq = std::get<3>(get_constraint_data());

    if (r_eq != 0) {
        throw std::runtime_error("QUBO construction assumes quadratic constraints are of form xᵀ Q x = 0");
    }
    /**
    penalized Quadratic and Linear equality constraints
    ρ( xᵀQx + ||Ax - b||² ) = ρ( xᵀ(AᵀA + Q)x - 2bᵀAx + bᵀb )
    -ρ2bᵀA goes on the diagonal
    */

    Eigen::VectorXf two_bTA = -2.0f * (A_eq.transpose() * b_eq);
    Eigen::SparseMatrix<float> Q = penalty_parameter * (
            Q_eq +
            A_eq.transpose() * A_eq +
            Eigen::SparseMatrix<float>(two_bTA.asDiagonal())
    );
    if (!feasibility) {
        Eigen::VectorXf c_obj = std::get<0>(get_objective_data());
        Eigen::SparseMatrix Q_obj = std::get<1>(get_objective_data());
        Q += Q_obj + Eigen::SparseMatrix<float>(c_obj.asDiagonal());
    }

    float constant = penalty_parameter * b_eq.dot(b_eq);

    return {Q, constant};
}

void RoutingProblem::export_mip(const std::string &filename) {
    // Export constrained integer program representation of problem
    std::string output_filename;
    if (filename.empty()) {
        output_filename = typeid(*this).name();
        output_filename += ".lp";
    } else {
        output_filename = filename;
    }

    std::unique_ptr<IloCplex> cplex_prob = get_cplex_problem();

    cplex_prob->exportModel(output_filename.c_str());
}
