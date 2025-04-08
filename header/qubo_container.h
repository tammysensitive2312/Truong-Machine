//
// Created by ACER on 07/04/2025.
//

#ifndef VEHICLE_ROUTING_PROBLEM_AS_QUBO_QUBO_CONTAINER_H
#define VEHICLE_ROUTING_PROBLEM_AS_QUBO_QUBO_CONTAINER_H

#include "Eigen/Sparse"
#include "Eigen/Core"
#include "any"

struct ReportResult {
    int size;
    int num_observables;
    float density;
    int distinct_eigenvalues;
    float optimal_value;  // Chỉ có khi obj_stats = true
    int num_solutions;    // Chỉ có khi obj_stats = true
    float expected_value; // Chỉ có khi obj_stats = true
    float optimality_gap; // Chỉ có khi obj_stats = true
};

/**
 * Tools for defining and manipulating Quadratic Unconstrained Binary Optimization
 * (QUBO) problems
 */
class QUBOContainer {
private:
    int n_vars; // number of variables
    Eigen::SparseMatrix<float> J; // J matrix
    Eigen::VectorXf h; // h vector
    float constant_ising; // constant term
    Eigen::SparseMatrix<float> Q; // QUBO matrix
    float constant_qubo; // constant term
public:
    /**
     * Constructor
     * @param Q QUBO matrix
     * @param constant_qubo constant term for QUBO
     * @param pattern "upper_triangular" or "symmetric"
     */
    QUBOContainer(const Eigen::SparseMatrix<float>& Q, float constant_qubo, const std::string& pattern = "upper_triangular");

    /**
     * Get the QUBO objective function
     * @return std::function that takes a vector x and returns the QUBO objective value
     */
    std::function<float(const Eigen::VectorXf&)> get_objective_function_QUBO();

    /**
     * Get the Ising objective function
     * @return std::function that takes a vector s and returns the Ising objective value
     */
    std::function<float(const Eigen::VectorXf&)> get_objective_function_Ising();

    /**
     * Evaluate the QUBO objective for binaries x
     * @param x binary vector
     * @return objective value
     */
    float evaluate_QUBO(const Eigen::VectorXf& x);

    /**
     * Evaluate the Ising objective for spins s
     * @param s spin vector
     * @return objective value
     */
    float evaluate_Ising(const Eigen::VectorXf& s);

    // A function for generating a dictionary of 'metrics'.
    // obj_stats=True will do exhaustive search over (exponentially-many) bitstrings
    ReportResult report(bool obj_stats = false, float tol = 1e-16f);

    /**
     * Export the QUBO or Ising problem to a file with a specific structure
     * @param filename Name of the file to write to (if empty, use default name)
     * @param as_ising If true, export as Ising; otherwise, export as QUBO
     */
    void export_to_file(const std::string& filename = "", bool as_ising = false);
};



#endif //VEHICLE_ROUTING_PROBLEM_AS_QUBO_QUBO_CONTAINER_H
