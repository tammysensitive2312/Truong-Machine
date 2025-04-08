//
// Created by ACER on 07/04/2025.
//

#ifndef VEHICLE_ROUTING_PROBLEM_AS_QUBO_QUBO_TOOLS_H
#define VEHICLE_ROUTING_PROBLEM_AS_QUBO_QUBO_TOOLS_H

#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <vector>

namespace QuboTools {
    // {0,1} to {-1,+1} variable map x :--> 1 - 2x
    Eigen::VectorXi x_to_s(const Eigen::VectorXi& x);
// {-1,+1} to {0,1} variable map s :--> 0.5*(1 - s)
    Eigen::VectorXi s_to_x(const Eigen::VectorXi& s);

/**
 * Evaluate the objective function of a QUBO problem defined by :
 * @param Q matrix/2d-array
 * @param c scalar
 * @param x vector of {0,1}
 * @return Q.dot(x).dot(x) + c
 */
    float evaluate_QUBO(const Eigen::MatrixXf& Q, float c, const Eigen::VectorXf& x);

/**
 * Evaluate the objective function of an Ising problem defined by :
 * @param J matrix/2d-array
 * @param h vector
 * @param c scalar
 * @param s vector of {-1,+1} ("spins")
 * @return J.dot(s).dot(s) + h.dot(s) + c
 *
 * @note if `J` does not have zeroed-out diagonal, this could be incorrect
 */
    float evaluate_Ising(const Eigen::MatrixXf& J, const Eigen::VectorXf& h, float c, const Eigen::VectorXf& s);

/**
 * Get 'J' matrix and 'h' vector from `matrix`, a `scipy.sparse` sparse array. \n
 * Mutates `matrix` - zeroes out its diagonal
 */
    std::pair<Eigen::SparseMatrix<float>, Eigen::VectorXf> get_Ising_J_h(Eigen::SparseMatrix<float>& matrix);

// Chuyển đổi QUBO thành Ising: trả về (J, h, c)
    std::tuple<Eigen::SparseMatrix<float>, Eigen::VectorXf, float> QUBO_to_Ising(
            const Eigen::SparseMatrix<float>& Q, float constant = 0.0f
    );

// Chuyển đổi Ising thành QUBO: trả về (Q, c)
    std::pair<Eigen::SparseMatrix<float>, float> Ising_to_QUBO(
            const Eigen::SparseMatrix<float>& J, const Eigen::VectorXf& h, float constant = 0.0f
    );

// Chuyển ma trận thành dạng tam giác trên
    Eigen::SparseMatrix<float> to_upper_triangular(const Eigen::SparseMatrix<float>& M);

// Chuyển ma trận thành dạng đối xứng
    Eigen::SparseMatrix<float> to_symmetric(const Eigen::SparseMatrix<float>& M);
}

#endif //VEHICLE_ROUTING_PROBLEM_AS_QUBO_QUBO_TOOLS_H
