//
// Created by ACER on 07/04/2025.
//

#ifndef VEHICLE_ROUTING_PROBLEM_AS_QUBO_ISING_TOOLS_H
#define VEHICLE_ROUTING_PROBLEM_AS_QUBO_ISING_TOOLS_H

#include <vector>
#include <string>
#include <Eigen/Sparse>

std::vector<short> load_spins(const std::string& filename);

std::pair<Eigen::SparseMatrix<float>, float> load_matrix(const std::string& filename, char comment_char);

inline std::pair<Eigen::SparseMatrix<float>, float> load_qubo_matrix(const std::string& filename) {
    return load_matrix(filename, 'c');
}

inline std::pair<Eigen::SparseMatrix<float>, float> load_ising_matrix(const std::string& filename) {
    return load_matrix(filename, '#');
}

#endif //VEHICLE_ROUTING_PROBLEM_AS_QUBO_ISING_TOOLS_H
