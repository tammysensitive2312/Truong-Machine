//
// Created by ACER on 07/04/2025.
//

#include "../header/qubo_container.h"
#include "../header/qubo_tools.h"
#include <stdexcept>
#include <set>
#include "chrono"
#include "iomanip"
#include "fstream"

QUBOContainer::QUBOContainer(const Eigen::SparseMatrix<float>& Q_input, float constant_qubo_input, const std::string& pattern)
        : constant_qubo(constant_qubo_input) {
    if (Q_input.rows() != Q_input.cols()) {
        throw std::invalid_argument("Expected a square matrix.");
    }
    n_vars = Q_input.rows();

    if (pattern == "upper_triangular") {
        Q = QuboTools::to_upper_triangular(Q_input);
    } else if (pattern == "symmetric") {
        Q = QuboTools::to_symmetric(Q_input);
    } else {
        Q = Q_input;
    }

    auto [J_out, h_out, c_ising] = QuboTools::QUBO_to_Ising(Q, constant_qubo);
    J = J_out;
    h = h_out;
    constant_ising = c_ising;
}

std::function<float(const Eigen::VectorXf&)> QUBOContainer::get_objective_function_QUBO() {
    // return QUBO objective function
    return [this](const Eigen::VectorXf& x) {
        return evaluate_QUBO(x);
    };
}

std::function<float(const Eigen::VectorXf&)> QUBOContainer::get_objective_function_Ising() {
    return [this](const Eigen::VectorXf& s) {
        return evaluate_Ising(s);
    };
}

float QUBOContainer::evaluate_QUBO(const Eigen::VectorXf& x) {
    return QuboTools::evaluate_QUBO(Q, constant_qubo, x);
}

float QUBOContainer::evaluate_Ising(const Eigen::VectorXf& s) {
    return QuboTools::evaluate_Ising(J, h, constant_ising, s);
}

ReportResult QUBOContainer::report(bool obj_stats, float tol) {
    ReportResult result;
    // Khởi tạo các trường không phụ thuộc vào obj_stats
    Eigen::SparseMatrix<float> matrix = QuboTools::to_upper_triangular(Q);
    int n = n_vars;

    result.size = n;
    int nnz = matrix.nonZeros();
    result.num_observables = nnz;
    result.density = (2.0f * nnz) / ((n + 1) * n);

    // Tính distinct_eigenvalues dựa trên đường chéo
    std::set<float> unique_diags;
    for (int i = 0; i < n; ++i) {
        unique_diags.insert(matrix.coeff(i, i));
    }
    result.distinct_eigenvalues = unique_diags.size();

    // Khởi tạo các trường phụ thuộc vào obj_stats với giá trị mặc định
    result.optimal_value = 0.0f;
    result.num_solutions = 0;
    result.expected_value = 0.0f;
    result.optimality_gap = 0.0f;

    if (obj_stats) {
        auto obj_func = get_objective_function_QUBO();
        float exp_val = 0.0f;
        // initialize to the objective value for x = [0, 0, ..., 0]
        float opt_val = constant_qubo;
        float second_best = std::numeric_limits<float>::max();
        int opt_count = 1;
        long long N = 1LL << n; // 2^n

        for (long long v = 1; v < N; ++v) {
            // Chuyển v thành vector nhị phân
            Eigen::VectorXf x(n);
            for (int i = 0; i < n; ++i) {
                x(i) = (v >> i) & 1 ? 1.0f : 0.0f;
            }

            float obj_val = obj_func(x);
            exp_val += obj_val / N;

            if (std::abs(obj_val - opt_val) <= tol) {
                opt_count++;
            } else if (obj_val < opt_val) {
                second_best = opt_val;
                opt_val = obj_val;
                opt_count = 1;
            } else if (second_best == std::numeric_limits<float>::max() || obj_val < second_best) {
                second_best = obj_val;
            }
        }

        result.optimal_value = opt_val;
        result.num_solutions = opt_count;
        result.expected_value = exp_val;
        if (second_best != std::numeric_limits<float>::max()) {
            result.optimality_gap = second_best - opt_val;
        }
    }

    return result;
}

void QUBOContainer::export_to_file(const std::string& filename, bool as_ising) {
    int N = n_vars;
    char cchar = '#';

    // Xác định ma trận, vector đường chéo, hằng số, và đuôi file
    const Eigen::SparseMatrix<float>& Mat = as_ising ? J : Q;
    Eigen::VectorXf d = as_ising ? h : Mat.diagonal();
    float constant = as_ising ? constant_ising : constant_qubo;
    std::string extension = as_ising ? ".rudy" : ".qubo";

    std::stringstream contents;

    // Lấy thời gian hiện tại
    auto now = std::chrono::system_clock::now();
    std::time_t now_time = std::chrono::system_clock::to_time_t(now);
    contents << cchar << " Generated " << std::put_time(std::localtime(&now_time), "%Y-%m-%d %H:%M:%S") << "\n";

    // Ghi hằng số
    contents << "\n" << cchar << " Constant term of objective = " << std::fixed << std::setprecision(2) << constant << "\n";

    // Ghi các phần tử trên đường chéo
    contents << "\n" << cchar << " Diagonal terms\n";
    int nDiagonals = 0;
    for (int i = 0; i < N; ++i) {
        float value = d(i);
        if (value != 0.0f) {
            contents << i << " " << i << " " << std::fixed << std::setprecision(2) << value << "\n";
            nDiagonals++;
        }
    }

    // Ghi các phần tử ngoài đường chéo
    contents << "\n" << cchar << " Off-Diagonal terms\n";
    int nElements = 0;
    for (int k = 0; k < Mat.outerSize(); ++k) {
        for (Eigen::SparseMatrix<float>::InnerIterator it(Mat, k); it; ++it) {
            int r = it.row();
            int c = it.col();
            float v = it.value();
            if (r == c) {
                continue; // Skip diagonal elements
            }
            contents << r << " " << c << " " << std::fixed << std::setprecision(2) << v << "\n";
            nElements++;
        }
    }

    std::string actual_filename = filename.empty() ? "fubo" + extension : filename;

    std::ofstream f(actual_filename, std::ios::out);
    if (!f.is_open()) {
        throw std::runtime_error("Unable to open file: " + actual_filename);
    }
    f << contents.str();
    f.close();
}
