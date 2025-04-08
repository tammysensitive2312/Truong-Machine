//
// Created by ACER on 07/04/2025.
//

#include "../header/qubo_tools.h"

namespace QuboTools {
    Eigen::VectorXi x_to_s(const Eigen::VectorXi &x) {
        return (Eigen::VectorXi::Ones(x.size()) - 2 * x);
    }

    Eigen::VectorXi s_to_x(const Eigen::VectorXi &s) {
        Eigen::VectorXf temp = (Eigen::VectorXi::Ones(s.size()) - s).cast<float>();
        return (0.5 * temp).cast<int>();
    }

    float evaluate_QUBO(const Eigen::MatrixXf &Q, float c, const Eigen::VectorXf &x) {
        return (x.transpose() * Q * x) + c;
    }

    float evaluate_Ising(const Eigen::MatrixXf &J, const Eigen::VectorXf &h, float c, const Eigen::VectorXf &s) {
        float term1 = (s.transpose() * J * s).eval()(0);
        float term2 = (h.transpose() * s).eval()(0);
        return term1 + term2 + c;
    }

    std::pair<Eigen::SparseMatrix<float>, Eigen::VectorXf> get_Ising_J_h(Eigen::SparseMatrix<float> &matrix) {
        Eigen::VectorXf h(matrix.rows());
        for (int i = 0; i < matrix.rows(); ++i) {
            h(i) = matrix.coeff(i, i);
        }

        matrix.diagonal().setZero();
        return {matrix, h};
    }

    std::tuple<Eigen::SparseMatrix<float>, Eigen::VectorXf, float>
    QUBO_to_Ising(const Eigen::SparseMatrix<float> &Q, float constant) {
        // Kiểm tra ma trận vuông
        if (Q.rows() != Q.cols()) {
            throw std::invalid_argument("Expected a square matrix.");
        }

        int n = Q.rows();

        // J = 0.25 * Q
        Eigen::SparseMatrix<float> J = 0.25 * Q;

        // h = -0.25 * (Q.sum(0) + Q.sum(1))
        Eigen::VectorXf h = Eigen::VectorXf::Zero(n);
        for (int j = 0; j < n; ++j) {
            float row_sum = 0.0f;
            float col_sum = 0.0f;
            for (Eigen::SparseMatrix<float>::InnerIterator it(Q, j); it; ++it) {
                col_sum += it.value();
            }
            for (int k = 0; k < Q.outerSize(); ++k) {
                for (Eigen::SparseMatrix<float>::InnerIterator it(Q, k); it; ++it) {
                    if (it.col() == j) {
                        row_sum += it.value();
                    }
                }
            }
            h(j) = -0.25 * (row_sum + col_sum);
        }

        // c = 0.25 * (Q.sum() + Q.diagonal().sum()) + const
        float Q_sum = Q.sum();
        float diag_sum = 0.0f;
        for (int i = 0; i < n; ++i) {
            diag_sum += Q.coeff(i, i);
        }
        float c = 0.25 * (Q_sum + diag_sum) + constant;

        // Đặt đường chéo của J về 0
        J.diagonal().setZero();
        J.prune(0.0f); // Tương đương eliminate_zeros()

        return {J, h, c};
    }

    std::pair<Eigen::SparseMatrix<float>, float>
    Ising_to_QUBO(const Eigen::SparseMatrix<float> &J, const Eigen::VectorXf &h, float constant) {
        // Kiểm tra ma trận vuông và kích thước tương thích
        if (J.rows() != J.cols()) {
            throw std::invalid_argument("Expected a square matrix.");
        }
        if (J.rows() != h.size()) {
            throw std::invalid_argument("Expected a matrix and vector of compatible size.");
        }

        int n = J.rows();

        // Tính tổng theo hàng và cột của J
        Eigen::VectorXf row_sums = Eigen::VectorXf::Zero(n);
        Eigen::VectorXf col_sums = Eigen::VectorXf::Zero(n);
        for (int j = 0; j < n; ++j) {
            for (Eigen::SparseMatrix<float>::InnerIterator it(J, j); it; ++it) {
                col_sums(j) += it.value();
            }
            for (int k = 0; k < J.outerSize(); ++k) {
                for (Eigen::SparseMatrix<float>::InnerIterator it(J, k); it; ++it) {
                    if (it.col() == j) {
                        row_sums(j) += it.value();
                    }
                }
            }
        }

        // Q = 4*J - 2*diag(J.sum(0) + J.sum(1) + h)
        Eigen::SparseMatrix<float> Q = 4.0 * J;
        for (int i = 0; i < n; ++i) {
            float diag_value = -2.0 * (row_sums(i) + col_sums(i) + h(i));
            Q.coeffRef(i, i) += diag_value; // Thêm vào đường chéo
        }

        // c = J.sum() + h.sum() + const
        float c = J.sum() + h.sum() + constant;

        Q.prune(0.0f); // Tương đương eliminate_zeros()
        return {Q, c};
    }

// Chuyển ma trận thành dạng tam giác trên
    Eigen::SparseMatrix<float> to_upper_triangular(const Eigen::SparseMatrix<float>& M) {
        // Kiểm tra ma trận vuông
        if (M.rows() != M.cols()) {
            throw std::invalid_argument("Expected a square matrix.");
        }

        int n = M.rows();
        std::vector<Eigen::Triplet<float>> triplets;

        // Lấy phần tam giác dưới nghiêm ngặt (k=-1)
        for (int k = 0; k < M.outerSize(); ++k) {
            for (Eigen::SparseMatrix<float>::InnerIterator it(M, k); it; ++it) {
                if (it.row() > it.col()) { // Phần tử dưới đường chéo
                    triplets.emplace_back(it.row(), it.col(), it.value());
                }
            }
        }

        // Tạo ma trận LT (tam giác dưới)
        Eigen::SparseMatrix<float> LT(n, n);
        LT.setFromTriplets(triplets.begin(), triplets.end());

        Eigen::SparseMatrix<float> LT_transpose = LT.transpose();
        Eigen::SparseMatrix<float> diff = LT_transpose - LT;
        Eigen::SparseMatrix<float> UT = M + diff;
        UT.prune(0.0f);

        return UT;
    }

// Chuyển ma trận thành dạng đối xứng
    Eigen::SparseMatrix<float> to_symmetric(const Eigen::SparseMatrix<float>& M) {
        // Kiểm tra ma trận vuông
        if (M.rows() != M.cols()) {
            throw std::invalid_argument("Expected a square matrix.");
        }

        Eigen::SparseMatrix<float> M_transpose = M.transpose();
        // S = 0.5 * (M + M^T)
        Eigen::SparseMatrix<float> S = 0.5 * (M + M_transpose);
        S.prune(0.0f);

        return S;
    }

}