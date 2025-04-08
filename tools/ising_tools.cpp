//
// Created by ACER on 07/04/2025.
//

#include "../header/ising_tools.h"
#include <fstream>
#include <sstream>
#include <cassert>
#include <vector>
#include <string>
#include <optional>

// Read spins saved in textfile
// Assumes values are separated by whitespace and line breaks,
// reading left to right and top to bottom.
std::vector<short> load_spins(const std::string& filename) {
    std::ifstream file(filename);
    if (!file.is_open()) {
        throw std::runtime_error("Unable to open file: " + filename);
    }

    std::vector<short> spins;
    std::string line;
    while (std::getline(file, line)) {
        std::istringstream iss(line);
        std::string token;
        while (iss >> token) {
            // Chuyển đổi token thành số nguyên ngắn (short)
            spins.push_back(static_cast<short>(std::stof(token)));
        }
    }

    file.close();
    return spins;
}

/** Load matrix defining problem, EITHER QUBO or Ising
 *
 * @param comment_char is character at beginning of line for comment \n
 * comment_char = 'c' expected for QUBO \n
 * comment_char = '#' expected for Ising
 */
std::pair<Eigen::SparseMatrix<float>, float> load_matrix(const std::string& filename, char comment_char) {
    std::ifstream file(filename);
    if (!file.is_open()) {
        throw std::runtime_error("Unable to open file: " + filename);
    }

    // Data to build sparse matrix
    std::vector<float> data;
    std::vector<int> row;
    std::vector<int> col;
    float constant = 0.0f;
    std::optional<int> mat_length;
    int num_rows = 0;
    int num_cols = 0;

    std::string line;
    while (std::getline(file, line)) {
        if (line.empty()) continue;

        if (line[0] == comment_char) {
            // Dòng comment, tìm hằng số nếu có dấu "="
            size_t pos = line.find('=');
            if (pos != std::string::npos) {
                std::string value_str = line.substr(pos + 1);
                constant = std::stof(value_str);
            }
        }
        else if (line[0] == 'p') {
            // Dòng "sentinel" cho QUBO: p qubo 0 maxDiagonals nDiagonals nElements
            std::istringstream iss(line);
            std::string token;
            std::vector<std::string> contents;
            while (iss >> token) {
                contents.push_back(token);
            }
            if (contents.size() >= 6) {
                int nDiagonals = std::stoi(contents[4]);
                int nElements = std::stoi(contents[5]);
                mat_length = nDiagonals + nElements;
            }
        }
        else {
            // Dòng dữ liệu: row col val hoặc numvars numelements (cho Ising)
            std::istringstream iss(line);
            std::vector<std::string> contents;
            std::string token;
            while (iss >> token) {
                contents.push_back(token);
            }

            if (contents.size() == 2) {
                // Dạng Ising: numvars numelements
                mat_length = std::stoi(contents[1]);
            }
            else if (contents.size() == 3) {
                // Dòng dữ liệu: row col val
                int r = std::stoi(contents[0]);
                int c = std::stoi(contents[1]);
                float val = std::stof(contents[2]);
                row.push_back(r);
                col.push_back(c);
                data.push_back(val);
                num_rows = std::max(num_rows, r + 1);
                num_cols = std::max(num_cols, c + 1);
            }
        }
    }

    file.close();

    // Kiểm tra tính hợp lệ
    assert(numrows == numcols && "Input matrix not square");
    if (mat_length.has_value()) {
        assert(static_cast<size_t>(mat_length.value()) == row.size() && "Input matrix length discrepancy");
    }

    // Construct sparse and return
    Eigen::SparseMatrix<float> sparse_matrix(num_rows, num_cols);
    std::vector<Eigen::Triplet<float>> triplets;
    for (size_t i = 0; i < data.size(); ++i) {
        triplets.emplace_back(row[i], col[i], data[i]);
    }
    sparse_matrix.setFromTriplets(triplets.begin(), triplets.end());

    return {sparse_matrix, constant};
}