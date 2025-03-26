import h5py
import numpy as np
from scipy.sparse import csr_matrix
from scipy.sparse import triu, tril
import sys

def load_qubo_hdf5(filename):
    with h5py.File(filename, 'r') as f:
        return csr_matrix(
            (f['data'][:], f['indices'][:], f['indptr'][:]),
            shape=f['shape'][:]
        )

def read_hdf5(filename):
    with h5py.File(filename, 'r') as f:
        return f['data'][:], f['indices'][:], f['indptr'][:], f['shape'][:]


def check_sparse_matrix_properties(matrix):
    # Kiểm tra ma trận đối xứng mà không cần chuyển sang dense
    # Ma trận đối xứng nếu A - A.T có tất cả các phần tử gần 0
    diff = matrix - matrix.T
    is_symmetric = (abs(diff).max() < 1e-10)

    # Kiểm tra tam giác trên (tất cả phần tử dưới đường chéo chính = 0)
    tril_part = tril(matrix, k=-1)
    is_upper = tril_part.nnz == 0

    # Kiểm tra tam giác dưới (tất cả phần tử trên đường chéo chính = 0)
    triu_part = triu(matrix, k=1)
    is_lower = triu_part.nnz == 0

    return {
        "symmetric": is_symmetric,
        "upper_triangular": is_upper,
        "lower_triangular": is_lower
    }

def main(input_file):
    qubo = load_qubo_hdf5(input_file)
    data = read_hdf5(input_file)
    # print(data)
    print(qubo)

    # Thông tin cơ bản về ma trận
    print("Matrix shape:", qubo.shape)
    print("Number of non-zero elements:", qubo.nnz)

    # Kiểm tra đặc tính của ma trận
    properties = check_sparse_matrix_properties(qubo)
    print("\nMatrix properties:")
    print(f"Is symmetric: {properties['symmetric']}")
    print(f"Is upper triangular: {properties['upper_triangular']}")
    print(f"Is lower triangular: {properties['lower_triangular']}")

    # Hiển thị ma trận nếu kích thước không quá lớn
    if max(qubo.shape) <= 25:  # Chỉ hiển thị nếu kích thước nhỏ
        print("\nMatrix:")
        print(qubo.toarray())
    else:
        print("\nMatrix is too large to display. Showing a small sample:")
        # Hiển thị góc trên bên trái của ma trận
        sample_size = min(10, min(qubo.shape))
        print(qubo.toarray()[:sample_size, :sample_size])


if __name__ == '__main__':
    if len(sys.argv) != 2:
        print("Usage: py -3.11 utils.py <input.h5>")
        sys.exit(1)

    main(sys.argv[1])