cmake_minimum_required(VERSION 3.27)
project(Vehicle_Routing_Problem_as_QUBO)

set(CMAKE_CXX_STANDARD 17)

# Eigen setup
set(EIGEN3_ROOT_DIR E:/Vehicle_Routing_Problem_as_QUBO/eigen-3.4.0)
set(CMAKE_PREFIX_PATH ${CMAKE_PREFIX_PATH} ${EIGEN3_ROOT_DIR})
include_directories(${EIGEN3_ROOT_DIR})

# CPLEX setup - đường dẫn có thể cần điều chỉnh dựa trên cài đặt của bạn
set(CPLEX_ROOT_DIR "E:/IBM")

# Xác định thư mục thư viện dựa trên hệ điều hành
if(WIN32)
    set(CPLEX_LIB_DIR_SUFFIX "x64_windows_msvc14/stat_mda")
elseif(APPLE)
    set(CPLEX_LIB_DIR_SUFFIX "x86-64_osx/static_pic")
else()
    set(CPLEX_LIB_DIR_SUFFIX "x86-64_linux/static_pic")
endif()

# Đường dẫn include và thư viện
set(CPLEX_INCLUDE_DIRS
        "${CPLEX_ROOT_DIR}/cplex/include"
        "${CPLEX_ROOT_DIR}/concert/include"
)

set(CPLEX_LIB_DIR "${CPLEX_ROOT_DIR}/cplex/lib/${CPLEX_LIB_DIR_SUFFIX}")
set(CONCERT_LIB_DIR "${CPLEX_ROOT_DIR}/concert/lib/${CPLEX_LIB_DIR_SUFFIX}")

# Debug - In ra thông tin đường dẫn để kiểm tra
message(STATUS "CPLEX_LIB_DIR: ${CPLEX_LIB_DIR}")
message(STATUS "CONCERT_LIB_DIR: ${CONCERT_LIB_DIR}")

# Tìm các thư viện CPLEX bằng cách chỉ định trực tiếp đường dẫn đầy đủ
file(GLOB CPLEX_LIB_FILES "${CPLEX_LIB_DIR}/cplex*.lib")
file(GLOB ILOCPLEX_LIB_FILES "${CPLEX_LIB_DIR}/ilocplex.lib")
file(GLOB CONCERT_LIB_FILES "${CONCERT_LIB_DIR}/concert.lib")

message(STATUS "CPLEX_LIB_FILES: ${CPLEX_LIB_FILES}")
message(STATUS "ILOCPLEX_LIB_FILES: ${ILOCPLEX_LIB_FILES}")
message(STATUS "CONCERT_LIB_FILES: ${CONCERT_LIB_FILES}")

# Tìm các thư viện CPLEX - sử dụng tên file đúng dựa trên MinGW
find_library(CPLEX_LIBRARY
        NAMES cplex2212 cplex2210 cplex
        PATHS ${CPLEX_LIB_DIR}
        NO_DEFAULT_PATH
)
find_library(ILOCPLEX_LIBRARY
        NAMES ilocplex
        PATHS ${CPLEX_LIB_DIR}
        NO_DEFAULT_PATH
)
find_library(CONCERT_LIBRARY
        NAMES concert
        PATHS ${CONCERT_LIB_DIR}
        NO_DEFAULT_PATH
)

message(STATUS "CPLEX_LIBRARY: ${CPLEX_LIBRARY}")
message(STATUS "ILOCPLEX_LIBRARY: ${ILOCPLEX_LIBRARY}")
message(STATUS "CONCERT_LIBRARY: ${CONCERT_LIBRARY}")

add_subdirectory(E:/Vehicle_Routing_Problem_as_QUBO/googletest)
# Thêm các thư mục include cho dự án
include_directories(
        ${CPLEX_INCLUDE_DIRS}
        ${CMAKE_SOURCE_DIR}/header
        ${gtest_SOURCE_DIR}/include
        ${gmock_SOURCE_DIR}/include
)

# Tạo thư viện chia sẻ
add_library(Vehicle_Routing_Problem_as_QUBO SHARED
        core/vehicle_routing_problem_with_time_window.cpp
        header/vehicle_routing_problem_with_time_window.h
        core/routing_problem.cpp
        header/routing_problem.h
        core/formulations/arc_based_routing_problem.cpp
        header/arc_based_routing_problem.h
        application/marine_inventory_routing_problem.cpp
        header/marine_inventory_routing_problem.h
        core/routing_problem_factory.cpp
        header/routing_problem_factory.h
)

# Kiểm tra nếu có tìm thấy thư viện CPLEX
if(CPLEX_LIBRARY AND ILOCPLEX_LIBRARY AND CONCERT_LIBRARY)
    # Liên kết thư viện CPLEX với thư viện của bạn
    target_link_libraries(Vehicle_Routing_Problem_as_QUBO
            ${ILOCPLEX_LIBRARY}
            ${CPLEX_LIBRARY}
            ${CONCERT_LIBRARY}
    )
else()
    message(FATAL_ERROR "Could not find CPLEX libraries. Please check your installation paths.")
endif()

add_executable(run_tests
        unittest/test_vehicle_routing_problem_with_time_window.cpp
)
target_compile_definitions(Vehicle_Routing_Problem_as_QUBO PRIVATE VEHICLE_ROUTING_PROBLEM_AS_QUBO_EXPORTS)
target_link_libraries(run_tests
        Vehicle_Routing_Problem_as_QUBO
        gtest
        gtest_main
)