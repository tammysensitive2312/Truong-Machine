//
// Created by ACER on 07/04/2025.
//
#include "../header/arc_based_routing_problem.h"
#include "gtest/gtest.h"
#include "Eigen/Dense"
#include "Eigen/Sparse"


class TestArcBased : public ::testing::Test {
protected:
    void SetUp() override {
    }

    void TearDown() override {
    }
};

TEST_F(TestArcBased, TestConstruction) {
    auto *ab = new ArcBasedRoutingProblem();
    ab->add_node("depot", 0);
    ab->add_node("node1", 1, std::make_tuple(1, 2));
    ab->add_node("node2", 1, std::make_tuple(3, 4));
    ab->add_arc("depot", "node1", 1, 1);
    ab->add_arc("node1", "node2", 1, 1);
    ab->add_arc("node2", "depot", 3, 3);

    EXPECT_EQ(3, ab->get_nodes().size()) << "Number of nodes is incorrect";
    EXPECT_EQ(3, ab->get_arcs().size()) << "Number of arcs is incorrect";

    std::vector<float> time_points = {6, 3, 1, 0};
    ab->add_time_points(time_points);
    Eigen::VectorXf expected_time_points(4);
    expected_time_points << 0, 1, 3, 6;

    const std::vector<float>& time_points_vec = ab->get_time_points();
    Eigen::VectorXf time_points_eigen = Eigen::Map<const Eigen::VectorXf> (
            time_points_vec.data(),
            time_points_vec.size()
            );

    double norm = (time_points_eigen - expected_time_points).norm();

    EXPECT_NEAR(0, norm, 1e-6) << "Time points are incorrect";
    EXPECT_EQ(3, ab->get_num_variables()) << "Number of variables is incorrect";

    int depot_index = ab->get_node_index("depot");
    int node1_index = ab->get_node_index("node1");
    int node2_index = ab->get_node_index("node2");
    // exception
    int dn1_index = ab->get_variable_index(depot_index, 0, node1_index, 1).value();
    int n1n2_index = ab->get_variable_index(node1_index, 1, node2_index, 3).value();
    int n2d_index = ab->get_variable_index(node2_index, 3, depot_index, 6).value();

    Eigen::VectorXf c = Eigen::VectorXf::Zero(3);
    c(dn1_index) = 1;
    c(n1n2_index) = 1;
    c(n2d_index) = 3;

    auto [c_obj, Q_obj] = ab->get_objective_data();
    EXPECT_NEAR(0, (c_obj - c).norm(), 1e-6) << "Objective is incorrect";
    EXPECT_EQ(0, Q_obj.nonZeros()) << "Too many quadratic terms in objective";

    Eigen::VectorXf x = Eigen::VectorXf::Ones(3);
    auto [A_eq, b_eq, Q_eq, r_eq] = ab->get_constraint_data();
    EXPECT_NEAR(0, (A_eq * x - b_eq).norm(), 1e-6) << "Linear constraints not satisfied";
    EXPECT_EQ(0, Q_eq.nonZeros()) << "Too many quadratic constraints";
    EXPECT_EQ(0, r_eq) << "Quadratic constraint data is wrong";

    float pp = 100.0;
    auto [Q, c_qubo] = ab->get_qubo(false, pp);
    EXPECT_NEAR(pp * b_eq.dot(b_eq), c_qubo, 1e-6) << "Constant of QUBO is wrong";

    Eigen::MatrixXf Q_manual = pp * (A_eq.transpose() * A_eq);
    Q_manual += (-2 * pp * A_eq.transpose() * b_eq + c_obj).asDiagonal();
    EXPECT_NEAR(0, (Q - Q_manual).norm(), 1e-6) << "QUBO matrix is incorrect";
    delete(ab);
}