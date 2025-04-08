//
// Created by ACER on 07/04/2025.
//
#include "../header/get_mirp.h"
#include "gtest/gtest.h"
#include "iostream"
#include "Eigen/Dense"
#include "marine_inventory_routing_problem.h"

class TestGetMirp : public ::testing::Test {
protected:
    void SetUp() override {
        // Code here will be called just before the test executes.
        // Initialize the MarineInventoryRoutingProblem object
        mirp = get_mirp(31);
        big_mirp = get_mirp(100);

        ASSERT_NE(nullptr, mirp.get()) << "Failed to initialize mirp";
        ASSERT_NE(nullptr, big_mirp.get()) << "Failed to initialize big_mirp";

        std::cout << "Setting up TestGetMirp\n";
//        std::cout << "mirp: " << *mirp << "\n";
    }

    void TearDown() override {
        std::cout << "Tearing down TestMirpG1\n";
    }

    void generic_sizing_tester(std::function<ArcBasedRoutingProblem*(bool)> getter, const std::string& name, int expected_size) {
        std::cout << name << "-based:\n";
        ArcBasedRoutingProblem* r_p = getter(true); // make_feasible = true
        // In thông tin các node (tương tự logger.debug trong Python)
        // Giả định ArcBasedRoutingProblem có phương thức get_nodes()
        // Nếu không, bạn cần điều chỉnh logic này
        ASSERT_NE(nullptr, r_p) << "Getter returned nullptr in " << name << "-based test";
        for (const auto& node : r_p->get_nodes()) {
            std::cout << node->get_name() << " " << node->get_demand() << " in ["
                      << std::get<0>(node->get_window()) << ", "
                      << std::get<1>(node->get_window()) << "]" << std::endl;
        }
        EXPECT_EQ(expected_size, r_p->get_num_variables())
                            << "Unexpected number of variables for " << name;
    }

    void generic_feas_tester(std::function<ArcBasedRoutingProblem*(bool)> getter, const std::string& name) {
        std::cout << name << "-based:\n";
        ArcBasedRoutingProblem* r_p = getter(true); // make_feasible = true
        ASSERT_NE(nullptr, r_p) << "Getter returned nullptr in " << name << "-based test";
        Eigen::VectorXf f_s = r_p->get_feasible_solution();

        // Lấy dữ liệu ràng buộc
        Eigen::SparseMatrix<float> A_eq;
        Eigen::VectorXf b_eq;
        Eigen::SparseMatrix<float> Q_eq;
        float r_eq;
        std::tie(A_eq, b_eq, Q_eq, r_eq) = r_p->get_constraint_data();

        // Tính residual tuyến tính: res = A_eq * f_s - b_eq
        Eigen::VectorXf res = A_eq * f_s - b_eq;

        // Tính residual bậc hai: res_q = f_s^T * Q_eq * f_s - r_eq
        float res_q = (f_s.transpose() * Q_eq * f_s).eval()(0) - r_eq;

        // Logging residual
        std::cout << name << "-based residual: " << res.transpose() << "\n";

        // Kiểm tra tính khả thi
        EXPECT_NEAR(0.0f, res.norm(), 1e-6)
                            << name << "-based feasible solution is not feasible";
        EXPECT_NEAR(0.0f, res_q, 1e-6)
                            << name << "-based feasible solution is not feasible (quad con)";
    }

    std::unique_ptr<MarineInventoryRoutingProblem> mirp;
    std::unique_ptr<MarineInventoryRoutingProblem> big_mirp;
};

TEST_F(TestGetMirp, TestArcBasedSizing) {
    auto getter = [this](bool make_feasible) {
        try {
            return mirp->get_arc_based(make_feasible);
        }
        catch (const std::exception& e) {
            ADD_FAILURE() << "Exception in getter: " << e.what();
        }
    };


    try {
        generic_sizing_tester(getter, "arc", 1680);
    }
    catch (const std::exception& e) {
        FAIL() << "Test failed with exception: " << e.what();
    }
}

TEST_F(TestGetMirp, TestArcBasedFeasibility) {
    auto getter = [this](bool make_feasible) {
        return big_mirp->get_arc_based(make_feasible);
    };
    generic_feas_tester(getter, "arc");
}
