//
// Created by ACER on 06/04/2025.
//
#include "../header/vehicle_routing_problem_with_time_window.h"
#include <gtest/gtest.h>

// Test case cho lớp Node
TEST(VRPTWTest, NodeTest) {
    // Tạo một Node với tên "foo", demand = 1, time window = (0, 1)
    Node n("foo", 1.0f, std::make_tuple(0.0f, 1.0f));

    // Kiểm tra các thuộc tính
    EXPECT_EQ("foo", n.get_name());                     // Tên phải là "foo"
    EXPECT_FLOAT_EQ(1.0f, n.get_demand());             // Demand phải là 1.0
    EXPECT_FLOAT_EQ(-1.0f, n.get_load());              // Load là -demand, nên là -1.0
    auto window = n.get_window();
    EXPECT_FLOAT_EQ(0.0f, std::get<0>(window));        // Giới hạn dưới của time window là 0
    EXPECT_FLOAT_EQ(1.0f, std::get<1>(window));        // Giới hạn trên của time window là 1

    // Kiểm tra ngoại lệ khi time window không hợp lệ (1 > 0)
    ASSERT_THROW(Node("bar", 0.0f, std::make_tuple(1.0f, 0.0f)), std::invalid_argument);
}

// Test case cho lớp Arc
TEST(VRPTWTest, ArcTest) {
    // Tạo hai Node
    Node n1("foo", 1.0f, std::make_tuple(0.0f, 1.0f));
    Node n2("bar", 1.0f, std::make_tuple(1.0f, 2.0f));

    // Tạo một Arc từ n1 đến n2
    Arc a(&n1, &n2, 1.1f, 0.1f);

    // Kiểm tra các thuộc tính
    EXPECT_EQ(&n1, a.get_origin());                     // Origin phải trỏ đến n1
    EXPECT_EQ(&n2, a.get_destination());               // Destination phải trỏ đến n2
    EXPECT_FLOAT_EQ(1.1f, a.get_travel_time());        // Travel time phải là 1.1
    EXPECT_FLOAT_EQ(0.1f, a.get_cost());               // Cost phải là 0.1
}

// Test case cho lớp VehicleRoutingProblemWithTimeWindows
TEST(VRPTWTest, VRPTWTest) {
    // Tạo một đối tượng VRPTW
    VehicleRoutingProblemWithTimeWindows vrptw;

    // Thêm các Node
    vrptw.add_node("foo", 1.1f, std::make_tuple(1.0f, 2.0f));
    vrptw.add_node("bar", -1.1f, std::make_tuple(2.0f, 3.0f));

    // Kiểm tra danh sách tên Node
    std::vector<std::string> expected_names = {"foo", "bar"};
    EXPECT_EQ(expected_names, vrptw.get_node_names());

    // Kiểm tra ngoại lệ khi thêm Node trùng lặp
    ASSERT_THROW(vrptw.add_node("foo", 1.1f, std::make_tuple(1.0f, 2.0f)), std::invalid_argument);

    // Đặt depot là "bar" và kiểm tra lại thứ tự
    vrptw.set_depot("bar");
    expected_names = {"bar", "foo"};
    EXPECT_EQ(expected_names, vrptw.get_node_names());

    // Kiểm tra demand của các Node
    EXPECT_FLOAT_EQ(1.1f, vrptw.get_node("foo")->get_demand());
    EXPECT_FLOAT_EQ(-1.1f, vrptw.get_node("bar")->get_demand());

    // Thêm Arc và kiểm tra kết quả
    bool added = vrptw.add_arc("foo", "bar", 1.0f);
    EXPECT_TRUE(added);                                // Arc phải được thêm thành công

    added = vrptw.add_arc("bar", "foo", 1.0f);
    EXPECT_FALSE(added);                               // Arc không được thêm do vi phạm time window

    // Kiểm tra ước lượng số lượng xe tối đa (lúc này là 0 vì chưa đủ Arc)
    EXPECT_EQ(0, vrptw.estimate_max_vehicles());

    // Thêm Node và Arc mới để kiểm tra estimate_max_vehicles
    vrptw.add_node("baz", 0.0f, std::make_tuple(3.0f, 4.0f));
    vrptw.add_arc("bar", "baz", 1.1f, 0.0f);
    EXPECT_EQ(1, vrptw.estimate_max_vehicles());       // Số xe tối đa phải là 1
}

// Hàm main để chạy tất cả các test
int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}