//
// Created by ACER on 07/04/2025.
//

#include "../header/marine_inventory_routing_problem.h"
#include "gtest/gtest.h"

TEST(MarineInventoryRoutingProblemTest, TestTimeWindows) {
    // Test time window construction
    MarineInventoryRoutingProblem mirp = MarineInventoryRoutingProblem(1, 1);
    // No prior visits, zero initial inventory, inventory fills at 1 unit/day:
    // a ship of size 1 can fill up at day 1,
    // and we max out the capacity at day 2
    auto time_windows = mirp.get_time_window(0, 0, 1, 2);

    // Kiểm tra từng phần tử trong tuple
    EXPECT_FLOAT_EQ(std::get<0>(time_windows), 1.0f) << "Start time is incorrect";
    EXPECT_FLOAT_EQ(std::get<1>(time_windows), 2.0f) << "End time is incorrect";

    // Hoặc kiểm tra cả tuple cùng lúc
    EXPECT_EQ(time_windows, std::make_tuple(1.0f, 2.0f)) << "Time window is incorrect";
}

TEST(MarineInventoryRoutingProblemTest, TestAddNodes) {
    // Test adding nodes
    MarineInventoryRoutingProblem mirp = MarineInventoryRoutingProblem(1, 1);
    // Time horizon is not long enough; no nodes should be added
    std::vector<std::string> node_names = mirp.add_nodes("foo", 0, 1, 2.0f);
    EXPECT_EQ(node_names.size(), 0) << "Nodes added, but should not have been";

    MarineInventoryRoutingProblem mirp2 = MarineInventoryRoutingProblem(1, 2);
    std::vector<std::string> node_names2 = mirp2.add_nodes("foo", 0, 1, 2.0f);
    EXPECT_EQ(node_names2.size(), 1) << "Wrong number of nodes added";
}

TEST(MarineInventoryRoutingProblemTest, TestHighCostEstimation) {
    // Test adding entry arcs
    MarineInventoryRoutingProblem mirp = MarineInventoryRoutingProblem(1, 2);
    std::vector<std::string> node_names = mirp.add_nodes("foo", 0, 1, 2.0f);
    bool added = mirp.add_arc("Depot", node_names[0], 1, 10);
    EXPECT_TRUE(added) << "Arc not added, but should have been";

    // One node, which has a frequency cap/rate = 2 days
    // Must be visited once in the 2 day time horizon
    float cost = mirp.estimate_high_cost();
    EXPECT_EQ(2*10*1, cost) << "High cost estimation is incorrect";
}