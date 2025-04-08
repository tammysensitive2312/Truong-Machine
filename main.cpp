//
// Created by ACER on 08/04/2025.
//
#include <iostream>
#include <functional>
#include <map>
#include <string>
#include <vector>
#include "../header/marine_inventory_routing_problem.h"

// Simple distance function between ports
float distance_between_ports(const std::string& port1, const std::string& port2) {
    // In a real implementation, this would use actual distances or coordinates
    // For testing purposes, we'll use a simple mapping
    std::map<std::pair<std::string, std::string>, float> distances = {
            {{"SupplyPort1", "DemandPort1"}, 100.0f},
            {{"SupplyPort1", "DemandPort2"}, 150.0f},
            {{"SupplyPort2", "DemandPort1"}, 200.0f},
            {{"SupplyPort2", "DemandPort2"}, 125.0f}
    };

    auto key = std::make_pair(port1, port2);
    if (distances.find(key) != distances.end()) {
        return distances[key];
    }

    // Check if the reverse pair exists
    auto reverse_key = std::make_pair(port2, port1);
    if (distances.find(reverse_key) != distances.end()) {
        return distances[reverse_key];
    }

    // Default distance if not found (should be avoided in practice)
    return 200.0f;
}

int main() {
    try {
        std::cout << "Creating Marine Inventory Routing Problem instance..." << std::endl;

        // Create a MIRP instance
        float cargo_size = 100.0f;        // Size of each vessel cargo
        float time_horizon = 30.0f;       // Planning horizon (e.g., 30 days)
        MarineInventoryRoutingProblem mirp(cargo_size, time_horizon);

        // Add supply ports
        std::cout << "Adding supply ports..." << std::endl;
        float supply_rate1 = 20.0f;       // Rate of production per day
        float supply_rate2 = 15.0f;
        float supply_init1 = 50.0f;       // Initial inventory level
        float supply_init2 = 80.0f;
        float supply_cap1 = 200.0f;       // Storage capacity
        float supply_cap2 = 150.0f;

        auto supply_nodes1 = mirp.add_nodes("SupplyPort1", supply_init1, supply_rate1, supply_cap1);
        auto supply_nodes2 = mirp.add_nodes("SupplyPort2", supply_init2, supply_rate2, supply_cap2);

        std::cout << "Created " << supply_nodes1.size() << " nodes for SupplyPort1" << std::endl;
        std::cout << "Created " << supply_nodes2.size() << " nodes for SupplyPort2" << std::endl;

        // Add demand ports
        std::cout << "Adding demand ports..." << std::endl;
        float demand_rate1 = -15.0f;      // Negative rate indicates consumption
        float demand_rate2 = -25.0f;
        float demand_init1 = 150.0f;      // Initial inventory level
        float demand_init2 = 180.0f;
        float demand_cap1 = 200.0f;       // Storage capacity
        float demand_cap2 = 250.0f;

        auto demand_nodes1 = mirp.add_nodes("DemandPort1", demand_init1, demand_rate1, demand_cap1);
        auto demand_nodes2 = mirp.add_nodes("DemandPort2", demand_init2, demand_rate2, demand_cap2);

        std::cout << "Created " << demand_nodes1.size() << " nodes for DemandPort1" << std::endl;
        std::cout << "Created " << demand_nodes2.size() << " nodes for DemandPort2" << std::endl;

        // Add travel arcs
        std::cout << "Adding travel arcs..." << std::endl;
        float vessel_speed = 20.0f;       // Distance units per time unit
        float cost_per_unit_distance = 0.5f;

        std::map<std::string, float> supply_port_fees = {
                {"SupplyPort1", 10.0f},
                {"SupplyPort2", 15.0f}
        };

        std::map<std::string, float> demand_port_fees = {
                {"DemandPort1", 12.0f},
                {"DemandPort2", 8.0f}
        };

        mirp.add_travel_arcs(distance_between_ports, vessel_speed, cost_per_unit_distance,
                             supply_port_fees, demand_port_fees);

        // Add entry and exit arcs
        std::cout << "Adding entry arcs..." << std::endl;
        float entry_time_limit = 10.0f;
        mirp.add_entry_arcs(entry_time_limit, 1.0f, 5.0f);

        std::cout << "Adding exit arcs..." << std::endl;
        mirp.add_exit_arcs(1.0f, 5.0f);

        // Print problem information
//        std::cout << "\nMIRP Problem:\n" << mirp << std::endl;

        // Test get_arc_based function
        std::cout << "Testing get_arc_based function..." << std::endl;
        bool make_feasible = true;
        ArcBasedRoutingProblem* abrp = mirp.get_arc_based(make_feasible);

        if (abrp) {
            std::cout << "Successfully created Arc-Based Routing Problem" << std::endl;
            std::cout << "Number of time points: " << abrp->get_time_points().size() << std::endl;

            // Print some time points for verification
            std::cout << "Time points: ";
            const auto& time_points = abrp->get_time_points();
            for (size_t i = 0; i < std::min(size_t(10), time_points.size()); ++i) {
                std::cout << time_points[i] << " ";
            }
            if (time_points.size() > 10) {
                std::cout << "... (" << time_points.size() - 10 << " more)";
            }
            std::cout << std::endl;

            // You can add more detailed testing of the ABRP here
        } else {
            std::cout << "Failed to create Arc-Based Routing Problem!" << std::endl;
        }

        return 0;
    } catch (const std::exception& e) {
        std::cerr << "ERROR: " << e.what() << std::endl;
        return 1;
    } catch (...) {
        std::cerr << "UNKNOWN ERROR occurred!" << std::endl;
        return 1;
    }
}