//
// Created by ACER on 06/04/2025.
//
#include "../header/marine_inventory_routing_problem.h"
#include "memory"

std::unique_ptr<MarineInventoryRoutingProblem> get_mirp(float time_horizon) {
    // Define a specific problem given a time horizon
    /**
     * Create a routing problem
     * specify cargo size/vessel capacity and time horizon
     */
     float cargo_size = 300;
     auto mirp = std::make_unique<MarineInventoryRoutingProblem>(cargo_size, time_horizon);

     std::vector<std::string> d_names = {"D1", "D2", "D3"};
     std::vector<int> d_inventories = {221, 215, 175};
     std::vector<int> d_rates = {-34, -31, -25};
     std::vector<int> d_tankages = {374, 403, 300};
     std::vector<int> d_fees = {60, 82, 94};

     std::map<std::string, float> demand_port_fees;
     for (size_t i = 0; i < d_names.size(); ++i) {
         demand_port_fees[d_names[i]] = d_fees[i];
     }

     for (size_t i = 0; i < d_names.size(); ++i) {
         mirp->add_nodes(d_names[i], d_inventories[i], d_rates[i], d_tankages[i]);
     }

     std::vector<std::string> s_names = {"S1", "S2"};
     std::vector<int> s_inventories = {220, 270};
     std::vector<int> s_rates = {47, 42};
     std::vector<int> s_tankages = {376,  420};
     std::vector<int> s_fees = {30,   85};

     std::map<std::string, float> supply_port_fees;
     for (size_t i = 0; i < s_names.size(); ++i) {
         supply_port_fees[s_names[i]] = s_fees[i];
     }

     for (size_t i = 0; i < s_names.size(); ++i) {
         mirp->add_nodes(s_names[i], s_inventories[i], s_rates[i], s_tankages[i]);
     }

     float vessel_speed = 665.0;
     float cost_per_unit_distance = 0.09;

     Eigen::MatrixXf distance_matrix(5, 5);
     distance_matrix << 0.00,    212.34,    5305.34,   5484.21,   5459.31,
                        212.34,  0.00,      5496.06,   5674.36,   5655.55,
                        5305.34, 5496.06,   0.00,      181.69,    380.30,
                        5484.21, 5674.36,   181.69,    0.00,      386.66,
                        5459.31, 5655.55,   380.30,    386.66,    0.00;

     std::vector<std::string> combined_ports;
     combined_ports.insert(combined_ports.end(), s_names.begin(), s_names.end());
     combined_ports.insert(combined_ports.end(), d_names.begin(), d_names.end());

     std::map<std::string, size_t> port_indices;
     for (size_t i = 0; i < combined_ports.size(); ++i) {
         port_indices[combined_ports[i]] = i;
     }
     auto distance_function = [&port_indices, &distance_matrix](const std::string& port1, const std::string& port2) {
         auto i = port_indices.at(port1);
         auto j = port_indices.at(port2);
         return distance_matrix(i, j);
     };

     mirp->add_travel_arcs(
             distance_function,
             vessel_speed,
             cost_per_unit_distance,
             supply_port_fees,
             demand_port_fees
             );
     mirp->add_exit_arcs();
     float time_limit = 14;
     mirp->add_entry_arcs(time_limit);
     return mirp;
}
