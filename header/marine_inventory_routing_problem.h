//
// Created by Truong on 05/04/2025.
//

#ifndef MARINE_INVENTORY_ROUTING_PROBLEM_H
#define MARINE_INVENTORY_ROUTING_PROBLEM_H

#ifdef VEHICLE_ROUTING_PROBLEM_AS_QUBO_EXPORTS
#define VEHICLE_ROUTING_PROBLEM_AS_QUBO_API __declspec(dllexport)
#else
#define VEHICLE_ROUTING_PROBLEM_AS_QUBO_API __declspec(dllimport)
#endif

#include <memory>
#include <ostream>
#include "vector"
#include "string"
#include "map"
#include "tuple"
#include "functional"
#include "../header/vehicle_routing_problem_with_time_window.h"
#include "../header/arc_based_routing_problem.h"

//class PathBasedRoutingProblem;
//class SequenceBasedRoutingProblem;

class VEHICLE_ROUTING_PROBLEM_AS_QUBO_API MarineInventoryRoutingProblem {
private:
    float cargo_size;
    float time_horizon;
    std::vector<std::string> supply_ports;
    std::vector<std::string> demand_ports;
    std::map<std::string, std::vector<std::string>> port_mapping;
    std::map<std::string, float> port_frequency;
    bool routes_added;
    std::unique_ptr<VehicleRoutingProblemWithTimeWindows> vrptw;
    std::unique_ptr<ArcBasedRoutingProblem> abrp;
//    std::unique_ptr<PathBasedRoutingProblem> pbrp;
//    std::unique_ptr<SequenceBasedRoutingProblem> sbrp;
public:
    MarineInventoryRoutingProblem(float cargo_size, float time_horizon);

    virtual ~MarineInventoryRoutingProblem() = default;
    void add_node(const std::string& name, float demand, std::tuple<float, float> time_window);
    bool add_arc(std::string origin_name, std::string destination_name, float travel_time, float cost);
    const std::tuple<float, float> get_time_window(int num_prior_visits, float inventory_init, float inventory_rate, float inventory_cap) const;
    std::vector<std::string> add_nodes(const std::string& name, float inventory_init, float inventory_rate, float inventory_cap);
    void add_travel_arcs(
            std::function<float(std::string, std::string)> distance_function,
            float vessel_speed,
            float cost_per_unit_distance,
            const std::map<std::string, float>& supply_port_fees,
            const std::map<std::string, float>& demand_port_fees
            );
    void add_entry_arcs(float time_limit, float travel_time=0.0f, float cost=0.0f);
    void add_exit_arcs(float travel_time = 0.0f, float cost = 0.0f);
    const float estimate_high_cost() const;
    ArcBasedRoutingProblem* get_arc_based(bool make_feasible = true);
    // PathBasedRoutingProblem* get_path_based(bool make_feasible = true);
    // SequenceBasedRoutingProblem* get_sequence_based(bool make_feasible = true, bool strict = true);

    friend std::ostream &operator<<(std::ostream &os, const MarineInventoryRoutingProblem &problem);
};


#endif //MARINE_INVENTORY_ROUTING_PROBLEM_H
