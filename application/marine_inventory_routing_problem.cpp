//
// Created by ACER on 05/04/2025.
//

#include "../header/marine_inventory_routing_problem.h"
#include "cmath"
#include "algorithm"
#include "stdexcept"
#include "set"


MarineInventoryRoutingProblem::MarineInventoryRoutingProblem(float cargo_size, float time_horizon)
        : cargo_size(cargo_size),
          time_horizon(time_horizon),
          supply_ports(),
          demand_ports(),
          port_mapping(),
          port_frequency(),
          routes_added(false),
          vrptw(std::make_unique<VehicleRoutingProblemWithTimeWindows>()),
          abrp(nullptr)
{
    /**
     The way we fit a MIRP into the VRPTW form is to assume that the depot
     is essentially a dummy/"source" node, and vessels have zero initial loading.
     Also, recall that the VRPTW problem assumes that vessels/vehicles are
     homogeneous (same size). \n
     We also assume full load/unload of vessels at each node.
     */
    vrptw->set_initial_loading(0.0f);
    vrptw->set_vehicle_capacity(cargo_size);
    vrptw->add_node("Depot", 0.0f, std::make_tuple(0.0f, time_horizon));
    vrptw->set_depot("Depot");
}

// Add a node to the underlying VRPTW graph
void MarineInventoryRoutingProblem::add_node(const std::string& name, float demand, std::tuple<float, float> time_window) {
    vrptw ->add_node(name, demand, time_window);
}

// Add an arc to the underlying VRPTW graph
void MarineInventoryRoutingProblem::add_arc(std::string origin_name, std::string destination_name, float travel_time,
                                            float cost) {
    vrptw ->add_arc(origin_name, destination_name, travel_time, cost);
}
/**
Determine the time window of this node. A single physical port must be
visited multiple times as it runs out of inventory or fills up its storage
capacity. The time window in which it must be visited depends on the number
of times it has previously been visited
        @param num_prior_visits (int): Number of times this node has been visited/serviced already
        @param inventory_init (float): Initial inventory level
        @param inventory_rate (float): Rate of change of inventory. \n
        - inventory_rate > 0: This is a supply port \n - inventory_rate < 0: This is a demand port
        @param inventory_cap (float): Amount of inventory capacity at this port

@return tw_start (float): Start of time window
@return tw_end (float): End of time window
*/
const std::tuple<float, float>
MarineInventoryRoutingProblem::get_time_window(int num_prior_visits, float inventory_init, float inventory_rate,
                                               float inventory_cap) const {
    float size = cargo_size;
    // inventory(t) = inventory_init + t*inventory_rate
    if (inventory_rate > 0) {
        //SUPPLY
        //Earliest a ship can load a full shipload:
        //inventory(t) - (num_prior_visits+1)*size >= 0
        float t_w0 = ((num_prior_visits+1)*size - inventory_init)/inventory_rate;
        // latest a ship can arrive before port capacity is exceeded:
        // inventory(t) - (num_prior_visits)*size > inventory_cap
        float t_w1 = (inventory_cap + (num_prior_visits)*size - inventory_rate);
        return std::make_tuple(t_w0, t_w1);
    } else {
        // DEMAND
        // Earliest a ship can discharge a full load into inventory:
        // inventory(t) + (num_prior_visits+1)*size <= inventory_cap
        float t_w0 = (inventory_cap - (num_prior_visits+1)*size - inventory_init)/inventory_rate;
        // latest a ship can arrive before port runs out of inventory:
        // inventory(t) + (num_prior_visits)*size < 0
        float t_w1 = (-(num_prior_visits)*size - inventory_init)/inventory_rate;
        return std::make_tuple(t_w0, t_w1);
    }
}

/**
 * Add nodes for this supply or demand port. A single physical port must be
   visited multiple times as it runs out of inventory or fills up its storage capacity
 *
 * @param name (string) : Base name of this port
 * @param inventory_init (float) : Initial inventory level
 * @param inventory_rate (float) : Rate of change of inventory \n
 * - inventory_rate > 0 : this is a supply port \n
 * - inventory_rate < 0 : this is a demand port
 * @param inventory_cap (float) : Amount of inventory capacity at this port
 * @return node_names (list of string) : THe names of the nodes that were added
 */
std::vector<std::string>
MarineInventoryRoutingProblem::add_nodes(const std::string& name, float inventory_init, float inventory_rate,
                                         float inventory_cap) {
    float demand_level;
    if (inventory_rate > 0) {
        // Supply port. "Demand" is negative, equal to full ship loading
        demand_level = - cargo_size;
        supply_ports.emplace_back(name);
    } else {
        // demand port. Demand is positive, equal to full ship unloading
        demand_level = cargo_size;
        demand_ports.emplace_back(name);
    }

    std::vector<std::string> node_names;
    port_mapping[name] = std::vector<std::string>();
    // Port frequency is an estimate of how frequently this port must be visited
    port_frequency[name] = std::fabs(inventory_cap/inventory_rate);
    int num_prior_visits = 0;
    while (true) {
        auto t_w = get_time_window(
                num_prior_visits,
                inventory_init,
                inventory_rate,
                inventory_cap
                );
        // only time windows fully within time horizon are added
        if (std::get<1>(t_w) > time_horizon) {
            break;
        }
        std::string node_name = name + "-" + std::to_string(num_prior_visits);
        node_names.push_back(node_name);
        vrptw -> add_node(node_names.back(), demand_level, t_w);
        port_mapping[name].push_back(node_name);
        num_prior_visits++;
    }
    return node_names;
}
/**
 *
 * Add main travel arcs between any supply port and any demand port (and vice versa) \n
        Time and cost based on distance, costs include port fees. \n
        Because the nodes have a time component, not all arcs are physically reasonable \n
  - but the underlying VRPTW checks for that.
 *
 * @param vessel_speed
 * @param cost_per_unit_distance
 * @param supply_port_fees
 * @param demand_port_fees
 */
void MarineInventoryRoutingProblem::add_travel_arcs(std::function<float(std::string, std::string)> distance_function, float vessel_speed,
                                                    float cost_per_unit_distance,
                                                    const std::map<std::string, float> &supply_port_fees,
                                                    const std::map<std::string, float> &demand_port_fees) {
    for (const auto& s_p : supply_ports) {
        for (const auto& d_p : demand_ports) {
            float distance = distance_function(s_p, d_p);
            float travel_time = distance/vessel_speed;
            float travel_cost = distance*cost_per_unit_distance;
            for (const auto& s_node : port_mapping[s_p]) {
                for (const auto& d_node : port_mapping[d_p]) {
                    add_arc(s_node, d_node, travel_time, travel_cost + demand_port_fees.at(d_p));
                    add_arc(d_node, s_node, travel_time, travel_cost + supply_port_fees.at(s_p));
                }
            }
        }
    }
}

// Add entry arcs from depot to any Supply node with time window less than
// `time_limit`
//
// Early demand ports will not get visited in time;
// Need to assume that there are loaded vessels available at start of time horizon.
// Enforce with dummy supply nodes.
// Note this is NOT required by all formulations
void MarineInventoryRoutingProblem::add_entry_arcs(float time_limit, float travel_time, float cost) {
    std::string depot_name = vrptw->get_node_names()[vrptw->get_depot_index()];

    // Thêm cung từ depot đến các nút cung
    for (const auto& port : supply_ports) {
        for (const auto& node : port_mapping[port]) {
            const auto& node_obj = vrptw->get_node(node);
            auto time_window = node_obj->get_window();
            if (std::get<1>(time_window) < time_limit) {
                std::cout << "Adding entry arc to " << node << std::endl;
                add_arc(depot_name, node, travel_time, cost);
            }
        }
    }

    // Thêm nút giả và cung cho các nút cầu sớm
    int num_dum = 0;
    for (const auto& port : demand_ports) {
        for (const auto& node : port_mapping[port]) {
            const auto& node_obj = vrptw->get_node(node);
            auto time_window = node_obj->get_window();
            if (std::get<1>(time_window) < time_limit) {
                std::string dummy = "Dum" + std::to_string(num_dum);
                num_dum++;
                // Thêm nút giả với nhu cầu -cargo_size, time_window mặc định [0, time_horizon]
                vrptw->add_node(dummy, -cargo_size, std::make_tuple(0.0f, time_horizon));
                std::cout << "Adding entry arc to " << node << std::endl;
                add_arc(depot_name, dummy, 0.0f, 0.0f);  // Từ depot đến nút giả
                add_arc(dummy, node, travel_time, cost);  // Từ nút giả đến nút cầu
            }
        }
    }
}

// Add exit arcs (back to Depot) from any "regular" supply/demand node
void MarineInventoryRoutingProblem::add_exit_arcs(float travel_time, float cost) {
    // Lấy tên depot từ vrptw
    std::string depot_name = vrptw->get_node_names()[vrptw->get_depot_index()];

    // Thêm cung từ các nút cung về depot
    for (const auto& port : supply_ports) {
        for (const auto& node : port_mapping[port]) {
            add_arc(node, depot_name, travel_time, cost);
        }
    }

    // Thêm cung từ các nút cầu về depot
    for (const auto& port : demand_ports) {
        for (const auto& node : port_mapping[port]) {
            add_arc(node, depot_name, travel_time, cost);
        }
    }
}
/**
 * Estimate a "high cost" as approximately the cost of a very expensive route.
 * Look at the port that must be visited the most often, then multiply the
 * number of times it must be visited by twice the most expensive arc cost
 */
const float MarineInventoryRoutingProblem::estimate_high_cost() const {
    // Kiểm tra nếu port_frequency rỗng
    if (port_frequency.empty()) {
        throw std::runtime_error("port_frequency is empty, cannot estimate high cost");
    }

    // Tìm tần suất nhỏ nhất (most_freq)
    auto min_freq_it = std::min_element(
            port_frequency.begin(),
            port_frequency.end(),
            [](const auto &a, const auto &b) { return a.second < b.second; }
    );
    float most_freq = min_freq_it->second;

    // Tính số lần thăm tối đa (most_trips)
    float most_trips = time_horizon / most_freq;

    // Tìm chi phí lớn nhất của cung (max_cost)
    const auto &arcs = vrptw->get_arcs();

    if (arcs.empty()) {
        throw std::runtime_error("No arcs available to estimate high cost");
    }

    float max_cost = std::max_element(
            arcs.begin(),
            arcs.end(),
            [](const auto &a, const auto &b) {
                return a.second->get_cost() < b.second->get_cost();
            })->second->get_cost();

        // Trả về chi phí ước lượng
        return 2.0f * max_cost * most_trips;
}
/**
 Return the arc-based routing problem object \n
 Build if necessary, including defining the time periods in some way
 */
ArcBasedRoutingProblem *MarineInventoryRoutingProblem::get_arc_based(bool make_feasible) {
    /**
     * Return the arc-based routing problem object
     * Build if necessary, including defining the time periods in some way
     */
    if (this->abrp != nullptr) {
        return this->abrp.get();
    }

    this->abrp = std::make_unique<ArcBasedRoutingProblem>(this->vrptw.get());


//    The only other thing we need to do for arc-based formulation is add
//    time points. This can be tricky; we want as much resolution as possible,
//    but also keep it small
//    Options:
//      integers that fall in any node's time window
//      endpoints and midpoints of time windows
//    We will use the former
    std::set<float> tw_points;
    for (const Node* n : vrptw->get_nodes()) {
        auto [tw0, tw1] = n->get_window();
        if (std::isinf(tw1)) {
            continue;
        }
        int start = static_cast<int>(std::ceil(tw0));
        int end = static_cast<int>(std::floor(tw1)) + 1;
        for (int t = start; t < end; ++t) {
            tw_points.insert(static_cast<float>(t));
        }
    }
    tw_points.insert(0.0f);
    std::vector<float> time_points(tw_points.begin(), tw_points.end());
    std::sort(time_points.begin(), time_points.end());

    this->abrp->add_time_points(time_points);

    if (make_feasible) {
        float high_cost = this->estimate_high_cost();
        this->abrp->make_feasible(high_cost);
    }
    return this->abrp.get();
}


std::ostream &operator<<(std::ostream &os, const MarineInventoryRoutingProblem &problem) {
    os << "Time horizon: " << problem.time_horizon << "\n";
    os << problem.vrptw;
    return os;
}