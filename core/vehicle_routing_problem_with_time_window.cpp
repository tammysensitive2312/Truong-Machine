//
// Created by ACER on 31/03/2025.
//

#include "../header/vehicle_routing_problem_with_time_window.h"
#include <algorithm>
#include <stdexcept>
#include <cmath>

/**
    * A node is a customer, with a demand level that must be satisfied in a particular
    * window of time.
    * Here, the sign convention for demand is from the perspective of the node:
    *     positive demand must be delivered,
    *     negative demand is supply that must be picked up.
*/
Node::Node(const std::string &name, float demand, const std::tuple<float, float> &t_w) :
        name(name),
        demand(demand),
        time_window(t_w) {
    if (std::get<0>(t_w) > std::get<1>(t_w)) {
        throw std::invalid_argument("Time window for " + name + " not valid: " +
                                    std::to_string(std::get<0>(t_w)) + " > " + std::to_string(std::get<1>(t_w)));
    }
}

Node::Node(const Node &other) :
    name(other.name),
    demand(other.demand),
    time_window(other.time_window) {
}

/** Get the name of this node */
std::string Node::get_name() const {
    return name;
}

/** Return the demand level of the node */
float Node::get_demand() const {
    return demand;
}

/**
  * Return what is loaded onto/off a vehicle servicing this node
  * (i.e., load is negative demand)
*/
float Node::get_load() const {
    return -demand;
}

/** Return the time window (a, b) of the node */
std::tuple<float, float> Node::get_window() const {
    return time_window;
}

std::ostream &operator<<(std::ostream &os, const Node &node) {
    os << node.name << ": " << node.demand << " in ("
       << std::get<0>(node.time_window) << ", "
       << std::get<1>(node.time_window) << ")";
    return os;
}

Node::Node() {}

/**
  * An arc goes from one node to another (distinct) node
  * It has an associated travel time, and potentially a cost
*/
Arc::Arc(Node *origin, Node *destination, float travelTime, float cost) :
        origin(origin), destination(destination), travel_time(travel_time), cost(cost) {
}

Node *Arc::get_origin() const {
    return origin;
}

Node *Arc::get_destination() const {
    return destination;
}

float Arc::get_travel_time() const {
    return travel_time;
}

float Arc::get_cost() const {
    return cost;
}

std::ostream &operator<<(std::ostream &os, const Arc &arc) {
    os << arc.origin->get_name() << " to " << arc.destination->get_name()
       << ", t=" << arc.travel_time;
    return os;
}

/**
A class to organize the basic data of a
        Vehicle Routing Problem with Time Windows (VRPTW) \n
This includes the graph structure of the problem and vehicle sizing. \n

This formulation of the problem is based on
M. Desrochers, J. Desrosiers, and M. Solomon, "A new optimization algorithm
for the vehicle routing problem with time windows"
https://doi.org/10.1287/opre.40.2.342
*/

VehicleRoutingProblemWithTimeWindows::VehicleRoutingProblemWithTimeWindows(
        const std::vector<std::string> &nodeNames,
        const std::vector<Node *> &nodes,
        const std::map<std::pair<int, int>, Arc *> &arcs,
        int depot_index,
        float vehicle_capacity,
        float initial_loading) :
        node_names(nodeNames), nodes(nodes), arcs(arcs),
        depot_index(depot_index), vehicle_capacity(vehicle_capacity),
        initial_loading(initial_loading) {
}

VehicleRoutingProblemWithTimeWindows::VehicleRoutingProblemWithTimeWindows() {}

VehicleRoutingProblemWithTimeWindows::VehicleRoutingProblemWithTimeWindows(
        const VehicleRoutingProblemWithTimeWindows &other) {
    vehicle_capacity = other.vehicle_capacity;
    initial_loading = other.initial_loading;
    depot_index = other.depot_index;

    node_names = other.node_names;
    for (const auto *node : other.nodes) {
        nodes.push_back(new Node(*node));
    }

    for (const auto &arc_pair : other.arcs) {
        int originIdx = -1, destIdx = -1;
        for (size_t i = 0; i < nodes.size(); i++) {
            if (nodes[i] -> get_name() == arc_pair.second -> get_origin()-> get_name()) {
                originIdx = i;
            }
            if (nodes[i] -> get_name() == arc_pair.second -> get_destination() -> get_name()) {
                destIdx = i;
            }
        }

        if (originIdx >= 0 && destIdx >= 0) {
            auto* newArc = new Arc(nodes[originIdx], nodes[destIdx],
                                   arc_pair.second -> get_travel_time(),
                                   arc_pair.second -> get_cost()
                                   );
            arcs[std::make_pair(originIdx, destIdx)] = newArc;
        }
    }
}

VehicleRoutingProblemWithTimeWindows::~VehicleRoutingProblemWithTimeWindows() {
    for (auto node: nodes) {
        delete node;
    }

    for (auto &arc_pair: arcs) {
        delete arc_pair.second;
    }
}

const std::vector<std::string> &VehicleRoutingProblemWithTimeWindows::get_node_names() const {
    return node_names;
}

const std::vector<Node *> &VehicleRoutingProblemWithTimeWindows::get_nodes() const {
    return nodes;
}

const std::map<std::pair<int, int>, Arc *> &VehicleRoutingProblemWithTimeWindows::get_arcs() const {
    return arcs;
}

int VehicleRoutingProblemWithTimeWindows::get_depot_index() const {
    return depot_index;
}

float VehicleRoutingProblemWithTimeWindows::get_vehicle_capacity() const {
    return vehicle_capacity;
}

float VehicleRoutingProblemWithTimeWindows::get_initial_loading() const {
    return initial_loading;
}

void VehicleRoutingProblemWithTimeWindows::set_vehicle_capacity(float vehicleCapacity) {
    VehicleRoutingProblemWithTimeWindows::vehicle_capacity = vehicleCapacity;
}

void VehicleRoutingProblemWithTimeWindows::set_initial_loading(float loading) {
    VehicleRoutingProblemWithTimeWindows::initial_loading = loading;
}

void VehicleRoutingProblemWithTimeWindows::add_node(
        const std::string &node_name,
        float demand,
        const std::tuple<float, float> &t_w) {
    if (std::find(node_names.begin(), node_names.end(), node_name) != node_names.end()) {
        throw std::invalid_argument(node_name + " is already in Node list");
    }
    nodes.push_back(new Node(node_name, demand, t_w));
    node_names.push_back(node_name);
}

int VehicleRoutingProblemWithTimeWindows::get_node_index(const std::string &node_name) const {
    auto it = std::find(node_names.begin(), node_names.end(), node_name);
    if (it == node_names.end()) {
        throw std::invalid_argument("Node name not found: " + node_name);
    }
    return std::distance(node_names.begin(), it);
}

Node *VehicleRoutingProblemWithTimeWindows::get_node(const std::string &nodeName) const {
    return nodes[get_node_index(nodeName)];
}

void VehicleRoutingProblemWithTimeWindows::set_depot(const std::string &depot_name) {
    int d_index = get_node_index(depot_name);

    // Check if depot time window is finite
    std::tuple<float, float> window = nodes[d_index]->get_window();
    if (!std::isinf(std::get<1>(window))) {
        std::cout << "Warning: Consider making Depot time window infinite in size..." << std::endl;
    }

    if (d_index == 0) {
        return;
    }

    // Move depot to front
    Node *depot = nodes[d_index];
    nodes.erase(nodes.begin() + d_index);
    node_names.erase(node_names.begin() + d_index);

    nodes.insert(nodes.begin(), depot);
    node_names.insert(node_names.begin(), depot_name);
}

bool VehicleRoutingProblemWithTimeWindows::add_arc(
        std::string originName,
        std::string destinationName,
        float travelTime,
        float cost) {
    int i = get_node_index(originName);
    int j = get_node_index(destinationName);

    float departureTime = std::get<0>(nodes[i]->get_window());
    if ((departureTime + travelTime) <= std::get<1>(nodes[j]->get_window())) {
        arcs[std::make_pair(i, j)] = new Arc(nodes[i], nodes[j], travelTime, cost);
        return true;
    }

    return false;
}

int VehicleRoutingProblemWithTimeWindows::estimate_max_vehicles() const {
    int num_outgoing = 0;
    int num_incoming = 0;

    for (auto &arc: arcs) {
        if (arc.first.first == depot_index) {
            num_outgoing++;
        }
        if (arc.first.second == depot_index) {
            num_incoming++;
        }
    }

    return std::min(num_outgoing, num_incoming);
}

std::ostream &operator<<(std::ostream &os, const VehicleRoutingProblemWithTimeWindows &vrptw) {
    os << "Depot index: " << vrptw.depot_index << std::endl;
    os << "Vehicle Capacity: " << vrptw.vehicle_capacity << std::endl;
    os << "Initial loading: " << vrptw.initial_loading << std::endl;

    os << "Nodes:" << std::endl;
    for (const auto &node: vrptw.nodes) {
        os << *node << std::endl;
    }

    os << "Arcs:" << std::endl;
    for (const auto &arc_pair: vrptw.arcs) {
        os << *(arc_pair.second) << std::endl;
    }

    return os;
}
