//
// Created by Truong on 31/03/2025.
//

#ifndef VEHICLE_ROUTING_PROBLEM_WITH_TIME_WINDOW_H
#define VEHICLE_ROUTING_PROBLEM_WITH_TIME_WINDOW_H

#ifdef VEHICLE_ROUTING_PROBLEM_AS_QUBO_EXPORTS
#define VEHICLE_ROUTING_PROBLEM_AS_QUBO_API __declspec(dllexport)
#else
#define VEHICLE_ROUTING_PROBLEM_AS_QUBO_API __declspec(dllimport)
#endif

#include <string>
#include <map>
#include <vector>
#include <tuple>
#include <limits>
#include <iostream>

class VEHICLE_ROUTING_PROBLEM_AS_QUBO_API Node {
private:
    std::string name;
    float demand;
    std::tuple<float, float> time_window;

public:
    Node(const std::string &name, float demand, const std::tuple<float, float> &t_w);

    Node(const Node& other);

    Node();

    std::string get_name() const;
    float get_demand() const;
    float get_load() const;
    std::tuple<float, float> get_window() const;

    friend std::ostream &operator<<(std::ostream &os, const Node &node);
};

class VEHICLE_ROUTING_PROBLEM_AS_QUBO_API Arc {
private:
    Node *origin;
    Node *destination;
    float travel_time;
    float cost;

public:
    Arc(Node *origin, Node *destination, float travel_time, float cost);

    Node *get_origin() const;
    Node *get_destination() const;
    float get_travel_time() const;
    float get_cost() const;

    friend std::ostream &operator<<(std::ostream &os, const Arc &arc);
};

class VEHICLE_ROUTING_PROBLEM_AS_QUBO_API VehicleRoutingProblemWithTimeWindows {
private:
    std::vector<std::string> node_names;
    std::vector<Node*> nodes;
    std::map<std::pair<int, int>, Arc *> arcs;
    int depot_index = 0;
    float vehicle_capacity = 0.0f;
    float initial_loading = 0.0f;

public:
    VehicleRoutingProblemWithTimeWindows(const std::vector<std::string> &node_names,
                                         const std::vector<Node *> &nodes,
                                         const std::map<std::pair<int, int>, Arc *> &arcs,
                                         int depot_index,
                                         float vehicle_capacity,
                                         float initial_loading);

    VehicleRoutingProblemWithTimeWindows();

    VehicleRoutingProblemWithTimeWindows(const VehicleRoutingProblemWithTimeWindows& other);

    virtual ~VehicleRoutingProblemWithTimeWindows();

    const std::vector<std::string> &get_node_names() const;
    const std::vector<Node *> &get_nodes() const;
    const std::map<std::pair<int, int>, Arc *> &get_arcs() const;
    int get_depot_index() const;
    float get_vehicle_capacity() const;
    float get_initial_loading() const;

    void set_vehicle_capacity(float vehicleCapacity);
    void set_initial_loading(float loading);

    void add_node(const std::string &node_name,
                  float demand,
                  const std::tuple<float, float> &t_w = std::make_pair(0.0f, std::numeric_limits<float>::infinity()));

    int get_node_index(const std::string &node_name) const;
    Node *get_node(const std::string &node_name) const;
    void set_depot(const std::string &depot_name);

    bool add_arc(std::string originName,
                std::string destinationName,
                float travelTime,
                float cost = 0);

    int estimate_max_vehicles() const;

    friend std::ostream &operator<<(std::ostream &os, const VehicleRoutingProblemWithTimeWindows &vrptw);
};

#endif // VEHICLE_ROUTING_PROBLEM_WITH_TIME_WINDOW_H
