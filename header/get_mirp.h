//
// Created by ACER on 07/04/2025.
//

#ifndef VEHICLE_ROUTING_PROBLEM_AS_QUBO_GET_MIRP_H
#define VEHICLE_ROUTING_PROBLEM_AS_QUBO_GET_MIRP_H

#ifdef VEHICLE_ROUTING_PROBLEM_AS_QUBO_EXPORTS
#define VEHICLE_ROUTING_PROBLEM_AS_QUBO_API __declspec(dllexport)
#else
#define VEHICLE_ROUTING_PROBLEM_AS_QUBO_API __declspec(dllimport)
#endif

#include "marine_inventory_routing_problem.h"

VEHICLE_ROUTING_PROBLEM_AS_QUBO_API std::unique_ptr<MarineInventoryRoutingProblem> get_mirp(float time_horizon);

#endif //VEHICLE_ROUTING_PROBLEM_AS_QUBO_GET_MIRP_H
