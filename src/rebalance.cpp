/*
 * The MIT License
 *
 * Copyright 2018 Vindula Jayawardana and Matthew Zalesak.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include "rebalance.hpp"
#include "routeplanner.hpp"

#include "fusion.h"             // For MOSEK functions.
#include <set>

using namespace mosek::fusion;  // For MOSEK functions.
using namespace monty;          // For MOSEK functions.
using namespace std;

namespace rebalance
{

map<Vehicle*,Trip> rebalance_matching_lp(
        map<Vehicle*,vector<Trip>> const & trips_map, vector<Request*> const & requests)
{
    if (trips_map.size() == 0)
        return map<Vehicle*,Trip>();
    
    int R = requests.size(), V = trips_map.size();
    int match_count = min(R, V);
    
    cout <<  "R = " << R << ", V = " << V << endl;
    
    // This is how Mosek says to create a model.
    Model::t M = new Model("rebalanceR"); auto _M = finally([&]() { M->dispose(); });
    Variable::t x = M->variable("x", new_array_ptr<int, 1>({V, R}), Domain::binary());
    
    // Constraint one : make correct number of matches.
    M->constraint("c1", Expr::sum(x), Domain::equalsTo(double(match_count)));
    
    // Constraints two and three : choose each vehicle and request at most once.
    M->constraint("c2", Expr::sum(x, 0), Domain::lessThan(1.0));
    M->constraint("c3", Expr::sum(x, 1), Domain::lessThan(1.0));
    
    // Objective function.
    auto costs = make_shared<ndarray<double, 2>>(shape(V, R));
    {
        int v = 0;
        for (auto &vehicle_to_trips : trips_map)
        {
            vector<Trip> trips_list = vehicle_to_trips.second;
            int r = 0;
            for (auto &trip : trips_list)
            {
                (*costs)[v * R + r] = trip.cost;
                r ++;
            }
            v ++;
        }
    }
    
    M->objective("obj", ObjectiveSense::Minimize, Expr::dot(costs, x));
    
    // Set maximum solution time, relative gap, and absolute gap paramters.
    M->setSolverParam("mioMaxTime", 20);
    M->setSolverParam("mioTolRelGap", 1e-1);  // 1e-4 is default value.
    M->setSolverParam("mioTolAbsGap", 0.0);   // 0.0  is default value.
    
    // Solve.
    M->solve();
    
    auto solution = x->level();
    vector<int> decision(R * V);
    for (int v = 0; v < V; v ++)
        for (int r = 0; r < R; r ++)
            decision[v * R + r] = (*solution)[v * R + r] > 0.5;
    
    map<Vehicle*,Trip> rebalancing_trips;
    
    int count = 0;
    for (auto & x : trips_map)
    {
        Vehicle* v = x.first;
        vector<Trip> const* trips = &x.second;
        
        for (int r = 0; r < trips->size(); r++)
            if (decision[r + count] > 0.5)
            {
                rebalancing_trips[v] = (*trips)[r];
                break;
            }
        
        count += trips->size();
    }
    
    return rebalancing_trips;
}

map<Vehicle*, vector<Trip>> make_rebalance_trips(vector<Vehicle*> & unassigned_vehicles, 
        vector<Request*> & unassigned_requests, Network const & network)
{
    map<Vehicle*, vector<Trip>> trip_list;
    for (Vehicle* v : unassigned_vehicles)
    {
        vector<Trip> trips;
        for (Request* r : unassigned_requests)
        {
            Trip t {};
            t.is_fake = true;
            t.requests.push_back(r);
            t.cost = network.get_vehicle_time(*v, r->origin);
            trips.push_back(t);
        }
        trip_list[v] = trips;
    }
    return trip_list;
}

map<Vehicle*,Trip> make_rebalance(
        map<Vehicle*,Trip> const & assigned_trips,
        vector<Vehicle*> & active_vehicles,
        vector<Request*> & active_requests,
        map<Vehicle*,Request> & dummy_request_store,
        Network const & network)
{
    // Compute unassigned vehicles [stopped, not allocated]
    vector<Vehicle*> unassigned_vehicles;
    for (Vehicle* v : active_vehicles)  // Unassigned vehicles have no passengers and no assignments.
        if (v->passengers.size() == 0 && !assigned_trips.count(v) && v->rebalance_target == -1)
            unassigned_vehicles.push_back(v);
    
    // Compute unassigned requests
    vector<Request*> unassigned_requests;
    {
        set<Request*> final_assigned_requests;
        for (auto & t : assigned_trips)
            for (Request* r : t.second.requests)
                final_assigned_requests.insert(r);
        
        for (Request* r : active_requests)
            if (!final_assigned_requests.count(r))
                unassigned_requests.push_back(r);
        
        for (Request* r : final_assigned_requests)  // For statistics collection.
            r->assigned = true;
    }

    map<Vehicle*, vector<Trip>> possible_trips; // TODO: Why is this scoped like this?
    map<Vehicle*,Trip> rebalancing_trips;
    if (unassigned_vehicles.size() != 0 && unassigned_requests.size() != 0)
    {
        possible_trips = make_rebalance_trips(unassigned_vehicles, unassigned_requests, network);
        rebalancing_trips = rebalance_matching_lp(possible_trips, unassigned_requests);
    }

    // Rebalancing vehicles with no assignments should continue.
    for (auto v : active_vehicles)
    {
        int vid = v->id;
        if (v->passengers.size() == 0 && !assigned_trips.count(v) && v->rebalance_target != -1)
        {
            Request r {};
            r.id = -1;
            r.origin = v->rebalance_target;
            r.destination = v->rebalance_target;
            dummy_request_store[v] = r;
        }
    }
    for (auto & x : dummy_request_store)
    {
        Trip t {};
        t.is_fake = true;
        t.requests = {&x.second};
        rebalancing_trips[x.first] = t;
    }
    
    return rebalancing_trips;
}

}
