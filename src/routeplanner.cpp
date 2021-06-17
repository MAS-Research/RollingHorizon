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

#include "../headers/formatting.hpp"
#include "../headers/routeplanner.hpp"
#include "../headers/settings.hpp"

#include <algorithm>
#include <fstream>
#include <iostream>
#include <map>
#include <math.h>
#include <mutex>
#include <set>


using namespace std;

namespace routeplanner
{

int const LP_LIMITVALUE = 8;

struct MetaNodeStop
{
    NodeStop* node;
    vector<MetaNodeStop*> unlocks;
    
    friend bool operator<(MetaNodeStop const & a, MetaNodeStop const & b)
    {
        if (a.node->node < b.node->node) // First sort criterion is node.
            return true;
        if (a.node->node > b.node->node)
            return false;
        if (a.node->is_pickup < b.node->is_pickup)  // Second by alightings before boardings.
            return true;
        if (a.node->is_pickup > b.node->is_pickup)
            return false;
        if (a.node < b.node || (a.node == b.node && &a.unlocks < &b.unlocks))
            return true;
        else
            return false;
    }
};

struct MnsSort
{
    bool operator ()(MetaNodeStop const * a, MetaNodeStop const * b) const
    {
       return *a < *b;
    }
};

pair<int,vector<NodeStop>> format_path(pair<int,vector<NodeStop*>> const & reverse_path, int time)
{
    int cost = reverse_path.first;
    if (CTSP_OBJECTIVE == CTSP_VMT && cost >= 0) // Use this with VMT objective only.
        cost -= time;
    vector<NodeStop> ordered_rs;
    for (auto i = reverse_path.second.size(); i--;)
        ordered_rs.push_back(*reverse_path.second[i]);
    
    // Format the response.
    return make_pair(cost, ordered_rs);
}

int get_alight_deadline(Request const * r)
{
    return r->entry_time + r->ideal_traveltime + MAX_DETOUR;
}

enum Action {PICKUP, DROPOFF, NO_ACTION};

pair<int,vector<NodeStop*>> recursive_search(int initial_location, int residual_capacity,
        set<MetaNodeStop*,MnsSort> const & initially_available, Network const & network, int time, int best_time,
        Action prev_action)
{
    // If there is no new available stop to add...
    if (!initially_available.size())
        return make_pair(time, vector<NodeStop*>()); // VMT objective
    
    // Iterate through the possible next NodeStops to visit.
    vector<NodeStop*> best_tail;
    MetaNodeStop* previous = NULL;
    for (MetaNodeStop* m : initially_available)
    {
        // Select the node.  Strict order on alightings.
        if (previous != NULL && !m->node->is_pickup && previous->node->node == m->node->node)
            continue;
        previous = m;
        
        // Compute time of visit.
        int new_location = m->node->node;
        int arrival_time = time + network.get_time(initial_location, new_location);
        if (m->node->is_pickup)
            if (m->node->r->entry_time > arrival_time)
                arrival_time = m->node->r->entry_time;
        
        // Account for rule about batched boarding/alighting.  Must match simulator behavior.
        if (prev_action == DROPOFF && (m->node->is_pickup || initial_location != new_location))
            arrival_time += DWELL_ALIGHT;
        else if (prev_action == PICKUP && (!m->node->is_pickup || initial_location != new_location))
            arrival_time += DWELL_PICKUP;
        if (m->node->is_pickup && m->node->r->entry_time > arrival_time)
            arrival_time = m->node->r->entry_time;
        
        // Skip if this violates the time bound.
        if (best_time != -1 && arrival_time >= best_time) // Use for VMT objective
            continue;
        
        // Assert this satisfies the capacity constraints.
        int new_residual_capacity = residual_capacity;
        if (m->node->is_pickup)
            new_residual_capacity--;
        else
            new_residual_capacity++;
        if (new_residual_capacity < 0)
            continue;
        
        // Assert this is in the acceptable time range.
        if (m->node->is_pickup && arrival_time > m->node->r->entry_time + MAX_WAITING)
            continue;
        if (get_alight_deadline(m->node->r) < arrival_time)
            continue;
        
        // Create the new availability list, considering unlocked nodes.f
        set<MetaNodeStop*,MnsSort> remaining_nodes;
        for (auto x : initially_available)
            if (x != m)
                remaining_nodes.insert(x);
        if (m->unlocks.size())
            for (auto newnode : m->unlocks)
                remaining_nodes.insert(newnode);
        
        // Basic check: can subsequent nodes be served?
        bool basic_reachability = true;
        for (auto x : remaining_nodes)
        {
            int reaching_time = arrival_time + network.get_time(new_location, x->node->node);
            if ((x->node->is_pickup && reaching_time > x->node->r->latest_boarding) ||
                    (!x->node->is_pickup && reaching_time > x->node->r->latest_alighting))
            {
                basic_reachability = false;
                break;
            }
        }
        if (!basic_reachability)
            continue;
        
        // Recursive call to get cost, partial reverse path of tail.
        Action this_action = (m->node->is_pickup ? PICKUP : DROPOFF);
        pair<int,vector<NodeStop*>> tail = recursive_search(new_location, new_residual_capacity,
                remaining_nodes, network, arrival_time, best_time, this_action);
        
        // If this is the best we have seen so far, update!
        if (tail.first == -1)
            continue;
        if (best_time == -1 || tail.first < best_time)
        {
            best_time = tail.first;
            best_tail = tail.second;
            best_tail.push_back(m->node);
        }
    }
    
    return make_pair(best_time, best_tail);
}

pair<int,vector<NodeStop*>> recursive_search(int initial_location, int residual_capacity,
        set<MetaNodeStop*> const & initially_available, Network const & network, int time, int best_time)
{
    set<MetaNodeStop*,MnsSort> update (initially_available.begin(), initially_available.end());
    return recursive_search(initial_location, residual_capacity, update, network, time, best_time, NO_ACTION);
}

pair<int,vector<NodeStop>> rebalance(Vehicle const & v, vector<Request*> const & rs, Network const & network)
{
    if (v.passengers.size())
    {
        cout << "RoutePlanner Error line " << __LINE__ << ". Non-empty vehicle rebalancing." << endl;
        getchar();
    }
    vector<NodeStop> nodes;
    nodes.push_back({rs[0], true, rs[0]->origin});
    nodes.push_back({rs[0], false, rs[0]->destination});
    int cost = network.get_vehicle_time(v, rs[0]->origin) + network.get_time(rs[0]->origin, rs[0]->destination);
    return make_pair(cost, nodes);
}

pair<int,vector<NodeStop>> new_travel(Vehicle const & v, vector<Request*> const & rs,
        Network const & network, int time)
{
    // Convert onboard passengers and new ones into NodeStops and MetaNodeStops.
    vector<NodeStop> nodes;                                 // Wrapper for locations.
    vector<MetaNodeStop> meta_nodes;                        // Object that store precedence.
    set<MetaNodeStop*> initially_available;                 // Place you can visit without precedence.
    nodes.reserve(2 * rs.size() + v.passengers.size());
    meta_nodes.reserve(2 * rs.size() + v.passengers.size());
    
    for (auto r : rs)
    {
        nodes.push_back({r, true, r->origin});
        nodes.push_back({r, false, r->destination});
        meta_nodes.push_back({&nodes[nodes.size() - 1], vector<MetaNodeStop*>()});
        meta_nodes.push_back({&nodes[nodes.size() - 2], 
                vector<MetaNodeStop*>{&meta_nodes[meta_nodes.size() - 1]}});
        initially_available.insert(&meta_nodes[meta_nodes.size() - 1]);
    }
    set<Request*> onboard;
    for (auto r : v.passengers)
        onboard.insert(r);
    for (auto & ns : v.order_record)
    {
        if (onboard.count(ns.r))
        {
            nodes.push_back(ns);
            meta_nodes.push_back({&nodes[nodes.size() - 1], vector<MetaNodeStop*>()});
            onboard.erase(ns.r);
        }
    }
    if (CTSP == FIX_ONBOARD && rs.size() + v.passengers.size() > 4 && v.passengers.size())
    {
        for (auto i = 0; i < v.passengers.size() - 1; i++)
            meta_nodes[meta_nodes.size() - 2 - i].unlocks = 
                    vector<MetaNodeStop*> {&meta_nodes[meta_nodes.size() - 1 - i]};
        initially_available.insert(&meta_nodes[meta_nodes.size() - v.passengers.size()]);
    }
    else
        for (auto i = 0; i < v.passengers.size(); i++)
            initially_available.insert(&meta_nodes[meta_nodes.size() - 1 - i]);
    
    // Consider recomputing the initially available set to save time, if threshold is exceeded.
    if (CTSP == FIX_PREFIX && meta_nodes.size() > LP_LIMITVALUE)
    {
        // First let's determine which requests are not available in the previous ordering.
        set<Request*> previous_requests (v.pending_requests.begin(), v.pending_requests.end());
        set<Request*> new_requests;
        for (auto r : rs)
            if (!previous_requests.count(r))
                new_requests.insert(r);
        
        if (2 * new_requests.size() > LP_LIMITVALUE)  // There are too many to process.  Reject.
            return make_pair(-1, vector<NodeStop>());
        
        // Now we must run the processing step.  First get an ordered list from last time.
        map<NodeStop,MetaNodeStop*> node_to_meta;
        for (auto & m : meta_nodes)
            node_to_meta[*m.node] = &m;
        vector<MetaNodeStop*> previous_order;
        for (auto & ns : v.order_record)
            if (node_to_meta.count(ns))
                previous_order.push_back(node_to_meta[ns]);
        
        if (previous_order.size() < meta_nodes.size() - LP_LIMITVALUE)
            throw runtime_error("The algebra here was done incorrectly!");
        
        // Now initialize the states.
        set<MetaNodeStop*> captured = initially_available;
        initially_available = set<MetaNodeStop*> {previous_order[0]};
        
        // Now run the algorithm.
        for (auto i = 0; i < meta_nodes.size() - LP_LIMITVALUE; i++)
        {
            captured.erase(previous_order[i]);  // These two steps update what is captured.
            for (auto m : previous_order[i]->unlocks)
                captured.insert(m);
            if (i + 1 < meta_nodes.size() - LP_LIMITVALUE) // These steps decide what is unlocked.
                previous_order[i]->unlocks = vector<MetaNodeStop*> {previous_order[i + 1]};
            else
                previous_order[i]->unlocks = vector<MetaNodeStop*>(captured.begin(), captured.end());
        }
    }
    
    // Call the recursive cost function.
    int call_time = time + v.offset;
    int start_node = v.node;
    pair<int, vector<NodeStop*>> optimal;
    if (CTSP_OBJECTIVE == CTSP_VMT)
        optimal = recursive_search(start_node, v.capacity - v.passengers.size(),
                initially_available, network, call_time, -1);
    else
        throw runtime_error("No valid CTSP objective selected.");
    
    return format_path(optimal, time);
}

pair<int,vector<NodeStop>> memory(Vehicle const & v, Network const & network, int time)
{
    // Create meta nodes, with unlocking sequence forcing the order from memory.
    vector<NodeStop> nodes;
    vector<MetaNodeStop> meta_nodes;
    nodes.reserve(v.order_record.size());
    for (auto & ns : v.order_record)
    {
        nodes.push_back(ns);
        meta_nodes.push_back(MetaNodeStop{&nodes[nodes.size() - 1], vector<MetaNodeStop*>()});
    }
    set<MetaNodeStop*> initially_available;
    if (meta_nodes.size())
        initially_available.insert(&meta_nodes[0]);
    for (auto i = 1; i < meta_nodes.size(); i++)
        meta_nodes[i - 1].unlocks = vector<MetaNodeStop*> {&meta_nodes[i]};
    
    // Call the recursive cost function.
    int call_time = time + v.offset;
    int start_node = v.node;
    pair<int, vector<NodeStop*>> optimal;
    if (CTSP_OBJECTIVE == CTSP_VMT)
        optimal = recursive_search(start_node, v.capacity - v.passengers.size(),
            initially_available, network, call_time, -1);
    else
        throw runtime_error("No valid CTSP objective selected.");
    return format_path(optimal, time);
}

// The implementation of Travel() function
pair<int,vector<NodeStop>> travel(Vehicle const & vehicle, vector<Request*> const & requests,
        Purpose trigger, Network const & network, int time)
{
    if (trigger == MEMORY)
        return memory(vehicle, network, time);
    else if (trigger == REBALANCING)
        return rebalance(vehicle, requests, network);
    else
        return new_travel(vehicle, requests, network, time);
}











pair<int,vector<NodeStop*>> recursive_search_timed(int initial_location, int residual_capacity,
        set<MetaNodeStop*,MnsSort> const & initially_available, Network const & network, int time, int best_time,
        chrono::steady_clock::time_point t)
{
    // If there is no new available stop to add...
    if (!initially_available.size())
        return make_pair(time, vector<NodeStop*>()); // VMT objective
    
    // Iterate through the possible next NodeStops to visit.
    vector<NodeStop*> best_tail;
    MetaNodeStop* previous = NULL;
    for (MetaNodeStop* m : initially_available)
    {
        // Check for a timeout...
        if (RTV_TIMELIMIT)
        {
            auto end_time = chrono::steady_clock::now();
            auto duration = chrono::duration_cast<chrono::milliseconds>
                    (end_time - t).count();
            if (duration > RTV_TIMELIMIT)
                break;
        }
        
        // Select the node.  Strict order on alightings.
        if (previous != NULL && !m->node->is_pickup && previous->node->node == m->node->node)
            continue;
        previous = m;
        
        // Compute time of visit.
        int new_location = m->node->node;
        int arrival_time = time + network.get_time(initial_location, new_location);
        if (m->node->is_pickup)
            if (m->node->r->entry_time > arrival_time)
                arrival_time = m->node->r->entry_time;
        
        // Skip if this violates the time bound.
        if (best_time != -1 && arrival_time >= best_time) // Use for VMT objective
            continue;
        
        // Assert this satisfies the capacity constraints.
        int new_residual_capacity = residual_capacity;
        if (m->node->is_pickup)
            new_residual_capacity--;
        else
            new_residual_capacity++;
        if (new_residual_capacity < 0)
            continue;
        
        // Assert this is in the acceptable time range.
        if (m->node->is_pickup && arrival_time > m->node->r->entry_time + MAX_WAITING)
            continue;
        if (get_alight_deadline(m->node->r) < arrival_time)
            continue;
        
        // Create the new availability list, considering unlocked nodes.f
        set<MetaNodeStop*,MnsSort> remaining_nodes;
        for (auto x : initially_available)
            if (x != m)
                remaining_nodes.insert(x);
        if (m->unlocks.size())
            for (auto newnode : m->unlocks)
                remaining_nodes.insert(newnode);
        
        // Basic check: can subsequent nodes be served?
        bool basic_reachability = true;
        for (auto x : remaining_nodes)
        {
            int reaching_time = arrival_time + network.get_time(new_location, x->node->node);
            if ((x->node->is_pickup && reaching_time > x->node->r->latest_boarding) ||
                    (!x->node->is_pickup && reaching_time > x->node->r->latest_alighting))
            {
                basic_reachability = false;
                break;
            }
        }
        if (!basic_reachability)
            continue;
        
        // Recursive call to get cost, partial reverse path of tail.
        pair<int,vector<NodeStop*>> tail = recursive_search_timed(new_location, new_residual_capacity,
                remaining_nodes, network, arrival_time, best_time, t);
        
        // If this is the best we have seen so far, update!
        if (tail.first == -1)
            continue;
        if (best_time == -1 || tail.first < best_time)
        {
            best_time = tail.first;
            best_tail = tail.second;
            best_tail.push_back(m->node);
        }
    }
    
    return make_pair(best_time, best_tail);
}

pair<int,vector<NodeStop*>> recursive_search_timed(int initial_location, int residual_capacity,
        set<MetaNodeStop*> const & initially_available, Network const & network, int time, int best_time,
        chrono::steady_clock::time_point t)
{
    set<MetaNodeStop*,MnsSort> update (initially_available.begin(), initially_available.end());
    return recursive_search_timed(initial_location, residual_capacity, update, network, time, best_time, t);
}

pair<int,vector<NodeStop>> new_time_travel(Vehicle const & v, vector<Request*> const & rs,
        Network const & network, int time, chrono::steady_clock::time_point t)
{
    // Convert onboard passengers and new ones into NodeStops and MetaNodeStops.
    vector<NodeStop> nodes;                                 // Wrapper for locations.
    vector<MetaNodeStop> meta_nodes;                        // Object that store precedence.
    set<MetaNodeStop*> initially_available;              // Place you can visit without precedence.
    nodes.reserve(2 * rs.size() + v.passengers.size());
    meta_nodes.reserve(2 * rs.size() + v.passengers.size());
    
    for (auto r : rs)
    {
        nodes.push_back({r, true, r->origin});
        nodes.push_back({r, false, r->destination});
        meta_nodes.push_back({&nodes[nodes.size() - 1], vector<MetaNodeStop*>()});
        meta_nodes.push_back({&nodes[nodes.size() - 2], 
                vector<MetaNodeStop*>{&meta_nodes[meta_nodes.size() - 1]}});
        initially_available.insert(&meta_nodes[meta_nodes.size() - 1]);
    }
    set<Request*> onboard;
    for (auto r : v.passengers)
        onboard.insert(r);
    for (auto & ns : v.order_record)
    {
        if (onboard.count(ns.r))
        {
            nodes.push_back(ns);
            meta_nodes.push_back({&nodes[nodes.size() - 1], vector<MetaNodeStop*>()});
            onboard.erase(ns.r);
        }
    }
    if (CTSP == FIX_ONBOARD && rs.size() + v.passengers.size() > 4 && v.passengers.size())
    {
        for (auto i = 0; i < v.passengers.size() - 1; i++)
            meta_nodes[meta_nodes.size() - 2 - i].unlocks = 
                    vector<MetaNodeStop*> {&meta_nodes[meta_nodes.size() - 1 - i]};
        initially_available.insert(&meta_nodes[meta_nodes.size() - v.passengers.size()]);
    }
    else
        for (auto i = 0; i < v.passengers.size(); i++)
            initially_available.insert(&meta_nodes[meta_nodes.size() - 1 - i]);
    
    // Consider recomputing the initially available set to save time, if threshold is exceeded.
    if (CTSP == FIX_PREFIX && meta_nodes.size() > LP_LIMITVALUE)
    {
        // First let's determine which requests are not available in the previous ordering.
        set<Request*> previous_requests (v.pending_requests.begin(), v.pending_requests.end());
        set<Request*> new_requests;
        for (auto r : rs)
            if (!previous_requests.count(r))
                new_requests.insert(r);
        
        if (2 * new_requests.size() > LP_LIMITVALUE)  // There are too many to process.  Reject.
            return make_pair(-1, vector<NodeStop>());
        
        // Now we must run the processing step.  First get an ordered list from last time.
        map<NodeStop,MetaNodeStop*> node_to_meta;
        for (auto & m : meta_nodes)
            node_to_meta[*m.node] = &m;
        vector<MetaNodeStop*> previous_order;
        for (auto & ns : v.order_record)
            if (node_to_meta.count(ns))
                previous_order.push_back(node_to_meta[ns]);
        
        if (previous_order.size() < meta_nodes.size() - LP_LIMITVALUE)
            throw runtime_error("The algebra here was done incorrectly!");
        
        // Now initialize the states.
        set<MetaNodeStop*> captured = initially_available;
        initially_available = set<MetaNodeStop*> {previous_order[0]};
        
        // Now run the algorithm.
        for (auto i = 0; i < meta_nodes.size() - LP_LIMITVALUE; i++)
        {
            captured.erase(previous_order[i]);  // These two steps update what is captured.
            for (auto m : previous_order[i]->unlocks)
                captured.insert(m);
            if (i + 1 < meta_nodes.size() - LP_LIMITVALUE) // These steps decide what is unlocked.
                previous_order[i]->unlocks = vector<MetaNodeStop*> {previous_order[i + 1]};
            else
                previous_order[i]->unlocks = vector<MetaNodeStop*>(captured.begin(), captured.end());
        }
    }
    
    // Call the recursive cost function.
    int call_time = time + v.offset;
    int start_node = v.node;
    pair<int, vector<NodeStop*>> optimal;
    if (CTSP_OBJECTIVE == CTSP_VMT)
        optimal = recursive_search_timed(start_node, v.capacity - v.passengers.size(),
                initially_available, network, call_time, -1, t);
    else
        throw runtime_error("No valid CTSP objective selected for timed feature.");
    
    return format_path(optimal, time);
}


pair<int,vector<NodeStop>> time_travel(Vehicle const & vehicle, vector<Request*> const & requests,
        Purpose trigger, Network const & network, int time, chrono::steady_clock::time_point t)
{
    if (!RTV_TIMELIMIT)
        return travel(vehicle, requests, trigger, network, time);
    if (trigger != STANDARD)
        throw runtime_error("Received a trigger type that is not valid for \"routeplanner::time_travel\".");
    return new_time_travel(vehicle, requests, network, time, t);
}

}
