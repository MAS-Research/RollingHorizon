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

#include "formatting.hpp"
#include "network.hpp"
#include "request.hpp"
#include "routeplanner.hpp"
#include "settings.hpp"
#include "simulator.hpp"

#include <fstream>
#include <iostream>
#include <mutex>
#include <set>
#include <sstream>
#include <vector>

using namespace std;

namespace simulator
{
mutex mtx;

/* Special case when a vehicle is finishing moving towards a goal with no riders. */
void move_jobless_vehicle(Vehicle & v, Network const & network, int time)
{
    int origin = v.prev_node;
    int destination = v.node;
    stringstream actions;
    
    if (v.offset <= INTERVAL)
    {
        actions << v.id << "," << encode_time(time + v.offset)
                << "," << destination << "," << endl;
        int distance = network.get_distance(origin, destination);
        v.add_distance(distance);
        v.prev_node = destination;
        v.offset = 0;
    }
    else
        v.offset -= INTERVAL;
    
    mtx.lock();
    {
        ofstream actionsfile (RESULTS_DIRECTORY + "/actions.log", std::ios_base::app);
        actionsfile << actions.rdbuf();
    }
    mtx.unlock();
    
    v.order_record.clear();
}


/* For vehicles given new assignments. */
void move_vehicle(Vehicle & vehicle, Trip const & trip, Network const & network, int time)
{
    vector<Request*> newRequests = trip.requests;
    set<Request*> pending_requests (newRequests.begin(), newRequests.end());

    Purpose trigger;
    if (trip.is_fake)
        trigger = REBALANCING;
    else if (trip.use_memory)
        trigger = MEMORY;
    else
        trigger = STANDARD;
    
    vehicle.rebalance_target = (trip.is_fake ? trip.requests[0]->origin : -1);
    
    int raw_cost;           // Cost of path.
    vector<NodeStop> path;
    if (!trip.order_record.size())
    {
        pair<int,vector<NodeStop>> travel_pair = routeplanner::travel(vehicle, 
                newRequests, trigger, network, time);
        raw_cost = travel_pair.first;
        path = travel_pair.second;
    }
    else
    {
        raw_cost = trip.cost;
        path = trip.order_record;
    }
    
    
    set<Request*> onboard(vehicle.passengers.begin(), vehicle.passengers.end());

    bool rebalancing = trip.is_fake;
    stringstream actions;
    
    if (raw_cost == -1) // If the simulation crashes, be sure to dump the state to file.
    {
        cout << "Beep." << endl;
        cout << "Path size " << path.size() << endl;
        cout << "This is an Error :: Simulator :: Line " << __LINE__ << endl;
        throw runtime_error("This is an error!");
    }

    bool interrupted = false;
    int traveltime_left = INTERVAL;
    int current_time = time;
    int jobs_completed = 0;
    
    // Set the vehicle state.
    if (!rebalancing && path.size() && !vehicle.passengers.size())
        vehicle.set_state(EnRoute, time);
    else if (rebalancing)
        vehicle.set_state(Rebalancing, time);
    
    // Let vehicle move towards its current destination.
    if (vehicle.offset < traveltime_left)
    {
        current_time += vehicle.offset;
        traveltime_left -= vehicle.offset;
        vehicle.offset = 0;
        vehicle.prev_node = vehicle.node;
        actions << vehicle.id << "," << encode_time(current_time) <<
                        "," << vehicle.node << "," << endl;
    }
    else
    {
        current_time += traveltime_left;
        vehicle.offset -= traveltime_left;
        traveltime_left = 0;
    }
    
    vector<int> latest_start(path.size()); // Time you must start traveling to each destination in path.
    vector<int> latest_execution; // The latest time each action must be completed by.
        vector<int> duration;
    if (LAST_MINUTE_SERVICE)
    {
        int tracking_time = current_time;
        
        int current_location = vehicle.node;
        for (auto & node : path)  // Compute durations and deadlines.
        {
            if (node.is_pickup)
                latest_execution.push_back(node.r->latest_boarding);
            else
                latest_execution.push_back(node.r->latest_alighting);
            duration.push_back(network.get_time(current_location, node.node));
            current_location = node.node;
        }
        
        for (auto i = latest_execution.size(); i--;) // Update deadlines from back, write start times.
        {
            latest_start[i] = latest_execution[i] - duration[i];
            if (i > 0)
                latest_execution[i - 1] = min(latest_execution[i - 1], latest_start[i]);
        }
    }
    
    for (auto x = 0; x < path.size() && traveltime_left > 0; x++) // While there are places for us to go...
    {
        Request* r = path[x].r;
        bool is_pickup = path[x].is_pickup;
        int target_node = path[x].node;
        
        if (LAST_MINUTE_SERVICE && !rebalancing) // If idle as long as possible
        {
            int delay = latest_start[x] - current_time;
            if (delay < 0)
                throw runtime_error("Delay for last minute service was negative!!! In simulator.");
            current_time += delay;
            traveltime_left -= delay;
        }
        
        // Actual nodes to visit as intermediaries.  Starts with origin node.
        vector<int> waypoints = network.dijkstra(vehicle.node, target_node);
        if (waypoints.size() == 1) // This always results in zero time travel! TODO : Remove this
        {
            int node = waypoints[0];
            vehicle.prev_node = node;
            vehicle.node = node;
            vehicle.offset = 0;
        }
        for (int w = 1; w < waypoints.size(); w++) // Only runs if multiple entries.
        {
            int origin = waypoints[w - 1];
            int destination = waypoints[w];
            
            int traveltime = network.get_time(origin, destination);
            vehicle.prev_node = origin;
            vehicle.node = destination;
            
            if (traveltime >= traveltime_left) // If too far...
            {
                interrupted = true;
                current_time += traveltime_left;
                vehicle.offset = traveltime - traveltime_left;
                traveltime_left = 0;
                break;
            }
            else // If we got there.
            {
                current_time += traveltime;
                traveltime_left -= traveltime;
                int distance = network.get_distance(origin, destination);
                vehicle.add_distance(distance);
                vehicle.prev_node = destination; // Since we got there!  Now they match.
                actions << vehicle.id << "," << 
                        encode_time(current_time) <<
                        "," << destination << "," << endl;
            }
        }
        
        if (interrupted)
            break;
        
        if (traveltime_left <= 0)
        {
            interrupted = true;  // Since this must be interrupted by waiting time.
            break;
        }

        
        // Now process waiting logic. 
        if (r->entry_time >= current_time)
        {
            int waiting_time = r->entry_time - current_time;
            if (waiting_time >= traveltime_left)
            {
                vehicle.prev_node = -30;
                interrupted = true;
                vehicle.offset = waiting_time - traveltime_left;
                break;
            }
            else
            {
                current_time += waiting_time;
                traveltime_left -= waiting_time;
            }
        }
 
        actions << vehicle.id << "," << encode_time(current_time) <<
                "," << target_node << ",W" << endl;

        // Now that we have arrived at the next destination, update the appropriate variables.
        jobs_completed++;
        char code;
        if (rebalancing)
            code = 'R';
        else if (is_pickup)
            code = 'P';
        else
            code = 'A';
        actions << vehicle.id << "," << encode_time(current_time) <<
                "," << target_node << "," << code << "R" << r->id << endl;
        
        // If this is end of rebalancing.
        if (rebalancing && target_node == newRequests[0]->origin)
        {
            vehicle.rebalance_target = -1;
            vehicle.set_state(Idle, current_time);
            break;
        }
             
        // Now process Dwell logic.  Pick up and drop off the passengers.
        if (!is_pickup)
        {
            r->alighting_time = current_time;
            vehicle.just_alighted.push_back(r);
            onboard.erase(r);
            if (!onboard.size())
                vehicle.set_state(Idle, current_time);
        }
        else
        {
            r->boarding_time = current_time;
            vehicle.just_boarded.push_back(r);
            pending_requests.erase(r);
            onboard.insert(r);
            vehicle.set_state(InUse, current_time);
            if (onboard.size() > 1)
                for (auto r : onboard)
                    r->shared = true;
        } 
        
        // This is the new batched dwell logic.  Must match routeplanner's expectations.
        int node;
        if (!is_pickup && (x + 1 == path.size() || path[x + 1].is_pickup || path[x + 1].node != target_node))
            node = -20; // DWELL_ALIGHT
        else if (is_pickup && (!path[x + 1].is_pickup || path[x + 1].node != target_node))
            node = -10; // DWELL_PICKUP.
        else
            node = target_node;
        int dwell = network.get_time(node, vehicle.node);
        if (dwell >= traveltime_left)
        {
            vehicle.prev_node = node;  // Note this is a dummy node -20 or -10, not real.
            interrupted = true;
            vehicle.offset = dwell - traveltime_left;
            break;
        }
        else
        {
            traveltime_left -= dwell;
            current_time += dwell;
        }
        actions << vehicle.id << "," << encode_time(current_time) <<
                "," << target_node << ",D" << endl;
    } // End visiting nodes section.
    
    // Transfer the final passenger list.
    vehicle.passengers = vector<Request*>(onboard.begin(), onboard.end());

    vehicle.order_record.clear();
    if (trigger != REBALANCING)
    {
        for (int i = jobs_completed; i < path.size(); i++)
            vehicle.order_record.push_back(path[i]);
        
        vehicle.pending_requests = vector<Request*> (pending_requests.begin(), pending_requests.end());
    }
    
    if (rebalancing)  // Temporary state to revert from rebalancing.  Remove when rebalancing fixed.
        vehicle.set_state(Idle, current_time);
    
    mtx.lock();
    {
        ofstream actionsfile (RESULTS_DIRECTORY + "/actions.log", std::ios_base::app);
        actionsfile << actions.rdbuf();
    }
    mtx.unlock();
}


void simulate_vehicle(Vehicle & vehicle, map<Vehicle*, Trip> & assignments, Network const & network, int time)
{
    // Prepare simulation.
    vehicle.just_boarded.clear();
    vehicle.just_alighted.clear();
    vehicle.pending_requests.clear();
    
    Trip t {};
    if (assignments.count(&vehicle))
        t = assignments[&vehicle];
    
    // Dispatch by job type.
    if (t.requests.size() || vehicle.passengers.size())
        move_vehicle(vehicle, t, network, time);
    else if (vehicle.offset)
        move_jobless_vehicle(vehicle, network, time);
    else
        vehicle.order_record.clear();
}


struct simulation_obj
{
    int time;
    map<Vehicle*, Trip>* assignments;
    Network const* network;
    vector<Vehicle>* vehicles;
};


void simulate_dispatch(void* simulation_data)
{
    // Extract data.
    struct thread_data* t = (struct thread_data*) simulation_data;
    int start = t->start;
    int end = t->end;
    
    struct simulation_obj* data = (struct simulation_obj*) t->data;
    auto time = data->time;
    auto assignments = data->assignments;
    auto network = data->network;
    auto vehicles = data->vehicles;
    
    for (auto i = start; i < end; i++)
        simulate_vehicle((*vehicles)[i], *assignments, *network, time);
}


void simulate_vehicles(vector<Vehicle> & vehicles, 
        map<Vehicle*, Trip> & assignments, 
        Network const & network, 
        int time,
        Threads & threads)
{
    if (SIMULATOR_VERBOSE)
    {
        ofstream joblogfile(RESULTS_DIRECTORY + "/joblog.log", std::ios_base::app);
        joblogfile << "TIME " << encode_time(time) << endl;
    }
    
    struct simulation_obj data {time, &assignments, &network, &vehicles};
    threads.auto_thread(vehicles.size(), simulate_dispatch, (void*) &data);
}

} /* namespace simulator */
