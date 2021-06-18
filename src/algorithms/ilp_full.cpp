/*
 * The MIT License
 *
 * Copyright 2020 Matthew Zalesak.
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

#include "algorithms/ilp_common.hpp"
#include "formatting.hpp"
#include "generator.hpp"
#include "routeplanner.hpp"
#include "settings.hpp"

#include <algorithm>
#include <cmath>
#include <mutex> // <-- Guilty party.  Secretly includes "chrono"
#include <fstream>
#include <set>
#include <sstream>
#include <stdexcept>

using namespace std;
 
namespace ilp_full
{

mutex mtx;

struct rtv_thread_data
{
    int time;
    map<Request*, set<Request*>>* rr_edges;
    map<Vehicle*, vector<Request*>>* vr_edges;
    map<Vehicle*, vector<Trip>>* trip_list;
    Network const* network;
    vector<Vehicle*> const* vehicles;
};

 
//Function to create RTV graph
void make_rtvgraph(void* rtv_data)
{
    bool rtvverbose = false;
    
    struct thread_data* t = (struct thread_data*) rtv_data;
    int start = t->start;
    int end = t->end;
    
    struct rtv_thread_data* data = (struct rtv_thread_data*) t->data;
    auto time = data->time;
    auto rr_edges = data->rr_edges;
    auto vr_edges = data->vr_edges;
    auto trip_list = data->trip_list;
    auto network = data->network;
    auto vehicles = data->vehicles;
    
    for (auto i = start; i < end; i ++)
    {
        // Boiler plate for timing.
        auto start_time = chrono::steady_clock::now();
        bool timeout = false; // Note:  Use RTV_TIMELIMIT in settings.hpp to control.
        
        // Select the vehicle, make our clique list by iteration k.
        Vehicle* v = (*vehicles)[i];
        vector<vector<Trip>> round;
        set<Request*> previous_assigned_passengers (v->pending_requests.begin(), v->pending_requests.end());
        
        // Generate initial trip with no assignment.
        {
            Trip baseline {};
            vector<Request*> rs;
            auto result = routeplanner::time_travel(*v, rs, STANDARD, *network, time, start_time);
            baseline.cost = result.first;
            baseline.order_record = result.second;
            round.push_back(vector<Trip>({baseline}));
            round[0][0].requests.clear();
        }
        
        mtx.lock();
        set<Request*> initial_pairing ((*vr_edges)[v].begin(), (*vr_edges)[v].end());
        mtx.unlock();
        
        round.push_back(vector<Trip>());
        initial_pairing.insert(v->pending_requests.begin(), v->pending_requests.end());
        mtx.lock();
        if (initial_pairing.size() > (*vr_edges)[v].size())
            cout << "Added " << initial_pairing.size() - (*vr_edges)[v].size() << " reqs" << endl;
        mtx.unlock();
        for (auto r : initial_pairing)
        {
            vector<Request*> requests {r};
            pair<int,vector<NodeStop>> path = routeplanner::time_travel(*v, requests, STANDARD, *network, time, start_time);
            if (path.first >= 0)
            {
                Trip trip {};
                trip.cost = path.first;
                trip.order_record = path.second;
                trip.requests = {r};
                round[1].push_back(trip);
            }
        }
        
        // In all subsequent rounds, take pairs from the previous round and build if they add one new element.
        int counter = 0;
        while (round[round.size() - 1].size() && !timeout)
        {
            int k = round.size();
            if (k > v->capacity)
                break;
            round.push_back(vector<Trip>());
            for (auto first = 0; first < round[k - 1].size() && !timeout; first++)
            {
                // Get new request set.
                set<Request*> left (round[k-1][first].requests.begin(), round[k-1][first].requests.end());
                
                for (auto second = first + 1; second < round[k - 1].size() && !timeout; second++)
                {
                    // Check the time.
                    auto end_time = chrono::steady_clock::now();
                    auto duration = chrono::duration_cast<chrono::milliseconds> (end_time - start_time).count();
                    if (RTV_TIMELIMIT && duration > RTV_TIMELIMIT)
                    {
                        timeout = true;
                        continue;
                    }

                    // Get new request set.
                    set<Request*> right (round[k-1][second].requests.begin(), round[k-1][second].requests.end());
                    set<Request*> requests = left;
                    requests.insert(right.begin(), right.end());
                    counter ++;
                    
                    // Reject if there are too many new requests.
                    int const MAX_NEW = 8;
                    {
                        int max_new = MAX_NEW;
                        for (auto r : requests)
                            if (!previous_assigned_passengers.count(r))
                                max_new -= 2;
                        if (max_new < 0)
                            continue;
                    }
                    
                    // Reject if this is not a simple +1.
                    if (requests.size() != k)
                        continue;
                    
                    // Reject if this is not a unique trip.
                    bool unique = true;
                    for (auto & t : round[k])
                        if (set<Request*>(t.requests.begin(), t.requests.end()) == requests)
                        {
                            unique = false;
                            break;
                        }
                    if (!unique)
                        continue;
                    
                    // Add a placeholder to show we've considered this option.
                    vector<Request*> request_vector (requests.begin(), requests.end());
                    round[k].push_back({-1, false, false, {}, request_vector});
                    
                    // Reject if the RR graph does not connect the requests.
                    bool rr_connected = true;
                    for (auto r : left)
                        if (!right.count(r))
                            for (auto rr : right)
                                if (!(*rr_edges)[r].count(rr) && !(*rr_edges)[rr].count(r))
                                {
                                    rr_connected = false;
                                    break;
                                }
                    if (!rr_connected)
                        continue;
                    for (auto r : right)
                        if (!left.count(r))
                            for (auto rr : left)
                                if (!(*rr_edges)[r].count(rr) && !(*rr_edges)[rr].count(r))
                                {
                                    rr_connected = false;
                                    break;
                                }
                    if (!rr_connected)
                        continue;
                    
                    // Reject if all subsets (-1) are not present.
                    bool subset_test = true;
                    for (auto r : requests)
                    {
                        set<Request*> subset = requests;
                        subset.erase(r);
                        bool matched = false;
                        for (auto & t : round[k - 1])
                            if (set<Request*>(t.requests.begin(), t.requests.end()) == subset)
                            {
                                matched = true;
                                break;
                            }
                        if (!matched)
                        {
                            subset_test = false;
                            break;
                        }
                    }
                    if (!subset_test)
                        continue;
                    
                    // Reject if there is no feasible routing for this request set.
                    bool preokay = true;
                    {
                        auto end_time = chrono::steady_clock::now();
                        auto duration = chrono::duration_cast<chrono::milliseconds>(end_time - start_time).count();
                        preokay = (duration <= RTV_TIMELIMIT);
                    }
                    pair<int,vector<NodeStop>> path = routeplanner::time_travel(
                            *v, request_vector, STANDARD, *network, time, start_time);
                    if (path.first < 0)
                        continue;
                    
                    // Accepted!  Save this new trip!
                    Trip trip {};
                    trip.cost = path.first;
                    trip.order_record = path.second;
                    trip.requests = vector<Request*>(requests.begin(), requests.end());
                    round[k][round[k].size() - 1] = trip;
                }
            }
            
            for (auto i = 0; i < round[k].size(); i++) // Filter failed placeholders.
                if (round[k][i].cost < 0)
                {
                    round[k][i] = round[k][round[k].size() - 1];
                    round[k].pop_back();
                    i--;
                }
        }
        
        // Convert into appropriate format.  This include adding in the pending requests.
        vector<Trip> potential_trip_list;
        for (auto & list : round)
            potential_trip_list.insert(potential_trip_list.end(), list.begin(), list.end());
        
        for (auto & t : potential_trip_list)
            if (t.cost == -1)
                throw runtime_error("Negative cost not cleaned up.");

        // Include possibility of previous assignment.
        if (v->order_record.size())
        {
            Trip previoustrip = generator::previoustrip(v, *network, time);
            potential_trip_list.push_back(previoustrip);

            int obj = previoustrip.cost;
            if (obj == -1)
            {
                cout << "The vehicle is " << v->id << endl;
                cout << "Requests are: ";
                for (auto r : v->pending_requests)
                    cout << r->id << " ";
                cout << endl;
                throw runtime_error("Previous assignment no longer feasible.");
            }
        }
        
        mtx.lock();
        (*trip_list)[v] = potential_trip_list;
        mtx.unlock();

    }
}


struct rv_thread_data
{
    int time;
    map<Request*, vector<Vehicle*>>* rv_edges;
    Network const* network;
    vector<Request*> const* requests;
    vector<Vehicle*> const* vehicles;
};


void make_rvgraph(void* rv_data)
{
    struct thread_data* t = (struct thread_data*) rv_data;
    int start = t->start;
    int end = t->end;
    
    struct rv_thread_data* data = (struct rv_thread_data*) t->data;
    auto time = data->time;
    auto rv_edges = data->rv_edges;
    auto network = data->network;
    auto requests = data->requests;
    auto vehicles = data->vehicles;
    
    for (int i = start; i < end; i++)
    {
        Request* r = (*requests)[i];
        vector<Request*> requests { r };
        int origin = r->origin;
        double buffer = 0;
        vector<Vehicle*> compatible_vehicles;

        multimap<int,Vehicle*> nearest_vs;
        for (Vehicle* v : *vehicles)
        {
            double min_wait = network->get_vehicle_time(*v, origin) - buffer;
            if (time + min_wait > r->latest_boarding) continue;
            nearest_vs.insert(make_pair(min_wait, v));
        }
        
        int count = 0;
        for (auto &x : nearest_vs)
        {
            Vehicle* v = x.second;
            pair<int,vector<NodeStop>> raw_path = routeplanner::travel(*v, requests, STANDARD, *network, time);
            if (raw_path.first >=0 )
            {
                compatible_vehicles.push_back(v);
                if (PRUNING_RV_K > 0 && ++count >= PRUNING_RV_K) break;
            }
        }
        
        mtx.lock();
        (*rv_edges)[r] = compatible_vehicles;
        mtx.unlock();
    }
}


double detour_factor(const Request* a, const Request* b, const Network* network)
{
    double best = INFINITY;
    int o1 = a->origin, o2 = b->origin;
    int d1 = a->destination, d2 = b->destination;
    auto onedist = network->get_time(o1, d1);
    if (onedist)
    {
        double ratio = network->get_time(o1, o2) + network->get_time(o2, d1);
        ratio /= onedist;
        best = min(best, ratio);
    }
    auto twodist = network->get_time(o2, d2);
    if (twodist)
    {
        double ratio = network->get_time(o2, o1) + network->get_time(o1, d2);
        ratio /= twodist;
        best = min(best, ratio);
    }
    if (!onedist && !twodist)
        best = 0;
    return best;
}


struct rr_thread_data
{
    int time;
    map<Request*, set<Request*>>* rr_edges;
    Network const* network;
    vector<Request*> const * requests;
};


// The graph specifically checks for r1 boarding first.
void make_rrgraph(void* rr_data)
{
    struct thread_data* t = (struct thread_data*) rr_data;
    int start = t->start;
    int end = t->end;
    
    struct rr_thread_data* data = (struct rr_thread_data*) t->data;
    auto time = data->time;
    auto rr_edges = data->rr_edges;
    auto network = data->network;
    auto requests = data->requests;
    
    for (int i = start; i < end; i++)
    {
        Request* r1 = (*requests)[i];
        int start_node = r1->origin;
        vector<Request*> compatible_requests;
        
        for (Request* r2 : *requests)
        {
            if (*r1 == *r2)  // Don't pair with itself!
                continue;
            vector<Request*> request_list { r1 , r2 };
            
            // Heuristic to prune the requests without calling the travel function.
            int r2_origin = r2->origin;
            double buffer = 0;
            double min_wait = network->get_time(start_node, r2_origin) - buffer;
            if (min_wait + max(time, r1->entry_time) > r2->latest_boarding)
                continue;
            
            Vehicle dummyVehicle(0, 0, 4, start_node);

            pair<int,vector<NodeStop>> raw_path = routeplanner::travel(dummyVehicle, request_list, STANDARD,
                    *network, time);
            if (raw_path.first >= 0) // I.e., valid trip.
                compatible_requests.push_back(r2);
        }
        
        auto sort_lambda = [network, r1](const Request* a, const Request* b) -> bool
        {
            double avalue = detour_factor(r1, a, network);
            double bvalue = detour_factor(r1, b, network);
            return avalue < bvalue;
        };
        sort(compatible_requests.begin(), compatible_requests.end(), sort_lambda);
        if (PRUNING_RR_K > 0 && compatible_requests.size() > PRUNING_RR_K) // Keep only the k best!
            compatible_requests.resize(PRUNING_RR_K);
        
        mtx.lock();
        (*rr_edges)[r1] = set<Request*>(compatible_requests.begin(), compatible_requests.end());
        mtx.unlock();
    }
}

// PNAS version : Performs sequence of steps, each in parallel, to produce RTV graph and then assignments. 
std::map<Vehicle*, Trip> assignment(
        std::vector<Vehicle*> const & vehicles,
        std::vector<Request*> const & requests,
        int time,
        Network const & network,
        Threads & threads)
{
    info("Building R-V edges of RV graph", Yellow);
    map<Vehicle*, vector<Request*>> vr_edges;  // RV edges indexed by vehicle id.
    {
        map<Request*, vector<Vehicle*>> rv_edges;
        struct rv_thread_data rv_data {time, &rv_edges, &network, &requests, &vehicles};
        threads.auto_thread(requests.size(), make_rvgraph, (void*) &rv_data);
        
        for (auto x : rv_edges) // Invert the graph.
        {
            Request* r = x.first;
            vector<Vehicle*> vs = x.second;
            for (auto v : vs)
                vr_edges[v].push_back(r);
        }
    }
    
    info("Buidling R-R edges of RV graph", Yellow);
    map<Request*, set<Request*>> rr_edges;  // RR edges indexed by request id.
    {
        struct rr_thread_data rr_data {time, &rr_edges, &network, &requests};
        threads.auto_thread(requests.size(), make_rrgraph, (void*) &rr_data);
    }
    
    info("Building RTV graph", Yellow);
    map<Vehicle*, vector<Trip>> trip_list;  // Store possible trips per vehicle
    {
        vector<Vehicle*> sorted_vs = vehicles;
        sort(sorted_vs.begin(), sorted_vs.end(),
                [vr_edges](Vehicle* & a, Vehicle* & b) -> bool {
                        if (vr_edges.count(a) && !vr_edges.count(b))
                            return true;
                        if (vr_edges.count(b) && !vr_edges.count(a))
                            return false;
                        if (vr_edges.count(a) && vr_edges.count(b))
                            if (vr_edges.at(a).size() > vr_edges.at(b).size())
                                return true;
                            else if (vr_edges.at(a).size() < vr_edges.at(b).size())
                                return false;
                        return a->id < b->id;
                });
        struct rtv_thread_data rtv_data {time, &rr_edges, &vr_edges, &trip_list, &network, &sorted_vs};
        threads.mega_thread(vehicles.size(), make_rtvgraph, (void*) &rtv_data);
    }
    
    int count = 0;
    for (auto & x : trip_list)
        count += x.second.size();
    info("Trip list is of size " + to_string(count), Red);
    
    { // Check to be sure no requests were downright rejected if they were previously assigned.
        set<Request*> rs;
        for (auto & x : trip_list)
            for (auto t : x.second)
                for (auto r : t.requests)
                    rs.insert(r);
        map<Request*,int> ps;
        for (auto v : vehicles)
            for (auto r : v->pending_requests)
                ps[r] = v->id;
        for (auto r : requests)
            if (r->assigned && !rs.count(r))
                if (!ps.count(r))
                    throw runtime_error("Help!  I was not included!");
                else
                {
                    cout << r << " with id " << r->id << " on " << ps[r] << endl;
                    throw runtime_error("Help!  I was a pending request!");
                }
    }
    { // Check to be sure that all previous trips are included in the future possibilities!
        for (auto v : vehicles)
        {
            set<Request*> prev (v->pending_requests.begin(), v->pending_requests.end());
            bool found = false;
            for (auto & t : trip_list[v])
            {
                set<Request*> rs (t.requests.begin(), t.requests.end());
                if (prev == rs)
                {
                    found = true;
                    break;
                }
            }
            if (!found)
            {
                // I have verified that the trip is feasible in rr and vr graph.  travel feasible too.
                cout << "Vid " << v->id << endl;
                throw runtime_error("Did not replicate the trip!");
            }
        }
    }

    // Output trace of generated trip_list.
    stringstream rtv; //ofstream rtv(RESULTS_DIRECTORY + "/rtv.log", ios_base::app); // Stringstream disables
    rtv << "TIME STAMP " << encode_time(time) << endl;
    for (auto & x : trip_list)
    {
        int vid = x.first->id;
        for (auto & t : x.second)
        {
            rtv << "{'v':" << vid << ",'rs':[";
            for (auto r : t.requests)
                rtv << r->id << ",";
            rtv << "],'c':" << t.cost << "}" << endl;
        }
    }

    return ilp_common::ilp_assignment(trip_list, requests, time);
}

}

