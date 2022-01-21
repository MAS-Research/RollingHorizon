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

#include "buffer.hpp"
#include "csvreader.hpp"
#include "formatting.hpp"
#include "generator.hpp"
#include "network.hpp"
#include "rebalance.hpp"
#include "request.hpp"
#include "routeplanner.hpp"
#include "settings.hpp"
#include "simulator.hpp"
#include "threads.hpp"
#include "trip.hpp"
#include "vehicle.hpp"

#include <chrono>
#include <fstream>
#include <iostream>
#include <map>
#include <set>
#include <stdexcept>
#include <vector>

using namespace std;
using namespace std::chrono;

int main(int argc, char *argv[])
{
    info("Starting Ridepool Simulator!!!", White);
    initialize(argc, argv);
    
    { // Head the output file with a description of the run.
        ofstream results(RESULTS_DIRECTORY + "/results.log", ios_base::app);
        results << "DATAROOT " << DATAROOT << endl;
        results << "RESULTS_DIRECTORY " << RESULTS_DIRECTORY << endl;
        results << "RH" << RH << endl;
        results << "TIMEFILE " << TIMEFILE << endl;
        results << "EDGECOST_FILE " << EDGECOST_FILE << endl;
        results << "VEHICLE_LIMIT " << VEHICLE_LIMIT << endl;
        results << "MAX_WAITING " << MAX_WAITING << endl;
        results << "MAX_DETOUR " << MAX_DETOUR << endl;
        results << "REQUEST_DATA_FILE " << REQUEST_DATA_FILE << endl;
        results << "VEHICLE_DATA_FILE " << VEHICLE_DATA_FILE << endl;
        results << "CARSIZE " << CARSIZE << endl;
        results << "INITIAL_TIME " << INITIAL_TIME << endl;
        results << "FINAL_TIME " << FINAL_TIME << endl;
        results << "ALGORITHM ";
        switch (ALGORITHM)
        {
            case ILP_FULL:
                results << "ILP_FULL" << endl;
                break;
            default:
                results << "UNLABELED" << endl;
        }
        results << "CTSP ";
        switch (CTSP)
        {
            case FULL:
                results << "FULL" << endl;
                break;
            case FIX_ONBOARD:
                results << "FIX_ONBOARD" << endl;
                break;
            case FIX_PREFIX:
                results << "FIX_PREFIX" << endl;
                break;
            default:
                results << "UNLABELED" << endl;
        }
        results << "CTSP_OBJECTIVE ";
        if (CTSP_OBJECTIVE == CTSP_VMT)
            results << "CTSP_VMT" << endl;
        else
            results << "NOT-VMT (other)" << endl;
        if (LAST_MINUTE_SERVICE)
            results << "LAST_MINUTE_SERVICE Active" << endl;
    }

    // Set up the thread pool for parallel work.
    info("Setting up Threadpool...", White);
    int num_threads = (argc > 1 ? atoi(argv[1]) : 1);
    if (argc == 1)
        info("Warning!  No thread count given, assuming 1.", Red);
    if (num_threads <= 0)
    {
        info("Warning!  Invalid thread count given as argument to program!", Red);
        throw runtime_error("Error!  Argument should be number of parallel threads to use.");
    }
    Threads threads(num_threads);
    info("Threadpool was set up!", Purple);

    // Set up routing matrix.
    info("Setting up network...", White);
    Network network;
    info("Network was loaded!", Purple);

    // Load all the vehicles and requests for the simulation.
    info("Loading vehicles and requests...", White);
    vector<Vehicle> vehicles = csvreader::load_vehicles();
    vector<Request> requests = csvreader::load_requests(network);
    vector<Request *> active_requests; // Holds requests across iterations.
    info("Vehicles and requests were loaded!", Purple);

    // Statistics and timing.
    info("Setting up other miscalaneous items...", White);
    int service_count = 0;
    int stats_dropoff_count = 0;
    int stats_entry_count = 0;
    int stats_total_waiting_time = 0;
    int stats_pickup_count = 0;
    int stats_total_in_vehicle_time = 0;
    int stats_total_delay = 0;
    int stats_shared_count = 0;
    int storage_service_count = 0;
    int storage_request_count = 0;
    double duration;
    double max_assignment_time = 0;
    chrono::time_point<chrono::high_resolution_clock>
        clock_start, clock_stop, clock_iteration_start, clock_iteration_stop;

    { // Add header to the top of the ilp log file.
        ofstream ilpfile(RESULTS_DIRECTORY + "/ilp.csv", std::ios_base::app);
        ilpfile << "Time\tObj\tSolverTime\tAbsGap\tRelGap\tNumAssigned\tStatus" << endl;
    }

    info("Done with all set up!", Purple);
    info("Starting iterations!", Cyan);
    int time = decode_time(INITIAL_TIME) - INTERVAL;
    while(time < decode_time(FINAL_TIME) - INTERVAL)  // Each loop is a simulation of a time step.
    {
        time += INTERVAL; // Increment simulation clock.
        info("Updated simulation clock to :" + to_string(encode_time(time)) +
                 "\tSystem time " + current_time(), Purple);

        clock_iteration_start = std::chrono::high_resolution_clock::now();
        clock_start = std::chrono::high_resolution_clock::now();

        // Get the set of active vehicles and new requests for this iteration.
        info("Running buffer update", Yellow);
        vector<Vehicle*> active_vehicles = buffer::get_active_vehicles(vehicles, time);
        
        vector <Request*> new_requests; //define new requests
        if (RH != 0)
        {
            if (time == 0)
            {
                new_requests = buffer::get_new_requests_0(requests, time, RH);
                vector<Request*> new_requests_real = buffer::get_new_requests(requests, time);
                stats_entry_count += new_requests_real.size();
            }
            else
            {
                new_requests = buffer::get_new_requests_offset(requests, time, RH);
                vector<Request*> new_requests_real = buffer::get_new_requests(requests, time);
                stats_entry_count += new_requests_real.size();
            }
        }
        else
        {
            new_requests = buffer::get_new_requests(requests, time);
            stats_entry_count += new_requests.size();
        }


        // Adding new requests.
        for (auto r : new_requests)
            active_requests.push_back(r);

        clock_stop = std::chrono::high_resolution_clock::now();
        duration = 0.000001 * duration_cast<microseconds>(clock_stop - clock_start).count();
        clock_start = clock_stop;
        info(to_string(duration) + " Buffer update completed", Green);

        // Select which trips to assign.
        info("Starting trip assignment problem", Yellow);
        map<Vehicle*, Trip> assigned_trips = generator::trip_assignment(active_vehicles,
                active_requests, time, network, threads);

        // Filter null trips so we don't confuse the rebalancing system.
        set<Vehicle*> blank_trips;
        for (auto & v : vehicles)
            if (assigned_trips.count(&v) && v.passengers.size() == 0 && assigned_trips[&v].requests.size() == 0)
                blank_trips.insert(&v);
        for (auto v : blank_trips)
            assigned_trips.erase(v);
        
        vector<Request *> assigned_requests;
        for (auto &t : assigned_trips)
            for (auto r : t.second.requests)
                assigned_requests.push_back(r);

        clock_stop = std::chrono::high_resolution_clock::now();
        double duration_ilp = 0.000001 * duration_cast<microseconds>(clock_stop - clock_start).count();
        clock_start = clock_stop;
        info(to_string(duration_ilp) + " Assignments have been made.", Green);
        cout << assigned_trips.size() << endl;

        // Rebalance unassigned vehicles.
        info("Computing vehicle rebalancing", Yellow);
        map<Vehicle*,Request> dummy_request_store;  // A problem with variable scoping.
        map<Vehicle*,Trip> rebalancing_trips = rebalance::make_rebalance(
                assigned_trips, active_vehicles, active_requests, dummy_request_store, network);
        assigned_trips.insert(rebalancing_trips.begin(), rebalancing_trips.end());
        {
            ofstream rb(RESULTS_DIRECTORY + "/rebalance.log", ios_base::app);
            rb << "TIME STAMP " << encode_time(time) << endl;
            for (auto & x : rebalancing_trips)
                rb << "{'v':" << x.first->id << ",'t':" << x.second.requests[0]->origin << "}" << endl;
        }

        clock_stop = std::chrono::high_resolution_clock::now();
        double duration_rebalancing = 0.000001 * duration_cast<microseconds>(clock_stop - clock_start).count();
        clock_start = clock_stop;
        info(to_string(duration_rebalancing) + "  Vehicle re-balancing completed", Green);

        // Record the time it took to process new requests and make routing assignments.
        clock_iteration_stop = std::chrono::high_resolution_clock::now();
        double duration_assignment_process = 0.000001 * duration_cast<microseconds>(
                clock_iteration_stop - clock_iteration_start).count();
        max_assignment_time = max(max_assignment_time, duration_assignment_process);

        // Perform the simulation.
        info("Vehicle simulation started", Yellow);
        simulator::simulate_vehicles(vehicles, assigned_trips, network, time, threads);

        clock_stop = std::chrono::high_resolution_clock::now();
        double duration_simulation = 0.000001 * duration_cast<microseconds>(clock_stop - clock_start).count();
        info(to_string(duration_simulation) + "  Vehicle simulation completed", Green);

        // Write intermediate results to file and update statistics.
        info("Recording results and updating statistics", Yellow);
        for (auto & vehicle : vehicles) // Collect statistics from vehicles.
        {
            for (auto r : vehicle.just_boarded)
            {
                stats_total_waiting_time += r->boarding_time - r->entry_time;
                stats_pickup_count++;
                service_count++;
            }
            for (auto r : vehicle.just_alighted)
            {   
                stats_dropoff_count++;
                stats_total_in_vehicle_time += r->alighting_time -  r->boarding_time;
                stats_total_delay += r->alighting_time - r->boarding_time - r->ideal_traveltime;
                stats_shared_count += (r->shared);
            }
        }
        {
            ofstream results_file(RESULTS_DIRECTORY + "/results.log", std::ios_base::app);
            results_file << "TIME STAMP:" << encode_time(time) << endl;
            results_file << "SYSTEM TIME: " << current_time() << endl;  // Automatically adds its own newline.
            results_file << "\tIteration Assignment Time\t" << duration_assignment_process << endl;
            results_file << "\tMaximum Assignment Time\t" << max_assignment_time << endl;
            results_file << "\tILP Assignment Time\t" << duration_ilp << endl;
            results_file << "\tRebalance Time\t" << duration_rebalancing << endl;
            results_file << "\tActive vehicles\t" << active_vehicles.size() << endl;
            results_file << "\tPending requests\t" << active_requests.size() << endl;
            results_file << "\tService Count\t" << service_count - storage_service_count << endl;
            storage_service_count = service_count;
            results_file << endl;

            // Update statistics.

            // Service Rate
            double service_rate = 100 * stats_pickup_count / double(stats_entry_count);
            results_file << "\tService Rate\t" << service_rate << "\t%" << endl;
            info("Service rate is " + to_string(service_rate) + ".", Red);

            // Average waiting time.
            double average_waiting_time = stats_total_waiting_time / double(stats_pickup_count);
            results_file << "\tAvg Waiting\t" << average_waiting_time << endl;

            // Average riding time.
            double average_riding_time = stats_total_in_vehicle_time / double(stats_dropoff_count);
            results_file << "\tAvg Riding\t" << average_riding_time << endl;

            // Average total delay.
            double average_total_delay = stats_total_delay / double(stats_dropoff_count);
            results_file << "\tAvg Delay\t" << average_total_delay << endl;

            // Mean passengers, absolute.
            double mean_passengers = (time != decode_time(INITIAL_TIME) ? 
                    stats_total_in_vehicle_time / double((time - decode_time(INITIAL_TIME)) *
                            active_vehicles.size())
                    : 0.0);
            results_file << "\tMean Passen\t" << mean_passengers << endl;

            // Shared rate
            double shared_rate = (stats_dropoff_count ? 
            100 * stats_shared_count / double(stats_dropoff_count)
                                                      : 0.0);
            results_file << "\tShared rate\t" << shared_rate << "\t%" << endl;

            // Total shared
            results_file << "\tTotal shared\t" << stats_shared_count << endl;

            // Average total distance.

            // Occupancy? (Passengers/Vehicle)
        }


        // Update active requests list to exclude requests that were not assigned.
        info("Updating the active requests list", Yellow);
        {   
            
            active_requests.clear();

            set<int> boarded_requests;
            for (auto &v : vehicles)
                for (Request* r : v.just_boarded)
                    boarded_requests.insert(r->id);

            for (Request* r : assigned_requests)
                if (!boarded_requests.count(r->id) && time < r->latest_boarding)
                    active_requests.push_back(r);
            
            set<Request *> final_assigned_requests; // Note: Captures rebalances as well.
            for (auto &t : assigned_trips)
                for (Request *r : t.second.requests)
                    final_assigned_requests.insert(r);

            for (Request *r : final_assigned_requests) // For statistics collection.
                r->assigned = true;
            
        }

        info("Current request buffer is updated", Green);

        info("Done with iteration", Green);
    } /* End of iteration loop. */

    // Final statistics for the final summary!
    {
        ofstream results_file(RESULTS_DIRECTORY + "/results.log", std::ios_base::app);
        results_file << "FINAL SUMMARY" << endl;

        int final_count = stats_pickup_count;
        int errors = 0;

        for (auto r : active_requests)
            if (r->assigned && r->boarding_time == 0)
                if (r->entry_time + MAX_WAITING < time)
                    errors++;
                else
                    final_count++;

        double service_rate = 100 * final_count / double(stats_entry_count);
        results_file << "\tService Rate\t" << service_rate << "\t%" << endl;
        results_file << "\tServed\t" << final_count << endl;
        results_file << "\tError Count\t" << errors << endl;

        int passenger_time = stats_total_in_vehicle_time;
        for (auto &v : vehicles)
            for(auto &r : v.passengers)
                if (r->alighting_time == 0)
                {
                    int duration = time - r->boarding_time;
                    passenger_time += duration;
                }

        double mean_passengers = passenger_time / double(
                (time - decode_time(INITIAL_TIME)) * vehicles.size()); // Buffer or not?
        results_file << "\tMean Passen\t" << mean_passengers << endl;

        long long total_idle = 0, total_enroute = 0, total_rebalancing = 0, total_inuse = 0;
        for (auto &v : vehicles)
        {
            total_idle += v.get_total_idle(time);
            total_enroute += v.get_total_enroute(time);
            total_rebalancing += v.get_total_rebalancing(time);
            total_inuse += v.get_total_inuse(time);
        }

        results_file << "\tTotal Idle\t" << total_idle << endl;
        results_file << "\tTotal En Route\t" << total_enroute << endl;
        results_file << "\tTotal Rebalancing\t" << total_rebalancing << endl;
        results_file << "\tTotal Inuse\t" << total_inuse << endl;
    }

}
