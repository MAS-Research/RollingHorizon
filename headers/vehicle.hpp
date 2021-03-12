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

#ifndef VEHICLE_HPP
#define VEHICLE_HPP

#include "request.hpp"
#include "trip.hpp"

#include <vector>

enum States {Idle, Rebalancing, EnRoute, InUse, Boarding};

class Vehicle
{
public:
    Vehicle(int id, int start_time, int capacity, int node);
    int const id;
    int const start_time;
    int const capacity;
    
    bool is_rebalancing;
    int rebalance_target;
    int get_capacity() const;
    int get_id() const;
    int get_start_time() const;
    std::vector<Request*> passengers;
    std::vector<Request*> just_boarded;
    std::vector<Request*> just_alighted;
    std::vector<Request*> pending_requests;  // Assigned passengers not yet picked up.
    std::vector<NodeStop> order_record;  // Order of events.
    
    void add_distance(double distance);
    double get_distance_traveled() const;
    double get_rebalance_distance() const;
    int prev_node;
    int node;
    int offset;  // How far a vehicle has left to get to the node.
    
    /* Sets the state for collecting statistics.  No update occurs if the state is the same as current state. 
       When getting totals, extra time is added on to the current (and final) state. */
    void set_state(enum States state, int time);
    int get_state() const;
    int get_total_idle(int time) const;
    int get_total_rebalancing(int time) const;
    int get_total_enroute(int time) const;
    int get_total_inuse(int time) const;

private:
    double total_rebalance_distance;
    double total_distance_traveled;
    
    enum States state;
    int total_idle, total_rebalancing, total_enroute, total_inuse;
    int time_stamp;
};

#endif /* VEHICLE_HPP */
