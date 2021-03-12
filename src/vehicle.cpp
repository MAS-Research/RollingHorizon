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

#include "vehicle.hpp"
#include "settings.hpp"

#include <iostream>

Vehicle::Vehicle(int id, int start_time, int capacity, int node) :
        id {id},
        start_time {start_time},
        capacity {capacity},
        is_rebalancing {false},
        rebalance_target {-1},
        prev_node {node},
        node {node},
        offset {0},
        total_rebalance_distance {0},
        total_distance_traveled {0},
        state {Idle},
        total_idle {0},
        total_rebalancing {0},
        total_enroute {0},
        total_inuse {0},
        time_stamp {0}
{}

void Vehicle::add_distance(double distance)
{
    this->total_distance_traveled += distance;
    if (is_rebalancing)
        this->total_rebalance_distance += distance;
}

double Vehicle::get_distance_traveled() const
{
    return this->total_distance_traveled;
}

double Vehicle::get_rebalance_distance() const
{
    return this->total_rebalance_distance;
}

void Vehicle::set_state(enum States state, int time)
{
    if (this->time_stamp == -1)
        this->time_stamp = time;
    
    if (state != this->state)
    {
        int duration = time - this->time_stamp;
        switch (this->state)
        {
            case Idle:
            {
                this->total_idle += duration;
                break;
            }
            case EnRoute:
            {
                this->total_enroute += duration;
                break;
            }
            case InUse:
            {
                this->total_inuse += duration;
                break;
            }
            default:
            {
                this->total_rebalancing += duration;
                break;
            }
        }
        this->state = state;
        this->time_stamp = time;
    }
}
    
/* When queried, extra time should be added on to the current (and final) state. */
int Vehicle::get_total_idle(int time) const
{
    if (this->state == Idle)
        return this->total_idle + (time - this->time_stamp);
    else
        return this->total_idle;
}

int Vehicle::get_total_rebalancing(int time) const
{
    if (this->state == Rebalancing)
        return this->total_rebalancing + (time - this->time_stamp);
    else
        return this->total_rebalancing;
}

int Vehicle::get_total_enroute(int time) const
{
    if (this->state == EnRoute)
        return this->total_enroute + (time - this->time_stamp);
    else
        return this->total_enroute;
}

int Vehicle::get_total_inuse(int time) const
{
    if (this->state == InUse)
        return this->total_inuse + (time - this->time_stamp);
    else
        return this->total_inuse;
}

int Vehicle::get_state() const
{
    if (this->state == Idle)
        return 0;
    else if (this->state == EnRoute)
        return 0;
    else if (this->state == Rebalancing)
        return 2;
    else
        return 3;
}
