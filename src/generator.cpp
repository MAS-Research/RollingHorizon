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

#include "algorithms/ilp_common.hpp"
#include "algorithms/ilp_full.hpp"
#include "generator.hpp"
#include "settings.hpp"

#include <fstream>
#include <set>

using namespace std;
 
namespace generator
{

/* Different versions of trip assignment */

std::map<Vehicle*, Trip> trip_assignment(
        std::vector<Vehicle*> const & vehicles,
        std::vector<Request*> const & requests,
        int time,
        Network const & network,
        Threads & threads)
{
    return ilp_full::assignment(vehicles, requests, time, network, threads);
}

Trip previoustrip(Vehicle* v, Network const & n, int time)
{
    Trip previoustrip {};
    pair<int,vector<NodeStop>> previouspair = routeplanner::travel(*v,
            v->pending_requests, MEMORY, n, time);
    previoustrip.cost = previouspair.first;
    previoustrip.order_record = previouspair.second;
    previoustrip.requests = v->pending_requests;
    previoustrip.use_memory = true;
    return previoustrip;
}

}
