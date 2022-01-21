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
#include "formatting.hpp"
#include "settings.hpp"

#include <set>

#define MAX_STAY_TIME 24 //24 hours
using namespace std;


vector<Vehicle*> buffer::get_active_vehicles(vector<Vehicle> & vehicles, int time)
{
    vector<Vehicle*> buffer;
    for (auto & v : vehicles)
        buffer.push_back(&v);
    
    return buffer;
}

vector<Request*> buffer::get_new_requests(vector<Request> & requests, int time)
{
    vector<Request*> buffer;

    for (auto & r : requests)
        if (r.entry_time <= time  && time < r.entry_time + INTERVAL) // If already entered, but not too long ago.
            buffer.push_back(&r);

    return buffer;
}

vector<Request*> buffer::get_new_requests_0(vector<Request> & requests, int time, int rh)
{
    vector<Request*> buffer;
    for (auto & r : requests)
	if (r.entry_time - (rh*INTERVAL) <= time) 
	    buffer.push_back(&r);
    return buffer;
}

vector<Request*> buffer::get_new_requests_offset(vector<Request> & requests, int time, int rh)
{
    vector<Request*> buffer;
    for (auto & r : requests)
	if (r.entry_time - (rh*INTERVAL) <= time && time < r.entry_time + INTERVAL - (rh*INTERVAL))
	    buffer.push_back(&r);
    return buffer;
}
