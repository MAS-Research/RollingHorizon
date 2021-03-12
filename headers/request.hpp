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

#ifndef PASSENGER_HPP
#define PASSENGER_HPP

#include <iostream>
#include <vector>

struct Request
{
    friend bool operator<(Request a, Request b)
    {
        return a.id < b.id;
    }
    friend bool operator==(Request a, Request b)
    {
        return a.id == b.id;
    }
    
    int id;
    int origin;
    int destination;
    int ideal_traveltime = 0;
    
    int entry_time;
    int boarding_time;
    int alighting_time;
    int latest_boarding;
    int latest_alighting;
    bool shared;
    bool assigned;
    
    double origin_longitude;
    double origin_latitude;
    double destination_longitude;
    double destination_latitude;
};

struct NodeStop
{
    Request* r;
    bool is_pickup;
    int node;
    
    friend bool operator<(NodeStop const & a, NodeStop const & b)
    {
        if (a.r < b.r)
            return true;
        else if (a.r == b.r && a.is_pickup < b.is_pickup)
            return true;
        else
            return false;
    }
};

#endif /* PASSENGER_HPP */
