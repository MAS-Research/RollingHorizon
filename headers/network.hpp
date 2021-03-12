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

#ifndef NETWORK_HPP
#define NETWORK_HPP

#include "vehicle.hpp"

#include <vector>

typedef int vertex_t;
typedef double weight_t;


struct neighbor
{
    vertex_t target;
    weight_t weight;
    neighbor(vertex_t arc_target, weight_t arc_weight)
        : target(arc_target), weight(arc_weight) {}
};
 
class Network
{
 public:
    Network();
    int get_time(int node_one, int node_two) const;
    std::vector<int> dijkstra(int source, int destination) const;
    int get_distance(int node_one, int node_two) const;
    int get_vehicle_time(Vehicle const & v, int node) const;
    int get_vehicle_distance(Vehicle const & v, int node) const;
    int get_vehicle_offset(Vehicle const & v) const;
private:
    std::vector<std::vector<int>> time_matrix;
    std::vector<std::vector<int>> distance_matrix;
    std::vector<std::vector<neighbor>> adjacency_list;
};
 
#endif /* NETWORK_HPP */
