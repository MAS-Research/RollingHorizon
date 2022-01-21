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
 
#include "network.hpp"
#include "settings.hpp"

#include <iostream>
#include <fstream>
#include <map>
#include <math.h>
#include <queue>
#include <sstream>
#include <stdexcept>
#include <string>
#include <boost/algorithm/string.hpp>
 
using namespace std;
using namespace boost;
 
Network::Network()
{  
    string line;
    ifstream timefile(DATAROOT + "/map/" + TIMEFILE);
    if (!timefile.is_open())
        throw runtime_error("Unable to open matrix file");
    
    int rowsize = 0;
    while (getline(timefile, line))
    {
        vector<int> row;
        row.reserve(rowsize);
        
        char * cstr = new char [line.length() + 1];
        std::strcpy(cstr, line.c_str());
        
        char* token;
        token = strtok(cstr, ",");
        while (token != NULL)
        {
            int entry_value = stoi(token);
            row.push_back(entry_value);
            token = strtok(NULL, ",");
        }
        delete[] cstr;
        time_matrix.push_back(row);
    }
    
    // WE ARE NOT USING DISTANCE HERE YET, SO JUST DUPLICATING TIME.
    //ifstream distancefile(DATAROOT + "/map/times.csv"); // full_distances.csv");
    ifstream distancefile(DATAROOT + "/map/" + TIMEFILE);
    if (!distancefile.is_open())
        throw runtime_error("Unable to open distances matrix file.");
    
    while (getline(distancefile, line))
    {
        vector<int> row;
        row.reserve(rowsize);
        
        char * cstr = new char [line.length() + 1];
        std::strcpy(cstr, line.c_str());
        
        char* token;
        token = strtok(cstr, ",");
        while (token != NULL)
        {
            int entry_value = stoi(token);
            row.push_back(entry_value);
            token = strtok(NULL, ",");
        }
        delete[] cstr;
        distance_matrix.push_back(row);
    }
    
    ifstream edgefile(DATAROOT + "/map/" + EDGECOST_FILE);
    if (edgefile.is_open())
    {
        while (getline(edgefile, line))
        {
            vector<string> fields;
            split(fields, line, is_any_of(","));
            int origin = stoi(fields[0]) - 1;
            int dest = stoi(fields[1]) - 1;
            int length = stoi(fields[2]);
            
            if (adjacency_list.size() < origin + 1)
                adjacency_list.resize(origin + 1);
            
            adjacency_list[origin].push_back(neighbor(dest, length));
        }
    }
    else
        throw runtime_error("Unable to open file for dijkstra shortest path calculation.");
}

/* Travel time between nodes. */
int Network::get_time(int node_one, int node_two) const
{
    if (node_one == -10)
        return DWELL_PICKUP;
    else if (node_one == -20)
        return DWELL_ALIGHT;
    else if (node_one == -30)
        return 0;
    else if (node_one < 0 || node_two < 0)
    {
        cout << "Network Error: Line " << __LINE__ << endl;
        getchar();
    }
    return time_matrix[node_one][node_two];
}

int Network::get_distance(int node_one, int node_two) const
{
    return get_time(node_one, node_two);
    if (node_one == -10 || node_one == -20 || node_one == -30)
        return 0;
    if (node_one < 0 || node_two < 0)
    {
        cout << "Negative index given to get_distance!  Network Line " << __LINE__ << endl;
        getchar();
    }
    return distance_matrix[node_one][node_two];
}

/* Specifically, this gets the distance offset. */
int Network::get_vehicle_offset(Vehicle const & v) const
{
    int origin = v.prev_node, destination = v.node;
    if (origin < 0 || destination < 0) // TODO Just error catcher, should remove.
        return 0;
    
    int time = get_time(origin, destination);
    int distance = get_distance(origin, destination);
    int time_offset = time - v.offset;
    
    if (time == 0)
        return 0;
    else
    {
        double percent = time_offset / time;
        double distance_offset_true = distance * percent;
        int distance_offset = int(floor(distance_offset_true));
        return distance_offset;
    }
}

int Network::get_vehicle_distance(Vehicle const & v, int node) const
{
    int origin = v.prev_node, destination = v.node;
    int current_leg = get_distance(origin, destination) - get_vehicle_offset(v);
    int final_leg = get_distance(destination, node);
    return current_leg + final_leg;
}

int Network::get_vehicle_time(Vehicle const & v, int node) const
{
    return v.offset + get_time(v.node, node);
}

vector<int> Network::dijkstra(int origin, int destination) const
{
    vector<int> path {origin};
    int here = origin;
    int count = 0;
    while (here != destination && here != -1 && count < 200)  // For each step of the way...
    {
        int best = get_time(here, destination) + 1;
        int node = -1;
        for (auto &n : adjacency_list[here])    // Try to take a strict step.
        {
            int n_node = n.target;
            int time = n.weight;
            
            if (n_node == destination)
            {
                node = n_node;
                break;
            }
            
            int follow_up = get_time(n_node, destination);
            if (time > 0 && time + follow_up < best)
            {
                best = time + follow_up;
                node = n_node;
            }
        }
        
        if (node == -1)                         // Logic if only good choices are zeros.
        {
            queue<neighbor> zeros;
            map<int,vector<int>> heritage;
            int comparison = get_time(here, destination);
            for (auto &n : adjacency_list[here])
                if (n.weight + get_time(n.target, destination) <= comparison)
                {
                    zeros.push(n);
                    heritage.emplace(n.target, initializer_list<int>{n.target});
                }
            
            while (zeros.size() && node == -1)
            {
                neighbor n = zeros.front();
                zeros.pop();
                
                for (auto &child : adjacency_list[n.target])
                {
                    if (child.weight + get_time(child.target, destination) <= comparison)
                    {
                        if (child.weight > 0 || child.target == destination)  // Golden case!  We found it!
                        {
                            path.insert(path.end(), heritage[n.target].begin(), heritage[n.target].end());
                            node = child.target;
                            break;
                        }
                        if (!heritage.count(child.target))  // Another zero we can consider!
                        {
                            zeros.push(child);
                            vector<int> new_heritage = heritage[n.target];
                            new_heritage.push_back(child.target);
                            heritage[child.target] = new_heritage;
                        }
                    }
                }
            }
        }
        
        path.push_back(node);
        here = node;
        count ++;
    }
    
    if (count > 200 || here == -1)
    {
        cout << "Oops!  Network dijkstra messed up somehow!" << endl;
        cout << "Query from " << origin << " to " << destination << endl;
        for (auto i : path)
            cout << i << "\t" << get_time(i, destination) << endl;
        getchar();
    }
    
    return path;
}

