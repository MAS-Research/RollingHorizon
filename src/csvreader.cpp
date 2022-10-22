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

#include "csvreader.hpp"
#include "formatting.hpp"
#include "settings.hpp"

#include <fstream>
#include <iostream>
#include <stdexcept>

using namespace std;

vector<Vehicle> csvreader::load_vehicles()
{
    vector<Vehicle> vehicles;
    ifstream vfile(DATAROOT + "/vehicles/" + VEHICLE_DATA_FILE);
    if (!vfile.is_open())
    {
        cout << "ERROR: Unable to open vehicles file." << endl;
        cout << "\tSearching for vehicles file at:" << endl;
        cout << "\t\t" << DATAROOT + "/vehicles/" + VEHICLE_DATA_FILE << endl;
        throw runtime_error("Vehicles file not found!");
    }
    
    string driver_id;
    string longitude;
    string latitude;
    string speed = "0.0";
    string time_string;
    string capacity;
    string starting_node;
  
    auto count = 0;

    while (!vfile.eof())
    {
        getline(vfile, driver_id, ',');
        getline(vfile, starting_node, ',');
        getline(vfile, latitude, ',');
        getline(vfile, longitude, ',');
        getline(vfile, time_string, ',');
        getline(vfile, capacity, '\n');
        
        if (!driver_id.size()) // Detect end of file.
            break;
        
        int vehicle_capacity = CARSIZE;
        if (vehicle_capacity < 0)
            vehicle_capacity = stoi(capacity);
    
        Vehicle vehicle(
                stoi(driver_id),
                0,
                vehicle_capacity, 
                stoi(starting_node) - 1);

        vehicles.push_back(vehicle);
        if (VEHICLE_LIMIT > 0 && ++count >= VEHICLE_LIMIT)
            break;
    }
    
    return vehicles;
}

vector<Request> csvreader::load_requests(Network const & network)
{
    vector<Request> requests;
    ifstream rfile(DATAROOT + "/requests/" + REQUEST_DATA_FILE);
    if (!rfile.is_open())
    {
        cout << "ERROR: Unable to open requests file." << endl;
        cout << "\tSearching for requests file at:" << endl;
        cout << "\t\t" << DATAROOT + "/requests/" + REQUEST_DATA_FILE << endl;
        throw runtime_error("Requests file not found!");
    }

    string request_id;
    string origin_longitude;
    string origin_latitude;
    string destination_longitude;
    string destination_latitude;
    string requested_time_string;
    string origin_node;
    string destination_node;
    int i = 0;
    while (!rfile.eof())
    {
        getline(rfile, request_id, ',');
        getline(rfile, origin_node, ',');
        getline(rfile, origin_longitude, ',');
        getline(rfile, origin_latitude, ',');
        getline(rfile, destination_node, ',');
        getline(rfile, destination_longitude, ',');
        getline(rfile, destination_latitude, ',');
        getline(rfile, requested_time_string, '\n');
        
        if (!request_id.size())  // Detect end of file.
            continue;
        
        Request r {};
        r.origin_longitude = stod(origin_longitude);
        r.origin_latitude = stod(origin_latitude);
        r.destination_longitude = stod(destination_longitude);
        r.destination_latitude = stod(destination_latitude);
        r.origin = stoi(origin_node) - 1;
        r.destination = stoi(destination_node) - 1;
        
        r.id = stoi(request_id);
        r.entry_time = read_time(requested_time_string);
        r.latest_boarding = r.entry_time + MAX_WAITING;
        network.get_time(r.origin, r.destination);
        r.latest_alighting = r.entry_time + MAX_DETOUR + network.get_time(r.origin, r.destination);
        r.ideal_traveltime = network.get_time(r.origin, r.destination);
        
        requests.push_back(r);

    }

    return requests;
}

