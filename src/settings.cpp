/*
 * The MIT License
 *
 * Copyright 2018 Matthew Zalesak.
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

#include "formatting.hpp" 
#include "settings.hpp"

#include <boost/algorithm/string.hpp>
#include <fstream>
#include <iostream>
#include <map>
#include <stdexcept>

using namespace std;

// These are the default values if not set otherwise.
Algorithm ALGORITHM = ILP_FULL;
double alpha = 0.5;
AssignmentObjective ASSIGNMENT_OBJECTIVE = AO_SERVICERATE;
int CARSIZE = 4;
Ctsp CTSP = FIX_PREFIX;
CtspObjective CTSP_OBJECTIVE = CTSP_VMT;
string DATAROOT = "data";
string EDGECOST_FILE = "edges.csv";
int FINAL_TIME = 240000;
int INITIAL_TIME = 0;       // Time in HHMMSS
int INTERVAL = 60;
bool LAST_MINUTE_SERVICE;
int MAX_DETOUR = 600;
int MAX_WAITING = 300;
string REQUEST_DATA_FILE = "requests.csv";
string RESULTS_DIRECTORY = "results";
int RTV_TIMELIMIT = 0;
string TIMEFILE = "times.csv";
string VEHICLE_DATA_FILE = "vehicles.csv";
int VEHICLE_LIMIT = 1000; // 0;

map<string,Algorithm> algorithm_index {
    {"ILP_FULL", ILP_FULL}};
map<string,Ctsp> ctsp_index {
    {"FULL", FULL},
    {"FIX_ONBOARD", FIX_ONBOARD},
    {"FIX_PREFIX", FIX_PREFIX},
    {"MEGA_TSP", MEGA_TSP}};
map<string,CtspObjective> ctspobjective_index {
    {"CTSP_VMT", CTSP_VMT},
    {"CTSP_TOTALDROPOFFTIME", CTSP_TOTALDROPOFFTIME},
    {"CTSP_TOTALWAITING", CTSP_TOTALWAITING}};
map<string,AssignmentObjective> assignmentobjective_index {
    {"AO_SERVICERATE", AO_SERVICERATE},
    {"AO_RMT", AO_RMT}};


string process_string(string & s)
{
    if (s.size() && s[s.size() - 1] == '/')
        s.pop_back();
    return s;
}

void initialize(int argc, char** argv)
{
    for (auto i = 2; i + 1 < argc; i += 2)  // Skip first two arguments, program name and num_threads.
    {
        string key(argv[i]);
        string value(argv[i + 1]);
    
        if (key == "DATAROOT")
            DATAROOT = process_string(value);
        else if (key == "RESULTS_DIRECTORY")
            RESULTS_DIRECTORY = process_string(value);
        else if (key == "TIMEFILE")
            TIMEFILE = process_string(value);
        else if (key == "EDGECOST_FILE")
            EDGECOST_FILE = process_string(value);
        else if (key == "VEHICLE_LIMIT")
            VEHICLE_LIMIT = stoi(value);
        else if (key == "MAX_WAITING")
            MAX_WAITING = stoi(value);
        else if (key == "MAX_DETOUR")
            MAX_DETOUR = stoi(value);
        else if (key == "REQUEST_DATA_FILE")
            REQUEST_DATA_FILE = process_string(value);
        else if (key == "VEHICLE_DATA_FILE")
            VEHICLE_DATA_FILE = process_string(value);
        else if (key == "CARSIZE")
            CARSIZE = stoi(value);
        else if (key == "INITIAL_TIME")
            INITIAL_TIME = stoi(value);
        else if (key == "FINAL_TIME")
            FINAL_TIME = stoi(value);
        else if (key == "ALGORITHM")
            if (algorithm_index.count(value))
                ALGORITHM = algorithm_index[value];
            else
                throw runtime_error("Could not find algorithm index in settings.cpp: " + value);
        else if (key == "CTSP")
            if (ctsp_index.count(value))
                CTSP = ctsp_index[value];
            else
                throw runtime_error("Could not find CTSP index in settings.cpp: " + value);
        else if (key == "CTSP_OBJECTIVE")
            if (ctspobjective_index.count(value))
                CTSP_OBJECTIVE = ctspobjective_index[value];
            else
                throw runtime_error("Could not find CTSP OBJECTIVE index in settings.cpp: " + value);
        else if (key == "ALPHA")
           alpha = stod(value);
        else if (key == "ASSIGNMENT_OBJECTIVE")
            if (assignmentobjective_index.count(value))
                ASSIGNMENT_OBJECTIVE = assignmentobjective_index[value];
            else
                throw runtime_error("Could not find Assignment Objective in index in settings.cpp: " + value);
        else if (key == "LAST_MINUTE_SERVICE")
        {
            string s = boost::algorithm::to_lower_copy(value);
            if (s == "true")
                LAST_MINUTE_SERVICE = true;
            else if (s == "false")
                LAST_MINUTE_SERVICE = false;
            else
            {
                cout << "For " << key << " trying to interpret \"" << value << "\"." << endl;
                throw runtime_error("Argument could not be converted into a boolean.");
            }
        }
        else if (key == "INTERVAL")
            INTERVAL = stoi(value);
        else if (key == "RTV_TIMELIMIT")
            RTV_TIMELIMIT = stoi(value);
        else
            throw runtime_error("Argument not recognized: " + key);
    }
}
