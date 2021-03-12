/*
 * The MIT License
 *
 * Copyright 2020 Matthew Zalesak.
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
#include "formatting.hpp"
#include "settings.hpp"

#include <fstream>
#include "fusion.h"             // For MOSEK functions.
#include <set>

using namespace mosek::fusion;  // For MOSEK functions.
using namespace monty;          // For MOSEK functions.
using namespace std;

namespace ilp_common
{

/* Function to solve main assignment problem. */
map<Vehicle*,Trip> ilp_assignment(
        map<Vehicle*, vector<Trip>> const & trip_list, vector<Request*> const & requests, int time)
{
    // Simultaneously count variable, get cost vector, and build set for constraint 2.
    int K = requests.size();
    int index = 0;
    vector<double> costs;
    map<int, set<int>> rids_to_trips;  // IRK + ITI
    
    for (auto &id_trip_pair : trip_list)
    {
        vector<Trip> trips = id_trip_pair.second;
        
        for (auto &trip : trips)
        {
            costs.push_back(trip.cost);
            vector<Request*> requests = trip.requests;
            for (Request* request : requests)
            {
                int id = request->id;
                rids_to_trips[id].insert(index);
            }
            index ++;
        }
    }
    
    if (index == 0)
        return {};
    
    // This is how Mosek says to create a model.
    Model::t M = new Model("Assignment"); auto _M = finally([&]() { M->dispose(); });
    Variable::t e = M->variable("e", new_array_ptr<int, 1>({index}), Domain::binary());
    Variable::t x = M->variable("x", new_array_ptr<int, 1>({K}), Domain::binary());
    
    // Objective function.
    if (ASSIGNMENT_OBJECTIVE == AO_SERVICERATE)
    {
        auto c = make_shared<ndarray<double, 1>>(shape(index), costs.begin(), costs.end());
        auto objective = Expr::add(Expr::dot(c, e), Expr::mul(MISS_COST, Expr::sum(x)));
        M->objective("obj", ObjectiveSense::Minimize, objective);
    }
    else if (ASSIGNMENT_OBJECTIVE == AO_RMT)
    {
        auto c = make_shared<ndarray<double, 1>>(shape(index), costs.begin(), costs.end());
        vector<double> travel_times;
        for (auto k = 0; k < K; k++)
            travel_times.push_back(requests[k]->ideal_traveltime);
        auto r = make_shared<ndarray<double, 1>>(shape(K), travel_times.begin(), travel_times.end());
        auto objective = Expr::add(Expr::dot(c, e), Expr::mul(RMT_REWARD, Expr::dot(r, x)));
        M->objective("obj", ObjectiveSense::Minimize, objective);
    }
    
    // Constraint One.
    int count = 0;
    for (auto &id_trip_pair : trip_list)
    {
        int vid = id_trip_pair.first->id;
        vector<Trip> trips = id_trip_pair.second;
        string name = "c1-" + to_string(vid);
        
        auto E = e->slice(count, count + trips.size());
        if (ALGORITHM != ILP_FULL)
            M->constraint(name, Expr::sum(E), Domain::lessThan(1.0));
        else
            M->constraint(name, Expr::sum(E), Domain::equalsTo(1.0)); // New test constraint.
        
        count += trips.size();
    }
    
    // Constraint Two.
    for (auto k = 0; k < K; k ++)
    {
        int id = requests[k]->id;
        set<int> indices = rids_to_trips[id];
        vector<int> index_v (indices.begin(), indices.end());
        string name = "c2-" + to_string(id);
        
        auto indices_vector = make_shared<ndarray<int, 1>>(shape(index_v.size()),
                index_v.begin(), index_v.end());
        auto E = e->pick(indices_vector);
        if (requests[k]->assigned)
            M->constraint(name, Expr::sum(E), Domain::equalsTo(1.0));
        else
            M->constraint(name, Expr::add(Expr::sum(E), x->index(k)), Domain::equalsTo(1.0));
    }

    {
        int i = 0;
        for (auto r : requests)
            if (r->assigned)
                i++;
        cout << "Number of assigned requests: " << i << "/" << requests.size() << endl;
    }
    
    // Set maximum solution time, relative gap, and absolute gap paramters.
    bool quick = true; // false;
    if (!quick)
    {
        M->setSolverParam("mioDisableTermTime", 60);
        M->setSolverParam("mioTolRelGap", 1e-8); // -1);  // 1e-4 is default value.
        M->setSolverParam("mioTolAbsGap", 0.0);   // 0.0  is default value.
        M->setSolverParam("mioNearTolRelGap", 1e-8); // 50);
    }
    else
    {
        M->setSolverParam("mioDisableTermTime", 60); //15);
        M->setSolverParam("mioTolRelGap", 1e-8);
        M->setSolverParam("mioTolAbsGap", 0.0);
        M->setSolverParam("mioNearTolRelGap", 5);
    }
    if (OPTIMIZER_VERBOSE)
        M->setLogHandler([=](const string &msg) {cout << msg << flush;});
    M->acceptedSolutionStatus(AccSolutionStatus::Feasible);
    
    // Solve.
    M->solve();
    
    vector<int> assignments;
    int icount = 0;
    
    auto E = (*e->level());
    for (auto i = 0; i < index; i++)
    {
        double d = E[i];
        assignments.push_back(d > 0.5);
        icount += (d > 0.5);
    }
    cout << "Made " << icount << " assignments." << endl;
    // Write statistics.
    {
        ofstream ilpfile(RESULTS_DIRECTORY + "/ilp.csv", std::ios_base::app);
        
        ilpfile << encode_time(time) << "\t";
        ilpfile << M->getSolverDoubleInfo("mioObjInt") << "\t";
        ilpfile << M->getSolverDoubleInfo("optimizerTime") << "\t";
        ilpfile << M->getSolverDoubleInfo("mioObjAbsGap") << "\t";
        ilpfile << M->getSolverDoubleInfo("mioObjRelGap") << "\t";
        ilpfile << icount << "\t";
        bool is_optimal = (M->getPrimalSolutionStatus() == SolutionStatus::NearOptimal ||
                M->getPrimalSolutionStatus() == SolutionStatus::Optimal);
        ilpfile << (is_optimal ? "Optimal" : "Suboptimal") << endl;
    }
    
    map<Vehicle*, Trip> assigned_trips;
    count = 0;
    for (auto & x : trip_list)
    {
        Vehicle* v = x.first;
        vector<Trip> const* trips = &x.second;
        for (auto r = 0; r < trips->size(); r++)
            if (assignments[r + count] > 0.5)
            {
                assigned_trips[v] = (*trips)[r];
                break;
            }
        
        count += trips->size();
    }
    
    return assigned_trips;
}

}
