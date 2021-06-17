// settings.hpp

#ifndef SETTINGS_HPP
#define SETTINGS_HPP

#define MISS_COST  10000000.0 // 100000
#define RMT_REWARD 100.0

#define OPTIMIZER_VERBOSE true
#define SIMULATOR_VERBOSE false
#define PRUNING_RV_K 0 // 30 // 0 // 30        // Heuristic that only connects requests with nearest k vehicles.
#define PRUNING_RR_K 0 //10    // Heuristic that only connects requests with nearest k requests.

enum Algorithm {ILP_FULL};
enum Ctsp {FULL, FIX_ONBOARD, FIX_PREFIX, MEGA_TSP};
enum CtspObjective {CTSP_VMT, CTSP_TOTALDROPOFFTIME, CTSP_TOTALWAITING};
enum AssignmentObjective {AO_SERVICERATE, AO_RMT};

#include<string>
extern Algorithm ALGORITHM;
extern double alpha;
extern AssignmentObjective ASSIGNMENT_OBJECTIVE;
extern int CARSIZE;
extern Ctsp CTSP;
extern CtspObjective CTSP_OBJECTIVE;
extern std::string DATAROOT;
extern int DWELL_ALIGHT;
extern int DWELL_PICKUP;
extern std::string EDGECOST_FILE;
extern int FINAL_TIME;
extern int INITIAL_TIME;
extern int INTERVAL;
extern bool LAST_MINUTE_SERVICE;                // Feature does not work with dwell times.
extern int MAX_DETOUR;
extern int MAX_WAITING;
extern std::string REQUEST_DATA_FILE;
extern std::string RESULTS_DIRECTORY;
extern int RTV_TIMELIMIT;
extern std::string TIMEFILE;
extern std::string VEHICLE_DATA_FILE;
extern int VEHICLE_LIMIT;

void initialize(int argc, char** argv);

#endif
