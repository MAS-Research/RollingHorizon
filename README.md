# Rolling horizon framework

The software is tested on linux with G++ compiler. Optimization is written with Mosek Optimizer version 8.1.0.56 and may not be compatible with later or current versions of the Mosek.

To compile the program, simply enter the directory and run the "make" command.

To run the simulator, in the console enter

./prog x
where x is the number of threads the simulator may use in parallel. Addiitonally, you may include the following arguments, all given as keywords followed by values. For example, to run the program with 500 vehicles you would use

./prog x VEHICLE_LIMIT 500
They keywords include (more listed in file settings.cpp):

DATAROOT - (default "./data") location to look for simulation inputs
RESULTS_DIRECTORY - (default "results") location to write results to, ignores if folder not found
RH (default 0) - rolling horizon factor
VEHICLE_LIMIT - (default no limit) maximum number of vehicles to load from vehicle file.
MAX_WAITING - (default 300) maximum waiting time for served passengers
MAX_DETOUR - (default 600) maximum detour for served passengers
REQUEST_DATA_FILE - (default requests.csv) Input request file within DATAROOT/requests/
CARSIZE - (default 4) maximum number of passengers per vehicle
INITIAL_TIME - (default 0) starting time of simulation given as HHMMSS.
FINAL_TIME - (default 24000) ending time of simulation given as HHMMSS.
INTERVAL - (default 60) time that passes between subsequent assignment epochs
RTV_TIMELIMIT - (default 0) number of miliseconds the RTV graph generator can spend on each vehicle
