# Rolling horizon framework

This is the software code for the rolling horizon framework suggested in the paper below.

Offline Pickup and Delivery Problem with Time Windows via Rolling Horizon Trip-Vehicle Assignment, Y Kim, D Edirimanna, M Wilbur, P Pugliese, A Laszka, A Dubey, S Samaranayake, accepted in The 37th AAAI Conference on Artificial Intelligence

---
## prerequisite
The software is tested on linux with G++ compiler. Optimization is written with Mosek Optimizer version 8.1.0.56 and may not be compatible with later or current versions of the Mosek. One will need a MOSEK license. Here is a link for getting an [academic license for MOSEK](https://www.mosek.com/products/academic-licenses/). It takes less than 3 minutes to request and you will get an email explaining how to install it (which can be done by a simple command line).  

___
## Guideline
I) Compile the program
```
make
```
To compile the program, simply enter the directory and run the "make" command.

II) Run the simulator

To run the simulator, in the console enter
```
./prog x
```
where x is the number of threads the simulator may use in parallel. Addiitonally, you may include the following arguments, all given as keywords followed by values. 

They keywords include (more listed in file settings.cpp):

```DATAROOT``` - (default "./data") location to look for simulation inputs

```RESULTS_DIRECTORY``` - (default "results") location to write results to, ignores if folder not found

```RH``` (default 0) - rolling horizon factor

```VEHICLE_LIMIT``` - (default no limit) maximum number of vehicles to load from vehicle file

```MAX_WAITING``` - (default 300) maximum waiting time for served passengers

```MAX_DETOUR``` - (default 600) maximum detour for served passengers

```REQUEST_DATA_FILE``` - (default requests.csv) Input request file within DATAROOT/requests/

```CARSIZE``` - (default 4) maximum number of passengers per vehicle

```INITIAL_TIME``` - (default 0) starting time of simulation given as HHMMSS

```FINAL_TIME``` - (default 24000) ending time of simulation given as HHMMSS

```INTERVAL``` - (default 60) time that passes between subsequent assignment epochs

```RTV_TIMELIMIT``` - (default 0) number of miliseconds the RTV graph generator can spend on each vehicle

For example, here is an examplary configuration
```
./prog 10 DATAROOT "data_Chattanooga" RH 1 VEHICLE_LIMIT 3 CARSIZE 8 INTERVAL 900 MAX_WAITING 1800 MAX_DETOUR 1800 DWELL_PICKUP 300 DWELL_ALIGHT 300
```

III) Repeat multiple simulations

## 
