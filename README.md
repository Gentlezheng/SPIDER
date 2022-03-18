# SDVN simulation platform
It is illustrated in the paper 'SPIDER: A Social Computing Inspired Predictive Routing Scheme for Softwarized Vehicular Networks'.
1. Starting the python file-testANN, and other test** you need to modify the data name to your own data file and related parameters.
2. Counterparts experiments: testTSD, testCT, testDijkstra, testIM-DOS.
3. The simulation parameters：First, we used the urban regions of Shenyang by extracting the map data from OpenStreetMap. Two maps of different sizes are selected. Then, by applying SUMO (Simulation of Urban Mobility), under the premise that the vehicle speed is set from 1 to 60 kilometers per hour, we obtained the simulated vehicle location coordinates (i.e., the vehicular trajectories). During the simulation, we execute 10 times for each experiment, and select 100 seconds of data each time. 
