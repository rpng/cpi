# CPI Compare

This package does the simulation estimation of the different preintegration methods.
This handles publishing our visualization onto ROS and the estimated poses to ROS topics for recording.
Key sections of the code will be described below.



## Step by Step Guide

1. Build your ROS workspace
2. Source your workspace
3. Launch RVIS and open the `rviz.rviz` config in the launch folder
4. Launch `roslaunch cpi cpi_compare synthetic_test.launch`
5. Look at RVIZ for trajectories


## Program Parts


* `main_estimator.cpp`: Main program that will construct the simulator reader, read in configuration variables, and has all callbacks from the simulator
* `/cpi/`: Preintegrator classes, that will compound IMU measurements and return preintegrated measurements that can be inserted into GTSAM factors.
* `/gtsam/`: All GTSAM factors and nodes, note that we use a special JPL navigation state. This also included the CPI factors that use the preintegration measurements.
* `/sim/`: Simulator loader, that will load all the .dat files.
* `/solvers/`: Contains the GTSAM graph solver, and also has a stereo feature initalizer









