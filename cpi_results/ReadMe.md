# CPI Results


## Step by Step Guide

1. First generate a Monte-Carlo simulation using the *cpi_simulation* MATLAB scripts
2. To record the estimation output, use the [posemsg_to_file](https://github.com/rpng/posemsg_to_file) package
3. Ensure configuration of the launch file is correct
    * Ensure `imurate` and `camrate` match what the simulator generated
    * Look at the groundtruth IMU simulation file, and ensure that the `imuwait` is equal to the IMU message till it starts moving
    * Ensure path, and noise values match that of what the simulator generated
4. It is recommended to test using the `synthetic_test.launch` file to ensure that the simulation results look good
5. Run the record script (i.e., if in your ROS workspace run `./src/cpi/cpi_compare/launch/run_mc.sh`)
6. The trajectories will be saved in the posemsg_to_file log folder
7. Copy the logs into the `/data_est/` folder
8. Copy a groundtruth trajectory file (`pose_true_gt.dat`) from any of the generated simulations (they will all be the same)
9. We provide two scripts
    * `sim_error_average.m`: computes the RMSE and average RMSE over the runs for each type
    * `sim_pos_average.m`: computes the average trajectory, and groundtruth trajectory
10. Run the desired script, and look at the results




