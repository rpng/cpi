# CPI Simulation

## Step by Step Guide

1. Using Gazebo, generate a trajectory of a MAV with a IMU attached to it
2. Record a ROS bag of the groundtruth IMU and POSE of the MAV (ensure IMU location is coincident with the pose)
3. Convert the ROS bag into a CSV file using [rosbag_to_csv](https://github.com/rpng/rosbag_to_csv) package
4. If needed, generate a new feature map in the area that the simulate MAV traveled in (use `SCRIPT_gen_map.m`)
5. Change the input file in `SCRIPT_gazebo_to_sim.m` to the new generated file, ensure correct rates are inputed
6. Edit the export location in `monte_carlo_gazebo.m` and run it to generate the dataset


## Measurement Noise Models

#### IMU Noise Model

Based on [kalibr's](https://github.com/ethz-asl/kalibr/wiki/IMU-Noise-Model#the-imu-noise-model) IMU noise model, we say that both the acceleration and gyroscope measurements are affected by a bias random walk and noise.
Assuming that the user has specified the *continuous time* sigma parameters, we need to also ensure that we convert these into the discrete bias and white noise terms.
We keep track of the bias from measurement to measurement, which can be initialized to some initial bias value.
Formally, we define the following equations:

<img src="https://latex.codecogs.com/gif.latex?%5Comega_m%5Bk%5D%20%3D%20%5Comega%5Bk%5D%20&plus;%20b_d%5Bk%5D%20&plus;%20n_d%5Bk%5D">

<img src="https://latex.codecogs.com/gif.latex?b_d%5Bk%5D%20%3D%20b_d%5Bk-1%5D%20&plus;%20%5Csigma_%7Bbg%7D~%5Csqrt%7B%5CDelta%20t%7D~%5Ctextrm%7Bgennoise%7D%28w%29">

<img src="https://latex.codecogs.com/gif.latex?n_d%5Bk%5D%3D%5Csigma_%7Bg%7D~%5Cfrac%7B1%7D%7B%5Csqrt%7B%5CDelta%20t%7D%7D~%5Ctextrm%7Bgennoise%7D%28w%29">

where

<img src="https://latex.codecogs.com/gif.latex?w%5Csim%5Cmathcal%7BN%7D%280%2C1%29">

<img src="https://latex.codecogs.com/gif.latex?%5Ctextrm%7Bgennoise%7D%28%5Ccdot%29%3A%20%5Ctextrm%7Bgenerate%20random%20sample%20from%20input%20distribution%7D">

Note that the above equations are for a **single** angular acceleration, this needs to be repeated for each direction (x,y,z) which should each have their own biases.
This can then can also be repeated for the acceleration measurements.

#### Camera Noise Model

We can apply noise to our camera UV points using the following equations:

<img src="https://latex.codecogs.com/gif.latex?uv_m%20%3D%20uv%20%2B%20%5Csigma_%7Bpix%7D%20~%5Ctextrm%7Bgennoise%7D%28w%29">

where 

<img src="https://latex.codecogs.com/gif.latex?w%5Csim%5Cmathcal%7BN%7D%280%2C1%29">

<img src="https://latex.codecogs.com/gif.latex?%5Ctextrm%7Bgennoise%7D%28%5Ccdot%29%3A%20%5Ctextrm%7Bgenerate%20random%20sample%20from%20input%20distribution%7D">

Note that we model the noise being added to the *normalized* UV coordinates.
In practice, this is simply one over the focal length of the camera.