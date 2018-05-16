
[Simulator Dataset 1]: ./images/sim_dataset_1.png "Simulator Dataset 1"
[Simulator Dataset 2]: ./images/sim_dataset_2.png "Simulator Dataset 2"
[Radar NIS]: ./images/radar_nis.png "Radar NIS"
[Lidar NIS]: ./images/lidar_nis.png "Lidar NIS"
[RMSE Position X]: ./images/rmse_position_x.png "RMSE Position X"
[RMSE Position Y]: ./images/rmse_position_y.png "RMSE Position Y"
[RMSE Velocity X]: ./images/rmse_velocity_x.png "RMSE Velocity X"
[RMSE Velocity Y]: ./images/rmse_velocity_y.png "RMSE Velocity Y"

# Unscented Kalman Filter Project 7

The project utilizes an Unscented Kalman Filter(UKF) to estimate the state of a moving object of interest with noisy lidar and radar measurements. Root Mean Squared Error(RMSE) values are recorded to verify how well the UKF algorithm did. The project uses a simulator provided by Udacity to send data to the program.  The program captures this data using `uWebSockets` API.  For more information visit the host site for this project at  https://github.com/udacity/CarND-Unscented-Kalman-Filter-Project.

---

## Dependencies

Dependencies are taken from the host project created by Udacity.
* cmake >= 3.5
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1 (Linux, Mac), 3.81 (Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools](https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)

## Basic Build Instructions

1. Clone this repository.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./UnscentedKF`

## Project Rubic Points

### Compiling
#### Your code should compile.
Code must compile without errors with `cmake` and `make`.
Given that we've made CMakeLists.txt as general as possible, it's recommended that you do not change it unless you can guarantee that your changes will still compile on any platform.

### Accuracy
#### px, py, vx, vy output coordinates must have an RMSE <= [.09, .10, .40, .30] when using the file: "obj_pose-laser-radar-synthetic-input.txt", which is the same data file the simulator uses for Dataset 1.

Your algorithm will be run against "obj_pose-laser-radar-synthetic-input.txt". We'll collect the positions that your algorithm outputs and compare them to ground truth data. Your px, py, vx, and vy RMSE should be less than or equal to the values [.09, .10, .40, .30].

The final RMSE values for `dataset 1` and `dataset 2` are:

RMSE Parameter  | Dataset 1  | Dataset 2   |
------|------------|-------------|
Position X  | 0.0762  | 0.1141
Position Y  | 0.0795  | 0.1060
Velocity X  | 0.3848  | 0.8013
Velocity Y  | 0.2546  | 0.5436



The screenshot below shows the final values at the end of simulation for dataset 1.

![Simulator Dataset 1]

The screenshot below shows the final values at the end of simulation for dataset 2.

![Simulator Dataset 2]

### Follows the Correct Algorithm

#### Your Sensor Fusion algorithm follows the general processing flow as taught in the preceding lessons.

While you may be creative with your implementation, there is a well-defined set of steps that must take place in order to successfully build a Kalman Filter. As such, your project should follow the algorithm as described in the preceding lesson.

My implementation of the Unscented Kalman Filter follows closely with the lesson material.

#### Your Kalman Filter algorithm handles the first measurements appropriately.

Your algorithm should use the first measurements to initialize the state vectors and covariance matrices.

The implementation initializes the state and covariance matrices based on the first data (which is the lidar for dataset 1).

#### Your Kalman Filter algorithm first predicts then updates.

Upon receiving a measurement after the first, the algorithm should predict object position to the current timestep and then update the prediction using the new measurement.

The implementation follows the process flow as show below:
* Prediction based on the current timestep
  - Generated sigma points
  - Predicted sigma points
  - Predicted mean and covariance
* Update
  - Predict measurement
  - Update state

#### Your Kalman Filter can handle radar and lidar measurements.

Your algorithm sets up the appropriate matrices given the type of measurement and calls the correct measurement function for a given sensor type.

In the `ProcessMeasurement` function, different sensor data is handled through if-else statements.

`

    if(meas_package.sensor_type_ == MeasurementPackage::RADAR)
    {
      double rho, theta, rho_dot;
      rho = meas_package.raw_measurements_[0];
      theta = meas_package.raw_measurements_[1];
      rho_dot = meas_package.raw_measurements_[2];

      x_(0) = rho*cos(theta);
      x_(1) = rho*sin(theta);
      x_(2) = sqrt(pow(rho_dot*cos(theta),2)+pow(rho_dot*sin(theta),2));
      x_(3) = 0;
      x_(4) = 0;
    }
    else if(meas_package.sensor_type_ == MeasurementPackage::LASER)
    {
      double px, py;
      px = meas_package.raw_measurements_[0];
      py = meas_package.raw_measurements_[1];

      x_(0) = px;
      x_(1) = py;
      // Changed from 0 to 1 (improved the vx and vy RMSE value)
      x_(2) = 1;
      x_(3) = 1;
      x_(4) = 1;
    }

`

### Code Efficiency

#### Your algorithm should avoid unnecessary calculations.

This is mostly a "code smell" test. Your algorithm does not need to sacrifice comprehension, stability, robustness or security for speed, however it should maintain good practice with respect to calculations.

Here are some things to avoid. This is not a complete list, but rather a few examples of inefficiencies.

Running the exact same calculation repeatedly when you can run it once, store the value and then reuse the value later.
Loops that run too many times.
Creating unnecessarily complex data structures when simpler structures work equivalently.
Unnecessary control flow checks.


### Suggestions to Make Your Project Stand Out!

#### NIS

The radar NIS graph is shown below.  There are not many peaks that exceed 7.815, so the initialization values are acquitted.
![Radar NIS]

The lidar NIS graph is shown below.  There are not many peaks that exceed 5.991, so the initialization values are acquitted.
![Lidar NIS]

#### EKF vs. UKF RMSE Values
![RMSE Position X]
![RMSE Position Y]
![RMSE Velocity X]
![RMSE Velocity Y]
