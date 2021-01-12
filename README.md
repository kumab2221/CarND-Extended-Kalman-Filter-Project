# Extended Kalman Filter Project Starter Code  
[![Udacity - Self-Driving Car NanoDegree](https://s3.amazonaws.com/udacity-sdc/github/shield-carnd.svg)](http://www.udacity.com/drive)


## Overview  
---
In this project you will utilize a kalman filter to estimate the state of a moving object of interest with noisy lidar and radar measurements.

## Description  
---
- This Project can compile
- RMSE of output coordinates is below threshold
- Sensor Fusion algorithm follows the general processing flow.
- Kalman Filter algorithm handles the first measurements appropriately.
- Kalman Filter algorithm first predicts then updates.
- Kalman Filter can handle radar and lidar measurements.

## Using Library
- [Eigen](https://eigen.tuxfamily.org/index.php?title=Main_Page)  
  Template header library for matrices,vectors,etc.
  Already included in the src folder.

- [uWebSockets](https://github.com/uNetworking/uWebSockets)  
  This is dynamic library for communicating with Term2.
  To communicate with Term2, you need to compile something with a specific commit ID, and you can build your environment with the following command:
```
./install-ubuntu.sh
```
## Basic Build And Run Instructions
---
Enter the command in the top directory of the project according to the following procedure.  

1. Compile the edited code
    1. mkdir build
    1. cd build
    1. cmake ..
    1. make
    1. ./ExtendedKF
1. Start Term2 Simulator

## Requirement
---
### Environment for starting an Extended Kalman Filter Project
- cmake >= 3.5
  - All OSes: [click here for installation instructions](https://cmake.org/install/)
- make >= 4.1 (Linux, Mac), 3.81 (Windows)
  - Linux: make is installed by default on most Linux distros
  - Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  - Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
- gcc/g++ >= 5.4
  - Linux: gcc / g++ is installed by default on most Linux distros
  - Mac: same deal as make - [install Xcode command line tools](https://developer.apple.com/xcode/features/)
  - Windows: recommend using [MinGW](http://www.mingw.org/)

### Simulator  
  This project involves the Term 2 Simulator which can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases).

## Communication protocol with the simulator  
Here is the main protocol that main.cpp uses for uWebSocketIO in communicating with the simulator.  

**INPUT**: values provided by the simulator to the c++ program

["sensor_measurement"] => the measurement that the simulator observed (either lidar or radar)  

**OUTPUT**: values provided by the c++ program to the simulator  

["estimate_x"] <= kalman filter estimated position x  

["estimate_y"] <= kalman filter estimated position y  

["rmse_x"]  

["rmse_y"]  

["rmse_vx"]  

["rmse_vy"]  

## Submission  
---
- [writeup.md](./writeup.md)
- src/FusionEKF.cpp
- src/kalman_filter.cpp
- src/tools.cpp



## Code Style  
---
Please (do your best to) stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).

## Licence
---
[MIT](LICENSE)

## Author
---
[kumab2221](https://github.com/kumab2221)