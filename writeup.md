
# Extended Kalman Filter
---
## Rubric Points
---
Here I will consider [the rubic points](https://review.udacity.com/#!/rubrics/748/view) individually and describe how I addressed each point in my implementation.

1. RMSE of output coordinates is below threshold  

Your algorithm will be run against Dataset 1 in the simulator which is the same as "data/obj_pose-laser-radar-synthetic-input.txt" in the repository. We'll collect the positions that your algorithm outputs and compare them to ground truth data. Your px, py, vx, and vy RMSE should be less than or equal to the values [.11, .11, 0.52, 0.52].

1. Sensor Fusion algorithm follows the general processing flow.  

While you may be creative with your implementation, there is a well-defined set of steps that must take place in order to successfully build a Kalman Filter. As such, your project should follow the algorithm as described in the preceding lesson.

1. Kalman Filter algorithm handles the first measurements appropriately.  

Your algorithm should use the first measurements to initialize the state vectors and covariance matrices.

1. Kalman Filter algorithm first predicts then updates.  

Upon receiving a measurement after the first, the algorithm should predict object position to the current timestep and then update the prediction using the new measurement.

1. Kalman Filter can handle radar and lidar measurements.  

Your algorithm sets up the appropriate matrices given the type of measurement and calls the correct measurement function for a given sensor type.
