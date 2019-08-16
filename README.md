# Error-State-Extended-Kalman-Filter
Vehicle State Estimation using Error-State Extended Kalman Filter

<p>
 In this project, I implemented the Error-State Extended Kalman Filter (ES-EKF) to localize a vehicle using data from the CARLA simulator.
</p>

- The data set contains measurements from a sensor array on a moving self-driving car.
- The sensor array consists of an IMU, a GNSS receiver, and a LiDAR, all of which provide measurements of varying reliability and at different rates.

- Our goal is to implement a state estimator that fuses the available sensor measurements to provide a reasonable estimate of the vehicle's pose and velocity. Specifically, we will be implementing the Error-State Extended Kalman Filter.

- In the main filter loop, you will first update the state and the uncertainty using IMU readings.

- Whenever a GNSS or LiDAR measurement becomes available, you will execute the appropriate common gain computation, error state, and covariance updates.

- We will have access to the actual pose and velocity values from Carla for a large section of the trajectory. So you will be able to compare your trajectory estimates to the ground truth data.
