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

## Solution Approach
![Screenshot_3](https://user-images.githubusercontent.com/30608533/63162879-5d6c8d80-c02c-11e9-91d4-f7f75809e3bb.jpg)

![Screenshot_1](https://user-images.githubusercontent.com/30608533/63163027-e5eb2e00-c02c-11e9-82b2-b2e23cef8ecd.jpg)

![Screenshot_2](https://user-images.githubusercontent.com/30608533/63163043-f3081d00-c02c-11e9-9b67-71fbb0184fe2.jpg)

![Screenshot_4](https://user-images.githubusercontent.com/30608533/63163048-f8656780-c02c-11e9-982e-8bba041c3c3c.jpg)


