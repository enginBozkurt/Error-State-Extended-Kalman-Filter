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

![Screenshot_5](https://user-images.githubusercontent.com/30608533/63163556-64949b00-c02e-11e9-956b-a976193cdda0.jpg)

![Screenshot_6](https://user-images.githubusercontent.com/30608533/63163561-69594f00-c02e-11e9-9dbd-2e0ea9af66ef.jpg)

![Screenshot_7](https://user-images.githubusercontent.com/30608533/63163569-6e1e0300-c02e-11e9-9d63-019ac33c13fb.jpg)

![Screenshot_8](https://user-images.githubusercontent.com/30608533/63163581-75dda780-c02e-11e9-8802-4fcfb0927d4b.jpg)

![Screenshot_9](https://user-images.githubusercontent.com/30608533/63163592-7d9d4c00-c02e-11e9-8743-a0a33d8d5396.jpg)

![Screenshot_10](https://user-images.githubusercontent.com/30608533/63163602-82fa9680-c02e-11e9-9709-e079a5462459.jpg)


# Running 

- To run es_ekf.py, simply call **python es_ekf.py** from the command line or 'run es_ekf.py' from within an interactive shell.

- As the code runs, some visualizations (plots) will already appear for you, including a plot of the ground truth trajectory, a plot of the ground truth trajectory compared to your estimated trajectory, and six error plots.



# Final Notes:
- This is a module assignment project from State Estimation and Localization course of **Self-Driving Cars Specialization by University of Toronto.**

- All of the data from Carla Simulator contained as the Python pickle (pkl) file inside the Data folder

- The data folder contains the data you will use for the project, and the rotations.py file contains a Quaternion class

