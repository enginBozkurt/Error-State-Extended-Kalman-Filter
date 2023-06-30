clc; clear; close all;
% Load the data from the CSV files
mavros_data = readtable('bag_file/mavros-local_position-pose.csv');
vicon_data = readtable('bag_file/vicon-ENAE788M_m500-ENAE788M_m500.csv');
% Reset the time to start from zero
mavros_data.Time = mavros_data.Time - min(mavros_data.Time);
vicon_data.Time = vicon_data.Time - min(vicon_data.Time);
% Extract the orientation quaternions from the tables
mavros_quat = [mavros_data.pose_orientation_w, mavros_data.pose_orientation_x,
mavros_data.pose_orientation_y, mavros_data.pose_orientation_z];
vicon_quat = [vicon_data.transform_rotation_w, vicon_data.transform_rotation_x,
vicon_data.transform_rotation_y, vicon_data.transform_rotation_z];
% Align the coordinate systems
% The rotation between the coordinate systems is a 90-degree yaw (Z-axis) rotation
R = eul2rotm([0, 0, -pi/2], 'XYZ');
rotated_vicon_quat = quatmultiply(rotm2quat(R), vicon_quat);
% Convert quaternions to Euler angles for easier visualization
mavros_euler = quat2eul(mavros_quat, 'XYZ');
rotated_vicon_euler = quat2eul(rotated_vicon_quat, 'XYZ');
% Problem 2 - Plotting part
% Create a common time vector
common_time = linspace(max(min(mavros_data.Time), min(vicon_data.Time)),
min(max(mavros_data.Time), max(vicon_data.Time)), max(length(mavros_data.Time),
length(vicon_data.Time)));
% Interpolate MAVROS and Vicon data onto the common time vector
interp_mavros_euler = interp1(mavros_data.Time, mavros_euler, common_time,
'linear');
interp_rotated_vicon_euler = interp1(vicon_data.Time, rotated_vicon_euler,
common_time, 'linear');
% Plot the orientations
figure;
subplot(3, 1, 1);
plot(common_time, rad2deg(interp_mavros_euler(:, 1)), 'b', 'DisplayName', 'MAVROS
Roll');
hold on;
plot(common_time, rad2deg(interp_rotated_vicon_euler(:, 1)), 'r', 'DisplayName',
'Vicon Roll');
xlabel('Time (s)','FontSize', 14);
ylabel('Roll (deg)','FontSize', 14);
legend;
grid on;
subplot(3, 1, 2);
plot(common_time, rad2deg(interp_mavros_euler(:, 2)), 'b', 'DisplayName', 'MAVROS
Pitch');
hold on;
plot(common_time, rad2deg(interp_rotated_vicon_euler(:, 2)), 'r', 'DisplayName',
'Vicon Pitch');
xlabel('Time (s)','FontSize', 14);
ylabel('Pitch (deg)','FontSize', 14);
legend;
grid on;
subplot(3, 1, 3);
plot(common_time, rad2deg(interp_mavros_euler(:, 3)-pi/2), 'b', 'DisplayName',
'MAVROS Yaw');
hold on;
plot(common_time, rad2deg(interp_rotated_vicon_euler(:, 3)), 'r', 'DisplayName',
'Vicon Yaw');
xlabel('Time (s)','FontSize', 14);
ylabel('Yaw (deg)','FontSize', 14);
legend;
grid on;
%% Problem 3 - imufilter using imu raw data
% Read IMU data
imu_data = readtable('bag_file/mavros-imu-data_raw.csv');
% Extract time, angular velocities and linear accelerations
% Reset the time to start from zero
imu_time = imu_data.Time - min(imu_data.Time);
gyro_data = [imu_data.angular_velocity_x, imu_data.angular_velocity_y,
imu_data.angular_velocity_z];
accel_data = [imu_data.linear_acceleration_x, imu_data.linear_acceleration_y,
imu_data.linear_acceleration_z];
Fs = 85; % Sampling frequency (Hz)
imu_filter = imufilter('SampleRate', Fs);
% Modify noise estimation parameters (optional)
imu_filter.GyroscopeNoise = 0.001;
imu_filter.AccelerometerNoise = 0.01;
% Process the accelerometer and gyroscope data
quat_est = zeros(length(imu_time), 4);
for i = 1:length(imu_time)
quat_obj = imu_filter(accel_data(i, :), gyro_data(i, :));
[quat_est(i, 1),quat_est(i, 2),quat_est(i, 3),quat_est(i, 4)] =
parts(quat_obj);
end
% Align the coordinate systems
% The rotation between the coordinate systems is a 90-degree roll (x-axis) rotation
R = eul2rotm([pi/2, 0, pi/2], 'XYZ');
quat_est = quatmultiply(rotm2quat(R), quat_est);
euler_est = quat2eul(quat_est, 'XYZ');
% Plot the fused data vs mavros orientation estimates
time_steps = 17000;
figure
subplot(3, 1, 1)
plot(imu_time(1:time_steps), rad2deg(euler_est(1:time_steps, 1)-pi/2), 'r',
'LineWidth', 1.5)
hold on
plot(mavros_data.Time(1:time_steps), rad2deg(mavros_euler(1:time_steps, 1)), 'b',
'LineWidth', 1.5)
hold off
xlabel('Time (s)','FontSize', 14)
ylabel('Roll (deg)','FontSize', 14)
legend('Fused', 'Mavros')
title('Roll comparison')
subplot(3, 1, 2)
plot(imu_time(1:time_steps), rad2deg(euler_est(1:time_steps, 2)), 'r', 'LineWidth',
1.5)
hold on
plot(mavros_data.Time(1:time_steps), rad2deg(mavros_euler(1:time_steps, 2)), 'b',
'LineWidth', 1.5)
hold off
xlabel('Time (s)','FontSize', 14)
ylabel('Pitch (deg)','FontSize', 14)
legend('Fused', 'Mavros')
title('Pitch comparison')
subplot(3, 1, 3)
plot(imu_time(1:time_steps), rad2deg(euler_est(1:time_steps, 3)), 'r', 'LineWidth',
1.5)
hold on
plot(mavros_data.Time(1:time_steps), rad2deg(mavros_euler(1:time_steps, 3)), 'b',
'LineWidth', 1.5)
hold off
xlabel('Time (s)','FontSize', 14)
ylabel('Yaw (deg)','FontSize', 14)
legend('Fused', 'Mavros')
title('Yaw comparison')
