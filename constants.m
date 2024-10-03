clear all
close all

% Inertia Matrix
I = [3000 0 0;
     0 4000 0;
     0 0 5000];
invI = inv(I);  % Precomputing this inverse to eliminate needless computation in the sim

% Initial Angular Velocity (non-zero for small initial rotation)
wbn_init = pi/180 * [0.1; 0.2; 0.15];  % Small initial angular velocities (in rad/sec)

% Initial Quaternion (normalized)
q_init = [-0.5 -0.5 -0.5 0.5];
%%q_init = q_init / norm(q_init);  % Normalize the quaternion

% Control gains
wn = 0.05;  % Closed-loop frequency, rad/sec (slightly faster response)
zet = 0.7071;  % Closed-loop damping ratio
Kp = 2 * wn^2 * I;  % Proportional gain
Kd = 2 * zet * wn * I;  % Derivative gain

% Commanded attitude and rate
q_c = [-0.2; -0.5; -0.5 ; 0.5;];
q_c = q_c / norm(q_c);  % Normalize the commanded quaternion
w_c = [0; 0 ; 0];  % Commanded angular rates in rad/sec

% Simulate the system
sim('slew')

% Plot the results
figure(1)
clf
set(gcf, 'color', [0.5 0.5 0.5])

subplot(3,1,1)
plot(rate.time, rate.signals.values(:,1), 'r')
set(gca, 'color', [0 0 0])
ylabel('Roll (b_1), rad/s')

subplot(3,1,2)
plot(rate.time, rate.signals.values(:,2), 'm')
set(gca, 'color', [0 0 0])
ylabel('Pitch (b_2), rad/s')

subplot(3,1,3)
plot(rate.time, rate.signals.values(:,3), 'y')
set(gca, 'color', [0 0 0])
ylabel('Yaw (b_3), rad/s')
