clear all; close all;

% Spacecraft Moment of Inertia
I=[3000 0 0
   0 3000 0
   0 0 5000]; % kg-m2

% From SolidWorks
% I=[2.8135*1000 0.1235*1000 0.2059*1000
%   0.1235*1000 4.1798*1000 0.1681*1000
%   0.2059*1000 0.1681*1000 2.8708*1000]; % kg-m2

invI=inv(I);  % Precomputing for simplifying simulation in Simulink

% For Time-Domain Solution
% Initial body spin of 0; the scoped result should show hop up to constant rate from 0
wbn_init = pi/30*[0 0 0]'; 
% Initial identity quaternion 
q_init = [0 0 0 1]'; q_init = q_init/norm(q_init);
% Remember that quaternion in Matlab defined as [phi, a_1, a_2, a_3]

% Control gains;  Tune these for a result
wn = .05; %0.01;               % closed-loop frequency, rad/sec
zet = .7071; % .7071;             % closed-loop damping ratio
Kp = 2*wn^2*I;           % The factor of 2 accounts for the fact that the quaternion components are roughly half the angle of rotation
Kd = 2*zet*wn*I;

% COMMANDED RATE
w_c = [0 0 10].'; % Angular velocity input [rad/s]

% Wheel Array
Iwi=100; ww0=[0 0 0 0].'; % arbitrary (not given)
A=[eye(3) -sqrt(3)/3*[1 1 1]' ]*Iwi; pinvA=pinv(A); % Wheel Jacobian
% Wheel Limits
wmax=150/Iwi;      % 150 Nms max momentum
accelmax=2/Iwi;        % 2 Nm max torque

% SIMULINK
sim('ConstantRateSDA')

% Plotting angular velocities (should be constant
figure(1); clf; set(gcf,'color',[0.5 0.5 0.5])
subplot(3,1,1); plot(rate.time,rate.signals.values(:,1),'r'); set(gca,'color',[0 0 0]); xlabel('Time [s]'); ylabel('Roll (b_1), rad/s');
subplot(3,1,2); plot(rate.time,rate.signals.values(:,2),'m'); set(gca,'color',[0 0 0]); xlabel('Time [s]'); ylabel('Pitch (b_2), rad/s')
subplot(3,1,3); plot(rate.time,rate.signals.values(:,3),'y'); set(gca,'color',[0 0 0]); xlabel('Time [s]'); ylabel('Yaw (b_3), rad/s')