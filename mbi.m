% Set up and integrate testbed w/ mass balancers
%{
% Mobile Platform
clc; clear; close all;

% Define grid parameters
x_min = -6; x_max = 6; % estimate room to be 12 m space? drawing is odd lol
y_min = -6; y_max = 6;
x_step = 2;  % Spacing for vertical lines
y_step = 2;  % Spacing for horizontal lines

figure; hold on; set(gca, 'Color', [0.8 0.8 0.8]); % Set background to light gray
xlim([x_min, x_max]); ylim([y_min, y_max]);
xlabel('X'); ylabel('Y');
title('Mobile Platform Grid');

for x = x_min:x_step:x_max
    plot([x x], [y_min y_max], 'c-', 'LineWidth', 1.5);
end
for y = y_min:y_step:y_max
    plot([x_min x_max], [y y], 'g-', 'LineWidth', 1.5);
end
for x = x_min:x_step:x_max
    for y = y_min:y_step:y_max
        plot(x, y, 'ko', 'MarkerSize', 3, 'MarkerFaceColor', 'k'); % Small black dots
    end
end

qr_dots = []; % Store plot handles for legend
for x = x_min:x_step:x_max
    for y = y_min:y_step:y_max
        qr_dots = [qr_dots, plot(x, y, 'ko', 'MarkerSize', 3, 'MarkerFaceColor', 'k')]; % Small black dots
    end
end

% Place a larger black dot at the center as the stationary satellite
stationary_sat = plot(0, 0, 'ko', 'MarkerSize', 12, 'MarkerFaceColor', 'k');

% Represent a mobile platform at one of the intersections
mobile_x = 0; mobile_y = 2; 
mobile_platform = plot(mobile_x, mobile_y, 'ro', 'MarkerSize', 6, 'MarkerFaceColor', 'r');



% Add legend
legend([qr_dots(1), stationary_sat, mobile_platform], ...
       {'QR Codes', 'Stationary Sat', 'Mobile Platform'}, ...
       'Location', 'northeast');
%}


% s variable in the Jacobian where we put the new unit vectors in it
% Say the first aligns exactly with the IMU. Z is down and if we say x is
% tangent to the IMU and y is pointing into the simulator. Number our
% mass balancers from 1 to 3 clockwise, they would be 120 degrees apart
% from one another and can simply do coordinate transformation. Z would
% always be the same. In this sense, the first index of r would be so
% aligned with tilt theta from z to have z' be the vertical distance that
% it travels and y' would be the horizontal distance that the balancers
% travel. From drawing, hypothetically have 12 in of horizontal travel and
% 6 in vertical travel, makes an angle 26.56 degree. So, we have 

% Global Variables for the System

global I Id r0 m s g invI invId c wref pref tref vref Kp Ki Kpj ...
    Kij Kv Kd J mr Kv Kd hmax hdmax dmax vmax amax
% globals that go unused: vref, tref, 

% Define Inertia of the satellite and damper (masses which are alot smaller)
I = [15, .25, .20; 
    .25, 20, -.15; 
    .20, -.15, 25];

Id = eye(3)*trace(I)*.003; % damper
c=trace(Id)*.03;    % Damper from little air drag

% Taking Inverse of inertia matrices just to simplify computation later
invI=inv(I); invId=inv(Id);

% Gravity vector and 3 mass balancers
g = [0, 0, -9.81].'; 
m(1) = 10; m(2) = 10; m(3) = 10;

% Mass of reaction wheels
mr = 200*[-.002, .002, -.02].';
%mr=400*[ 0 0 .001].'; % Sanity check: high CG, +X rate -> +X torque & momentum
%mr=400*[ 0 0 0].'; % Sanity check: high CG, +X rate -> +X torque & momentum

% Initial mass locations. We can define these WRT to the CoM for the entire
% satellite. They may be at the bottom of their positions so it will always
% be consistent or it will be a read value. This can be compiled into the
% Arduino so maybe like .getPosition(); in Arduino but would need a sensor
r0(1:3,1)=[0 0 0]'; r0(1:3,2)=[0 0 0]'; r0(1:3,3)=[0 0 0]';

s(1:3,1) = [ 1, .1, .3]'; s(1:3,1) = s(1:3,1)/norm(s(1:3,1));   % Max travel is 0.15 m
s(1:3,2) = [.3, 1, .3]'; s(1:3,2) = s(1:3,2)/norm(s(1:3,2));   % Max speed is 0.3 m/sec
s(1:3,3) = [.1, .3,  1]'; s(1:3,3) = s(1:3,3)/norm(s(1:3,3));

% Initial Conditions
% Mass (Balancer) Position and Mass (Balancer) Velocity
d_init = [0, 0, 0]'; dd_init=[0, 0, 0]';

% Quaternion, Angular Momentum, Angular Rate of Satellite
q_init=[0 0 0 1]'; h_init=[0 0 0]'; wbn_init=[0 0 0]';

% wbn_init=[.01 0 0]'; % Sanity check: high CG, +X rate -> +X torque & momentum
wdn_init = wbn_init; % Just also sets the initial rate of the reaction wheels to 0.

% Saturation Limits of Angular Momentum for Body and Damper, dmax, ddmax?
hmax=0.5; hdmax=0.5; dmax=0.15; vmax=0.3; amax=20;

% Control Gains
Kp = -eye(3)*5.5; Ki = -eye(3)*0.5;

% CM compensation, jerk?
% This defines max change in acceleration? Cannot have instantaneous
% acceleration change greater than this?
Kpj = -eye(3)*.6; Kij = -eye(3)*.002;

% Balancer velocity trim Kd -> bandwidth, Kv -> damping
Kv = -eye(3)*.008; Kd = -eye(3)*.00002; 

wref=zeros(3,1); pref=zeros(3,1); %pref=[0.1 0.1 0.1]';
tref=zeros(3,1); vref=zeros(3,1);

% Jacobian for balancer effectiveness (gX factored out)
J=-[m(1)*s(:,1), m(2)*s(:,2), m(3)*s(:,3)];
CGref=[0 0 0]';

T0=0; Tfinal=300;
x0=[wbn_init; wdn_init; h_init; q_init; d_init; dd_init];

%options = odeset('RelTol',1.e-3);
%[t,x] = ode45('mbf', [T0:0.05:Tfinal], x0, options);
options = odeset('OutputFcn',@odeplot);
figure(1); clf;
[t,x] = ode45('mbf', [T0:0.5:Tfinal], x0,options);

trq = [diff(x(:,7))./diff(t) diff(x(:,8))./diff(t) diff(x(:,9))./diff(t)];
figure(2); clf;
subplot(3, 2, 1); plot(t,x(:,1:3)); title('Body Rates'); grid; 
subplot(3,2,2); plot(t,x(:,10:13)); title('Quaternion'); grid;
subplot(3,2,3); plot(t,x(:,7:9)); title('CMG Momentum'); grid;
subplot(3,2,4); plot(t(2:end),trq); title('CMG Torque'); grid;
subplot(3,2,5); plot(t,x(:,14:16)); title('Balancer Position'); grid;
subplot(3,2,6); plot(t,x(:,17:19)); title('Balancer Velocity'); grid;
