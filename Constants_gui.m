% INERTIAS for Reaction wheels and SC
% Ensure directions are good
% Show constant rate next week
% It will wander off 
% Test in simulation with correct inertias, tune inertias, etc.
%   Any errors would therefore be 
%   Take measurements from the magnetometer, 
%   NEED 2 VECTORS, FIRST ONE IS DOWN WITH THE GRAVITY; pick another but they are all pretty bad to choose 
%   Magnetic field and down? for the 2 vectors
%   TRIAD solution using vectors 
%   Need to know Orientation of IMU (XYZ relative to roll, pitvch, yaw of the vehicle)
%   

% For plotting in real time with scopes
%   Open scopes before even running so it does not have to process opening
%   at the same time as all the calculations.
%   Signal specification block before it goes into scope. Adjust signal
%   sample time
%   Can also set limits of the scaling in the scope so that SImulink doesnt
%   take a long time to autoscale

% Do physics of moving pendulum of the center of mass being below the
% rotation point in the simulation

% PD control. Rate control first. Rate go to zero, gain can be anything for
% the physical hardware system. After rate control works, then do the
% proportional gain. Then tuning

% Need to do discrete time instead of continuous time for slew mode. Do
% Z^{-1} in simulink instead of 1/s.

% Enabled subsystem.

clear all; close all;

I=[3000 0 0
   0 4000 0
   0 0 5000]; 
invI=inv(I);  % precomputing this inverse eliminates needless computation in the sim

% For Time-Domain Solution
wbn_init=pi/30*[0 0 0]';
q_init=[0 0 0 1]'; q_init=q_init/norm(q_init);
%
% Control gains
wn=0.01;               % closed-loop frequency, rad/sec
zet=.7071;             % closed-loop damping ratio
Kp=2*wn^2*I;           % The factor of 2 accounts for the fact that the quaternion components are roughly half the angle of rotation
Kd=2*zet*wn*I;

%%% EVERYTHING BELOW THIS IS NEW.
runSimulink = input('Run the Simulink model? (Y/N): ', 's');
if strcmpi(runSimulink, 'Y')
    % LampFunction(); %% Simplified Mode Selection
    ACSModeSequence(); %% Input initial conditions, etc.
    open_system('SimulatorSpencer'); sim('SimulatorSpencer');  
end

%% Simplified Mode Selection
%{
function LampFunction()
    fprintf('Available Modes:\n'); fprintf('0.\n1.\n2.\n3.\n4.'); % Add 3, 4 when get this working.
    LampModes = input('Enter modes to run in an array (e.g., [2, 3, 5], ensure to put bracket): '); 
    if isempty(LampModes)
        error('No modes selected.');
    end

    for i = 1:length(LampModes)
        CheckBox = LampModes(i); %(i);  % Current mode
        assignin('base', 'CheckBox', CheckBox); % Update base workspace
    end
end
%}